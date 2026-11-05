// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation from a JSON job.
//!
//! ```text
//! vyges-est estimate <job.json> [-o FILE]
//! ```
//!
//! The job: `steps`, the case's commands in order as `{cmd, args}` with the arguments as Tcl
//! evaluated them (`read_lef`, `read_def`, `read_db`, `read_liberty [-corner c]`,
//! `define_corners`, `set_mode <name>`, `define_scene <name> [-mode m]`, `create_clock [-name N] <sources>`, `set_propagated_clock <objects>`,
//! `set_clock_latency <objects>`, `set_layer_rc`,
//! `set_wire_rc`, `estimate_parasitics -placement [-spef_file F]`); `alpha` (the Steiner
//! builder's, 0.3 by default) and `trace` (where to write the RC state and the per-net decisions,
//! one `VYGE|…` line each).

use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_est::liberty::LibertyClocks;
use vyges_est::modes::Modes;
use vyges_est::placement::{estimate_wire_parasitics_with_skips, skips_with_clocks, trace, Branch, ClockObject, Clocks, SttTree, Timing};
use vyges_est::network::{self, NetCtx};
use vyges_est::placement::Decision;
use vyges_est::rc::{Rc, Units};
use vyges_est::spef::{self, SpefUnits};
use vyges_opendb::Db;

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`.
fn stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> SttTree {
    LUT.with(|lut| {
        let t = vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree");
        SttTree { deg: t.deg, branch: t.branch.iter().map(|b| Branch { x: b.x, y: b.y, n: b.n }).collect() }
    })
}

fn run(job: &Value) -> Result<Value, String> {
    let mut db = Db::new();
    let mut lib = LibertyClocks::default();
    let mut have_lib = false;
    let mut rc = Rc::new();
    let mut modes = Modes::default();
    let mut trace_text = String::new();
    let mut estimated = 0usize;
    let mut calls = 0usize;
    let mut spef_files: Vec<String> = Vec::new();
    let alpha = job["alpha"].as_f64().unwrap_or(0.3) as f32;
    let units = |lib: &LibertyClocks| -> Result<Units, String> {
        let u = lib.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    // Each step's outcome, when it has one to report: a query's value, or the error a `catch` step
    // raised. A step marked `catch` (a case's `check_error`, `catch { … }`) that fails is recorded
    // and the job goes on, as the script does; any other failing step ends the job.
    let mut outcomes: Vec<Value> = Vec::new();
    // The parasitics' source (`EstimateParasitics::parasiticsSource`): SPEF once one is read, else
    // the last estimate's (`parasitics_src_`), `none` before any.
    let mut spef_read = false;
    let mut source: &'static str = "none";
    for (index, step) in job["steps"].as_array().ok_or("steps")?.iter().enumerate() {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
        let catch = step["catch"].as_bool().unwrap_or(false);
        let outcome: Result<Option<String>, String> = (|| {
        match cmd {
            "read_lef" => {
                let path = args.last().ok_or("read_lef path")?;
                db.read_lef(path).map_err(|e| format!("{path}: {e}"))?
            }
            "read_def" => {
                let path = args.last().ok_or("read_def path")?;
                db.read_def(path, "default").map_err(|e| format!("{path}: {e}"))?
            }
            // A prepared design: the database as it stood just before the estimate.
            "read_db" => {
                let path = args.last().ok_or("read_db path")?;
                db = Db::open(path).map_err(|e| format!("{path}: {e}"))?
            }
            // -corner reads a corner's library; the cells and units taken are the first read's.
            "read_liberty" => {
                let path = args.last().ok_or("read_liberty path")?;
                let text = if path.ends_with(".gz") {
                    use std::io::Read;
                    let mut s = String::new();
                    flate2::read::MultiGzDecoder::new(std::fs::File::open(path).map_err(|e| format!("{path}: {e}"))?)
                        .read_to_string(&mut s)
                        .map_err(|e| format!("{path}: {e}"))?;
                    s
                } else {
                    std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?
                };
                lib.read(&text)?;
                have_lib = true;
            }
            "define_corners" => modes.define_corners(&mut rc, &args),
            "define_scene" => modes.define_scene(&mut rc, &args)?,
            "set_mode" => {
                let [name] = args.as_slice() else { return Err("set_mode needs one mode name".into()) };
                modes.set_cmd_mode(&rc, name)?
            }
            // `-name N` first when the recorder knows it; without one the clock is named after
            // its first source, as the reference names it.
            "create_clock" => {
                let (name, sources) = match args.first().map(String::as_str) {
                    Some("-name") => (args.get(1).cloned().ok_or("create_clock -name")?, args[2..].to_vec()),
                    _ => (args.first().cloned().unwrap_or_default(), args.clone()),
                };
                modes.cmd_clocks().create_clock(&name, &sources);
            }
            "set_propagated_clock" => {
                let objects = clock_objects(&db, modes.cmd_clocks(), &args)?;
                modes.cmd_clocks().set_propagated_clock(&objects)?
            }
            "set_clock_latency" => {
                if args.iter().any(|a| a == "-source") {
                    return Ok(None); // an insertion delay: propagation is unchanged
                }
                let objects = clock_objects(&db, modes.cmd_clocks(), &args)?;
                modes.cmd_clocks().set_clock_latency(&objects)?
            }
            // `read_spef` (traced by `est::spef_read`, `spefRead`): the parasitics are now the file's;
            // what the file holds is the timer's, not est's.
            "read_spef" => spef_read = true,
            "est::parasitics_source" => return Ok(Some(parasitics_source(spef_read, source).to_string())),
            "set_layer_rc" => rc.set_layer_rc(&mut db, units(&lib)?, &args)?,
            // The argument checks first: the reference raises them before converting any value.
            "set_wire_rc" => {
                rc.check_set_wire_rc(&db, &args)?;
                rc.set_wire_rc(&db, units(&lib)?, &args)?
            }
            "estimate_parasitics" => {
                let tech = db.tech_get_name();
                if args.iter().any(|a| a == "-global_routing") {
                    return Err("estimate_parasitics -global_routing: not modelled".into());
                }
                // No flag (OpenROAD cf9e5840e36, in at 4155ac3): `designParasiticsSrc` picks the
                // source, and an estimate it would not make is an error, not a warning.
                if !args.iter().any(|a| a == "-placement") {
                    // `ensure_parasitics_cmd`: after a SPEF read the source is none, nothing estimated.
                    if !design_parasitics_src(&db, &rc, &tech, spef_read)? {
                        return Ok(None);
                    }
                }
                // check_corner_wire_caps (EST-0018): a corner with no signal wire capacitance, and
                // nothing is estimated at all; estimateWireParasitics then also needs one resolved.
                let zero: Vec<usize> = (0..rc.scenes.len()).filter(|&k| { let v = rc.resolved(&tech, k); (v[2] + v[3]) / 2.0 == 0.0 }).collect();
                for &k in &zero {
                    let warning = format!(
                        "[WARNING EST-0018] wire capacitance for corner {} is zero. Use the set_wire_rc command to set wire resistance and capacitance.",
                        rc.scenes[k]
                    );
                    rc.log.push(warning);
                }
                if !zero.is_empty() {
                    return Ok(None);
                }
                // `estimateParasitics(kPlacement)`: the SPEF is forgotten and the source set, even when
                // `estimateWireParasitics` then finds no wire RC to estimate with.
                spef_read = false;
                source = "placement";
                if rc.resolve(&tech, |w| &w.signal_cap).is_empty() {
                    return Ok(None);
                }
                calls += 1;
                rc.sort_clk_and_signal_layers();
                trace_text.push_str(&rc.trace(&tech));
                let clocks = modes.estimate_clocks()?.clone();
                let timing = Timing { liberty: have_lib.then_some(&lib), clock_sources: clocks.sources(), propagated: clocks.all_propagated() };
                // ensureClkNetwork, for every mode: the clock network is built from the SDC as it stands now.
                let skips = skips_with_clocks(&db, &timing, &clocks)?;
                modes.network_read();
                let nets = estimate_wire_parasitics_with_skips(&db, &timing, &skips, alpha, &stt)?;
                estimated += nets.len();
                trace_text.push_str(&trace(&nets));
                // -spef_file: one file per corner, `_<corner>` before `.spef` when there are several.
                if let Some(k) = args.iter().position(|a| a == "-spef_file") {
                    let path = args.get(k + 1).ok_or("-spef_file needs a path")?;
                    let u = lib.units.ok_or("SPEF units need a liberty library")?;
                    let su = SpefUnits { time: u.time, capacitance: u.capacitance, resistance: u.resistance };
                    for (c, corner) in rc.scenes.iter().enumerate() {
                        let mut file = path.clone();
                        if rc.scenes.len() > 1 {
                            let suffix = format!("_{corner}");
                            match file.find(".spef").or_else(|| file.find(".SPEF")) {
                                Some(_) => file.insert_str(file.len() - 5, &suffix),
                                None => file.push_str(&suffix),
                            }
                        }
                        let mut text = spef::header(&db, su);
                        for n in &nets {
                            let g = match &n.decision {
                                Decision::Tree { tree, non_leaf_clock, .. } => {
                                    let cx = NetCtx { db: &db, rc: &rc, tech: &tech, corner: c, is_clk: *non_leaf_clock };
                                    network::make_steiner_parasitic(&cx, &n.net, tree)?
                                }
                                Decision::Pad { pins } => network::make_pad_parasitic(&db, pins)?,
                                _ => continue,
                            };
                            text.push_str(&spef::write_net(&n.net, &g, su));
                        }
                        std::fs::write(&file, text).map_err(|e| format!("{file}: {e}"))?;
                        spef_files.push(file);
                    }
                }
            }
            other => return Err(format!("step {other}: not modelled")),
        }
        Ok(None)
        })();
        match outcome {
            Ok(Some(value)) => outcomes.push(json!({ "step": index, "value": value })),
            Ok(None) => {}
            Err(e) if catch => outcomes.push(json!({ "step": index, "error": e })),
            Err(e) => return Err(e),
        }
    }
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, &trace_text).map_err(|e| format!("{path}: {e}"))?;
    }
    // ⛔ A run that estimated no net is VACUOUS, never `estimated`: a pass word must not come
    // from a run that did nothing. That includes a correct nothing — EST-0018, where the reference
    // estimates nothing either — because the word asserts that work was done.
    let status = if estimated == 0 { "vacuous" } else { "estimated" };
    Ok(json!({ "tool": "vyges-est", "status": status, "estimates": calls, "nets": estimated,
               "spef_files": spef_files, "log": rc.log, "outcomes": outcomes }))
}

/// `designParasiticsSrc` for `estimate_parasitics` with no flag, as far as a job can reach it:
/// a job reads no SPEF and runs no router, so the source is placement unless the database holds
/// global routes (`haveRoutes` loads them from the net guides), which is refused as the
/// `-global_routing` path is. ⛔ In the reference's order: an unplaced design (EST-0009), then
/// the routes, then a scene without signal wire capacitance (EST-0014, an ERROR here where
/// `-placement` only warns EST-0018).
/// The objects a clock command names. `@clock:N` (the recorder's `get_clocks`/`all_clocks`) is a
/// clock; a name with a `/` or a port's name is a pin; otherwise a clock of that name, or every
/// clock for `all`.
fn clock_objects(db: &Db, clocks: &Clocks, args: &[String]) -> Result<Vec<ClockObject>, String> {
    let ports: std::collections::BTreeSet<String> = db.bterm_names().into_iter().collect();
    let mut out = Vec::new();
    for a in args.iter().filter(|a| !a.starts_with('-')) {
        if let Some(n) = a.strip_prefix("@clock:") {
            out.push(ClockObject::Clock(n.to_string()));
        } else if a.contains('/') || ports.contains(a) {
            out.push(ClockObject::Pin(a.clone()));
        } else if clocks.clocks().iter().any(|c| &c.name == a) {
            out.push(ClockObject::Clock(a.clone()));
        } else if a == "all" {
            out.extend(clocks.clocks().iter().map(|c| ClockObject::Clock(c.name.clone())));
        } else {
            return Err(format!("{a}: neither a clock nor a pin"));
        }
    }
    Ok(out)
}

fn design_parasitics_src(db: &Db, rc: &Rc, tech: &str, spef_read: bool) -> Result<bool, String> {
    // Parasitics read from a SPEF are not estimated over (`kNone`): checked before anything else.
    if spef_read {
        return Ok(false);
    }
    if !design_is_placed(db) {
        return Err("[ERROR EST-0009] The design is not placed: no parasitics to estimate.".into());
    }
    if db.net_names().iter().any(|n| db.num_net_get_guides(n) > 0) {
        return Err("estimate_parasitics: the database holds global routes, so the source is global routing: not modelled".into());
    }
    for (k, scene) in rc.scenes.iter().enumerate() {
        let v = rc.resolved(tech, k);
        if (v[2] + v[3]) / 2.0 == 0.0 {
            return Err(format!(
                "[ERROR EST-0014] wire capacitance for corner {scene} is zero. Use the set_wire_rc command to set wire resistance and capacitance."
            ));
        }
    }
    Ok(true)
}

/// `EstimateParasitics::parasiticsSource`: `spef` once a SPEF was read, else the last estimate's
/// source.
fn parasitics_source(spef_read: bool, source: &'static str) -> &'static str {
    if spef_read { "spef" } else { source }
}

/// `GlobalRouter::designIsPlaced`: every port on a non-special net has its first pin placed, and
/// every instance on a non-special net is placed.
fn design_is_placed(db: &Db) -> bool {
    let ports = db.bterm_names().into_iter().all(|bt| {
        let net = db.bterm_get_net(&bt);
        net.is_empty() || db.net_is_special(&net) || matches!(db.bterm_get_first_pin_placement_status(&bt).as_str(), "PLACED" | "FIRM" | "LOCKED" | "COVER")
    });
    ports
        && db.net_names().into_iter().filter(|n| !db.net_is_special(n)).all(|n| {
            db.net_iterms(&n).iter().all(|it| it.rsplit_once('/').is_none_or(|(inst, _)| db.inst_is_placed(inst)))
        })
}

const USAGE: &str = "\
vyges loom est — parasitic estimation: the RC network of every placed net, as SPEF

USAGE:
  vyges loom est estimate <job.json> [-o FILE]
  vyges loom est --describe
  vyges loom est --help
  vyges loom est --version

JOB FIELDS:
  steps     required — the commands in order, each {\"cmd\": ..., \"args\": [...]}:
              read_lef, read_def, read_db, read_liberty [-corner c], define_corners, set_mode, define_scene,
              create_clock [-name N] <sources>, set_propagated_clock <objects>,
              set_clock_latency <objects> (per clock and pin: an ideal clock's nets
              get no network), set_layer_rc, set_wire_rc,
              estimate_parasitics [-placement] [-spef_file F] (no flag: the source is
                picked as the reference does, placement here; unplaced is EST-0009, no
                wire capacitance EST-0014, nothing after read_spef),
              read_spef <file> (the parasitics become the file's), and the query
              est::parasitics_source
            a step with \"catch\": true that fails is reported and the job goes on;
            the report's `outcomes` holds each query's value and each caught error
  alpha     the Steiner builder's Prim-Dijkstra alpha (default 0.3)
  trace     write the RC state and one decision line per net to this path

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

EXIT STATUS:
  0  estimated    at least one net was estimated and every requested SPEF was written
  2  vacuous      no net was estimated — no estimate step, or every corner lacked wire
                  capacitance (EST-0018). NOT a pass.
  2  error        usage, unreadable input, or a step that failed
  3  refused      a step or an option this engine does not model — see `reason`
";

/// The pin, inherited from the crate every engine already depends on.
const CRATE_PIN: &str = vyges_opendb::OPENROAD_PIN;

/// ⛔ The `openroad_pin` FIELD is this token, substituted at print time — a hand-typed pin
/// reports what was typed, not what the binary links, and a harness comparing it against the
/// oracle it is about to launch would compare nothing. A correlation claim in the prose names the
/// commit it was MEASURED at and stays a literal.
const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, CRATE_PIN)
}

/// ⚠️ **`maturity` is `structured`**: the rung needs a pinned design in-repo that the test suite
/// runs end to end for `workflow-validated`, and this engine's correlation runs against the
/// reference outside the repository. What is not modelled goes in `provenance_limitations`.
const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "est",
  "summary": "parasitic estimation from placement: a Steiner tree per net, its RC network per corner, written as SPEF",
  "maturity": "structured",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of estimated, vacuous, refused or error. VACUOUS IS NOT ESTIMATED: no net was estimated, either because no estimate step ran or because every corner lacked signal wire capacitance (EST-0018, where the reference estimates nothing either). The declared assertion passes only on estimated. Exit status is 0 for estimated, 2 for vacuous and for error, 3 for refused.",
    "Correlated against the reference at pin 27288f5c90265ae5296b2f5a9fa6e8714fa191ad, 2026-10-12, comparing every SPEF byte for byte against a fresh reference run: 9 of 9 cases of the reference's own estimate_parasitics -placement tests with 0 differing lines, and 191 of 192 cases across the rest of its regression suite that call it, 22,484 nets exact; the one corpus case not scored fails while the reference prepares it. Earlier, at pin da9f29f18b6487825aa880597176e0fa97110b31, 2026-10-02, comparing every SPEF byte for byte against a fresh reference run: 9 of 9 cases of the reference's own estimate_parasitics -placement tests, and 185 of 186 cases across the rest of its regression suite that call it, 21,637 nets, 0 differing lines; the one corpus case not scored fails in the reference itself. The logic simulation that decides constant drivers matches the reference reading the same database pin for pin on every one of those designs. A NUMBER HERE MEANS NOTHING WITHOUT THE BUILD: the pin is part of the claim.",
    "Only the placement source is modelled: -placement, and estimate_parasitics with no flag, which picks the source as the reference does (placement for a job, which runs no router; a database holding global routes is refused) and raises the reference's errors for an unplaced design (EST-0009) and missing wire capacitance (EST-0014). estimate_parasitics -global_routing is refused as a command; the library's global-route network (the wire module) is used by the router, not by this command.",
    "REFUSED rather than approximated: liberty latch_bank groups; set_case_analysis, set_logic_*, set_disable_timing, create_generated_clock, set_sense and set_clock_sense, which change which pins are clocks without touching the database.",
    "Units, cell clock pins and constant outputs come from the FIRST liberty library read, as the timer's do. RC commands before any liberty read are refused: the timer's default units are not modelled. set_wire_rc checks its selectors before that, as the reference does: both -tech and -redistribution_layer (EST-0028) or an unknown -tech (EST-0030) is an error, and -redistribution_layer alone is refused (RDL chips are not modelled).",
    "Every value is single precision where the reference's is: Tcl doubles are narrowed to float at the command boundary, resistances and capacitances are summed as float, and SPEF values are printed as %g with six significant digits after a float division by the unit scale.",
    "The Steiner tree is built by vyges-stt, which is correlated separately."
  ],
  "invocation": {
    "args_template": ["estimate", "{job}"],
    "optional": [ { "arg": "out", "flag": "-o" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["job"],
    "properties": {
      "job": { "type": "string", "description": "path to a JSON job: {steps: [{cmd, args}], alpha, trace}" },
      "out": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["job"],
  "artifacts": [ { "role": "spef", "field": "spef_files" } ],
  "assertion": {
    "id": "parasitics-estimated",
    "field": "status",
    "pass_when": { "eq": "estimated" }
  }
}"#;

fn link(flag: &str) -> Option<(&'static str, &'static str)> {
    Some(match flag {
        "--bug-report" => (
            "Report a bug",
            "https://github.com/vyges/community/issues/new?template=bug_report_template.yaml",
        ),
        "--feature-request" => (
            "Request a feature",
            "https://github.com/vyges/community/issues/new?labels=enhancement",
        ),
        "--sponsor" => ("Sponsor Vyges", "https://github.com/sponsors/vyges-ip"),
        "--star" => ("Star this tool", "https://github.com/vyges-tools/est"),
        _ => return None,
    })
}

/// The exit status for a report's `status`, in ONE place.
fn exit_for(status: &str) -> u8 {
    match status {
        "estimated" => 0,
        "refused" => 3,
        _ => 2,
    }
}

/// The `vyges-events` causal trail: every event goes to STDERR, the report to stdout (or `-o`), so
/// a caller can parse one without the other. Codes name a situation, not a message:
///
/// | code | meaning |
/// |---|---|
/// | `EST-DONE` | the run finished; a census of what it did (nets estimated, SPEF files written). `warn` when the status is not the pass word |
/// | `EST-REFUSED` | a step this engine does not model; the reason names it |
/// | `EST-ERROR` | usage, unreadable input, or a failed write |
mod events {
    use vyges_events::{emit, Event, Severity};

    const TOOL: &str = "vyges-est";

    /// One event for the run's outcome, from its status word and the report's own fields.
    pub fn outcome(status: &str, pass: &str, reason: Option<&str>, census: &str) {
        let (code, severity) = match status {
            "refused" => ("EST-REFUSED", Severity::Error),
            "error" => ("EST-ERROR", Severity::Error),
            s if s == pass => ("EST-DONE", Severity::Info),
            _ => ("EST-DONE", Severity::Warn),
        };
        let text = match reason {
            Some(r) => format!("{status}: {r}"),
            None => format!("{status}: {census}"),
        };
        emit(&Event::new(TOOL, severity, text).with_code(code));
    }
}

pub fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    // `-o FILE` — the value is consumed here so it never reaches the positional scan.
    let mut out: Option<String> = None;
    let mut positional: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-o" => match args.get(i + 1) {
                Some(v) => {
                    out = Some(v.clone());
                    i += 1;
                }
                None => {
                    eprintln!("vyges-est: -o needs a FILE");
                    return ExitCode::from(2);
                }
            },
            "--json" => {}
            a if a.starts_with('-') && !positional.is_empty() => {
                eprintln!("vyges-est: unknown option {a}\n\n{USAGE}");
                return ExitCode::from(2);
            }
            a => positional.push(a),
        }
        i += 1;
    }
    match positional.first().copied() {
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("-V") | Some("--version") => {
            println!("vyges-est {} ({})\nCopyright (c) Vyges. Apache-2.0.", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            return ExitCode::SUCCESS;
        }
        Some("--describe") => {
            println!("{}", describe());
            return ExitCode::SUCCESS;
        }
        Some(f) if link(f).is_some() => {
            let (label, url) = link(f).unwrap();
            println!("{label}:\n  {url}");
            // Only on a terminal: launching a browser out of a pipeline would be a surprise.
            if std::io::IsTerminal::is_terminal(&std::io::stdout()) {
                let opener = if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
                let _ = std::process::Command::new(opener).arg(url).status();
            }
            return ExitCode::SUCCESS;
        }
        Some("estimate") if positional.len() == 2 => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    // ⛔ Before any database exists: libodb then logs to the events trail (stderr) only, and
    // stdout carries nothing but the JSON report a caller parses.
    vyges_opendb::init_events_logging();
    let path = positional[1];
    let report = match std::fs::read_to_string(path)
        .map_err(|e| format!("{path}: {e}"))
        .and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| format!("{path}: {e}")))
    {
        Err(e) => json!({ "tool": "vyges-est", "status": "error", "reason": e }),
        Ok(job) => match run(&job) {
            Ok(r) => r,
            // A refusal names what is not modelled; anything else is an error.
            Err(e) => json!({ "tool": "vyges-est", "status": if e.contains("not modelled") { "refused" } else { "error" }, "reason": e }),
        },
    };
    let spefs = report["spef_files"].as_array().map(Vec::len).unwrap_or(0);
    let census = format!("estimates={} nets={} spef_files={spefs}", report["estimates"], report["nets"]);
    events::outcome(report["status"].as_str().unwrap_or("error"), "estimated", report["reason"].as_str(), &census);
    let text = format!("{report}\n");
    match &out {
        Some(f) => {
            if let Err(e) = std::fs::write(f, &text) {
                eprintln!("vyges-est: {f}: {e}");
                return ExitCode::from(2);
            }
        }
        None => print!("{text}"),
    }
    ExitCode::from(exit_for(report["status"].as_str().unwrap_or("error")))
}

#[cfg(test)]
mod descriptor_tests {
    //! ⛔ A descriptor that outlives the truth is the suite's recurring defect, and nothing fails
    //! when it does. These are the gate.
    use super::{describe, exit_for, run, CRATE_PIN, DESCRIBE, PIN_TOKEN, USAGE};
    use serde_json::json;

    fn json() -> serde_json::Value {
        serde_json::from_str(DESCRIBE).expect("--describe must be valid JSON")
    }

    #[test]
    fn the_descriptor_parses_and_carries_the_contract_fields() {
        let d = json();
        for k in ["schema", "openroad_pin", "name", "summary", "maturity", "provenance_limitations",
                  "invocation", "inputs", "consumes", "artifacts", "assertion"] {
            assert!(d.get(k).is_some(), "descriptor is missing `{k}`");
        }
        assert_eq!(d["schema"], "vyges-tool-descriptor/1.1");
        assert_eq!(d["name"], "est");
    }

    /// ⛔ The ladder is a closed enum: an unknown word is not a modest claim, it degrades to
    /// `discovered` and the consumer discards the verdict.
    #[test]
    fn maturity_is_one_of_the_three_legal_rungs() {
        let m = json()["maturity"].as_str().unwrap_or_default().to_string();
        assert!(["discovered", "structured", "workflow-validated"].contains(&m.as_str()), "{m}");
    }

    #[test]
    fn the_descriptor_reports_the_pin_this_binary_was_built_against() {
        let d = describe();
        assert!(!d.contains(PIN_TOKEN), "the pin placeholder survived into the output");
        let v: serde_json::Value = serde_json::from_str(&d).expect("still valid JSON once filled in");
        assert_eq!(v["openroad_pin"], CRATE_PIN);
        assert_eq!(CRATE_PIN.len(), 40, "a full commit SHA, not an abbreviation");
        assert_eq!(json()["openroad_pin"], PIN_TOKEN, "the FIELD must be the token in the source");
    }

    /// Rule (`designParasiticsSrc`, OpenROAD cf9e5840e36): `estimate_parasitics` with no flag on
    /// an unplaced design is the error EST-0009; placed, with no wire capacitance, the error
    /// EST-0014 (where `-placement` only warns EST-0018 and estimates nothing).
    #[test]
    fn a_flagless_estimate_errors_as_the_reference_picks_its_source() {
        const LEF: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\n\
            LAYER metal1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.2 ;\n  WIDTH 0.1 ;\nEND metal1\n\
            SITE core\n  CLASS CORE ;\n  SIZE 0.2 BY 1.4 ;\nEND core\n\
            MACRO BUF\n  CLASS CORE ;\n  ORIGIN 0 0 ;\n  SIZE 1 BY 1.4 ;\n  SITE core ;\n\
            PIN A\n    DIRECTION INPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0 0 0.1 0.1 ;\n    END\n  END A\n\
            PIN Z\n    DIRECTION OUTPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0.5 0 0.6 0.1 ;\n    END\n  END Z\nEND BUF\nEND LIBRARY\n";
        let def = |status: &str| {
            format!(
                "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN top ;\nUNITS DISTANCE MICRONS 1000 ;\n\
                 DIEAREA ( 0 0 ) ( 10000 10000 ) ;\nCOMPONENTS 1 ;\n  - u1 BUF {status} ;\nEND COMPONENTS\n\
                 PINS 1 ;\n  - in + NET in + DIRECTION INPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 5000 ) N ;\nEND PINS\n\
                 NETS 1 ;\n  - in ( PIN in ) ( u1 A ) ;\nEND NETS\nEND DESIGN\n"
            )
        };
        let dir = std::env::temp_dir().join(format!("vyges-est-src-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let lef = dir.join("t.lef");
        std::fs::write(&lef, LEF).unwrap();
        let reason = |status: &str, flag: Option<&str>| {
            let d = dir.join(format!("{}.def", status.len()));
            std::fs::write(&d, def(status)).unwrap();
            let est: Vec<&str> = flag.into_iter().collect();
            let job = json!({ "steps": [
                { "cmd": "read_lef", "args": [lef.to_str().unwrap()] },
                { "cmd": "read_def", "args": [d.to_str().unwrap()] },
                { "cmd": "estimate_parasitics", "args": est },
            ]});
            run(&job)
        };
        let unplaced = reason("+ UNPLACED", None).unwrap_err();
        assert!(unplaced.starts_with("[ERROR EST-0009]"), "{unplaced}");
        let no_rc = reason("+ PLACED ( 1000 1000 ) N", None).unwrap_err();
        assert!(no_rc.starts_with("[ERROR EST-0014] wire capacitance for corner default is zero."), "{no_rc}");
        // `-placement` keeps its old behaviour: a warning, and nothing estimated
        let warned = reason("+ PLACED ( 1000 1000 ) N", Some("-placement")).unwrap();
        assert_eq!(warned["status"], "vacuous");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// One placed buffer on one input net (Nangate-like units), and the steps that load it.
    fn one_buffer(tag: &str) -> (std::path::PathBuf, Vec<serde_json::Value>) {
        const LEF: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\n\
            LAYER metal1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.2 ;\n  WIDTH 0.1 ;\nEND metal1\n\
            SITE core\n  CLASS CORE ;\n  SIZE 0.2 BY 1.4 ;\nEND core\n\
            MACRO BUF\n  CLASS CORE ;\n  ORIGIN 0 0 ;\n  SIZE 1 BY 1.4 ;\n  SITE core ;\n\
            PIN A\n    DIRECTION INPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0 0 0.1 0.1 ;\n    END\n  END A\n\
            PIN Z\n    DIRECTION OUTPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0.5 0 0.6 0.1 ;\n    END\n  END Z\nEND BUF\nEND LIBRARY\n";
        const DEF: &str = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN top ;\nUNITS DISTANCE MICRONS 1000 ;\n\
            DIEAREA ( 0 0 ) ( 10000 10000 ) ;\nCOMPONENTS 1 ;\n  - u1 BUF + PLACED ( 3000 1000 ) N ;\nEND COMPONENTS\n\
            PINS 1 ;\n  - in + NET in + DIRECTION INPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 5000 ) N ;\nEND PINS\n\
            NETS 1 ;\n  - in ( PIN in ) ( u1 A ) ;\nEND NETS\nEND DESIGN\n";
        const LIB: &str = r#"library (t) { capacitive_load_unit (1,ff); pulling_resistance_unit : "1kohm";
            cell (BUF) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A";
              timing () { related_pin : "A"; } } } }"#;
        let dir = std::env::temp_dir().join(format!("vyges-est-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut steps = Vec::new();
        for (name, text, cmd) in [("t.lef", LEF, "read_lef"), ("t.def", DEF, "read_def"), ("t.lib", LIB, "read_liberty")] {
            std::fs::write(dir.join(name), text).unwrap();
            steps.push(json!({ "cmd": cmd, "args": [dir.join(name).to_str().unwrap()] }));
        }
        (dir, steps)
    }

    /// The outcome a job reported for step `index`: its `value` or its `error`.
    fn outcome(report: &serde_json::Value, index: usize, key: &str) -> String {
        let all = report["outcomes"].as_array().expect("every report carries outcomes");
        let o = all.iter().find(|o| o["step"] == index).unwrap_or_else(|| panic!("no outcome for step {index}: {report}"));
        o[key].as_str().unwrap_or_else(|| panic!("step {index} has no {key}: {report}")).to_string()
    }

    /// Reference rule (the suite's `check_error`, test/helpers.tcl at the 4155ac3 pin): the error a
    /// checked script raises is caught and the case goes on. A step marked `catch` that fails is
    /// reported in `outcomes` and the job continues; any other failing step still ends the job.
    #[test]
    fn a_caught_step_is_reported_and_the_job_goes_on() {
        let (dir, mut steps) = one_buffer("catch");
        let spef = dir.join("after.spef").to_str().unwrap().to_string();
        let uncaught = json!({ "steps": steps.clone().into_iter()
            .chain([json!({ "cmd": "estimate_parasitics", "args": [] })]).collect::<Vec<_>>() });
        steps.push(json!({ "cmd": "estimate_parasitics", "args": [], "catch": true }));   // 3: EST-0014
        steps.push(json!({ "cmd": "set_wire_rc", "args": ["-resistance", "0.0357", "-capacitance", "0.0752"] }));
        steps.push(json!({ "cmd": "estimate_parasitics", "args": ["-spef_file", spef] }));
        let r = run(&json!({ "steps": steps })).unwrap();
        assert!(outcome(&r, 3, "error").contains("EST-0014"), "{r}");
        assert!(std::path::Path::new(&spef).exists(), "the estimate after the caught error ran: {r}");
        assert!(run(&uncaught).unwrap_err().contains("EST-0014"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Reference rules (`EstimateParasitics` at the 4155ac3 pin): `parasiticsSource` is `none`
    /// before any estimate, the last estimate's source after one, and `spef` once `read_spef` ran
    /// (`spefRead`, traced from `sta::read_spef`). A flagless estimate after a SPEF read estimates
    /// nothing (`designParasiticsSrc` returns kNone before any other check), and any estimate that
    /// runs forgets the SPEF (`estimateParasitics` clears `spef_read_`).
    #[test]
    fn a_read_spef_stops_the_flagless_estimate_until_an_explicit_one_runs() {
        let (dir, mut steps) = one_buffer("spef");
        let spef = |n: &str| dir.join(format!("{n}.spef")).to_str().unwrap().to_string();
        let q = json!({ "cmd": "est::parasitics_source", "args": [] });
        steps.push(q.clone());                                                                  // 3: none
        steps.push(json!({ "cmd": "set_wire_rc", "args": ["-resistance", "0.0357", "-capacitance", "0.0752"] }));
        steps.push(json!({ "cmd": "estimate_parasitics", "args": ["-spef_file", spef("auto")] }));
        steps.push(q.clone());                                                                  // 6: placement
        steps.push(json!({ "cmd": "read_spef", "args": [spef("auto")] }));
        steps.push(json!({ "cmd": "estimate_parasitics", "args": ["-spef_file", spef("after_read")] }));
        steps.push(q.clone());                                                                  // 9: spef
        steps.push(json!({ "cmd": "estimate_parasitics", "args": ["-placement"] }));
        steps.push(q);                                                                          // 11: placement
        let r = run(&json!({ "steps": steps })).unwrap();
        assert_eq!(outcome(&r, 3, "value"), "none");
        assert_eq!(outcome(&r, 6, "value"), "placement");
        assert!(std::path::Path::new(&spef("auto")).exists());
        assert!(!std::path::Path::new(&spef("after_read")).exists(), "after read_spef the flagless estimate writes nothing");
        assert_eq!(outcome(&r, 9, "value"), "spef");
        assert_eq!(outcome(&r, 11, "value"), "placement", "an explicit estimate forgets the SPEF");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Reference rule (OpenSTA `ClkNetwork::findClkPins` and `Sdc::setClockLatency` at the 4155ac3
    /// pin, read by `isSkipPin` when the estimate runs): the ideal clock pins are walked per clock,
    /// from the clocks that are not propagated. `set_propagated_clock [all_clocks]` then
    /// `set_clock_latency` on clk2 leaves clk1 propagated and makes clk2 ideal again, so clk2's net
    /// gets no network and clk1's does. A latency after the network is built is refused (the
    /// reference reads it stale).
    #[test]
    fn a_clock_made_ideal_by_its_latency_is_skipped_and_the_other_is_not() {
        const LEF: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\n\
            LAYER metal1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.2 ;\n  WIDTH 0.1 ;\nEND metal1\n\
            SITE core\n  CLASS CORE ;\n  SIZE 0.2 BY 1.4 ;\nEND core\n\
            MACRO BUF\n  CLASS CORE ;\n  ORIGIN 0 0 ;\n  SIZE 1 BY 1.4 ;\n  SITE core ;\n\
            PIN A\n    DIRECTION INPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0 0 0.1 0.1 ;\n    END\n  END A\n\
            PIN Z\n    DIRECTION OUTPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0.5 0 0.6 0.1 ;\n    END\n  END Z\nEND BUF\nEND LIBRARY\n";
        const DEF: &str = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN top ;\nUNITS DISTANCE MICRONS 1000 ;\n\
            DIEAREA ( 0 0 ) ( 10000 10000 ) ;\nCOMPONENTS 2 ;\n  - u1 BUF + PLACED ( 3000 1000 ) N ;\n  - u2 BUF + PLACED ( 3000 6000 ) N ;\nEND COMPONENTS\n\
            PINS 2 ;\n  - clk1 + NET clk1 + DIRECTION INPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 1000 ) N ;\n\
              - clk2 + NET clk2 + DIRECTION INPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 6000 ) N ;\nEND PINS\n\
            NETS 2 ;\n  - clk1 ( PIN clk1 ) ( u1 A ) ;\n  - clk2 ( PIN clk2 ) ( u2 A ) ;\nEND NETS\nEND DESIGN\n";
        const LIB: &str = r#"library (t) { capacitive_load_unit (1,ff); pulling_resistance_unit : "1kohm";
            cell (BUF) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A";
              timing () { related_pin : "A"; } } } }"#;
        let dir = std::env::temp_dir().join(format!("vyges-est-clk-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let (lef, def, lib, tr) = (dir.join("t.lef"), dir.join("t.def"), dir.join("t.lib"), dir.join("t.vyge"));
        std::fs::write(&lef, LEF).unwrap();
        std::fs::write(&def, DEF).unwrap();
        std::fs::write(&lib, LIB).unwrap();
        let p = |x: &std::path::PathBuf| x.to_str().unwrap().to_string();
        let job = |tail: Vec<serde_json::Value>| {
            let mut steps = vec![
                json!({ "cmd": "read_lef", "args": [p(&lef)] }),
                json!({ "cmd": "read_def", "args": [p(&def)] }),
                json!({ "cmd": "read_liberty", "args": [p(&lib)] }),
                json!({ "cmd": "create_clock", "args": ["-name", "clk1", "clk1"] }),
                json!({ "cmd": "create_clock", "args": ["-name", "clk2", "clk2"] }),
                json!({ "cmd": "set_propagated_clock", "args": ["@clock:clk1", "@clock:clk2"] }),
            ];
            steps.extend(tail);
            json!({ "steps": steps, "trace": p(&tr) })
        };
        let rc = json!({ "cmd": "set_wire_rc", "args": ["-resistance", "0.0357", "-capacitance", "0.0752"] });
        let est = json!({ "cmd": "estimate_parasitics", "args": ["-placement"] });
        let latency = json!({ "cmd": "set_clock_latency", "args": ["@clock:clk2"] });
        run(&job(vec![latency.clone(), rc.clone(), est.clone()])).unwrap();
        let t = std::fs::read_to_string(&tr).unwrap();
        assert!(t.contains("VYGE|skip|clk2\n"), "clk2 is ideal again:\n{t}");
        assert!(!t.contains("VYGE|skip|clk1\n") && t.contains("VYGE|tree|clk1|"), "clk1 stays propagated:\n{t}");
        // Without the latency both clocks are propagated: neither is skipped.
        run(&job(vec![rc.clone(), est.clone()])).unwrap();
        let t = std::fs::read_to_string(&tr).unwrap();
        assert!(!t.contains("VYGE|skip|"), "{t}");
        // A latency once the network is built is read stale by the reference: refused.
        let stale = run(&job(vec![rc, est.clone(), latency, est])).unwrap_err();
        assert!(stale.contains("stale"), "{stale}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The assertion passes on the pass word, and ONLY the pass word exits 0.
    #[test]
    fn only_estimated_passes_and_exits_zero() {
        assert_eq!(json()["assertion"]["pass_when"]["eq"], "estimated");
        assert_eq!(exit_for("estimated"), 0);
        assert_eq!(exit_for("vacuous"), 2);
        assert_eq!(exit_for("error"), 2);
        assert_eq!(exit_for("refused"), 3);
        assert_eq!(exit_for("anything else"), 2);
    }

    /// The artifact field names a field the report actually carries.
    #[test]
    fn the_artifact_field_is_in_the_report() {
        let src = include_str!("main.rs");
        let field = json()["artifacts"][0]["field"].as_str().unwrap().to_string();
        assert!(src.contains(&format!("\"{field}\": {field}")), "report has no `{field}`");
    }

    /// ⚠️ The book's CLI reference is `--help` verbatim; regenerate it when USAGE changes.
    #[test]
    fn the_book_reference_is_the_usage_verbatim() {
        let page = include_str!("../docs/src/reference/vyges-est.md");
        assert!(page.contains(&format!("```text\n{USAGE}```")), "docs/src/reference/vyges-est.md is stale");
    }

    #[test]
    fn the_usage_documents_every_exit_status_and_step() {
        for w in ["0  estimated", "2  vacuous", "2  error", "3  refused"] {
            assert!(USAGE.contains(w), "USAGE is missing `{w}`");
        }
        for step in ["read_lef", "read_def", "read_db", "read_liberty", "define_corners", "set_mode", "define_scene", "read_spef", "est::parasitics_source", "create_clock",
                     "set_propagated_clock", "set_clock_latency", "set_layer_rc", "set_wire_rc", "estimate_parasitics"] {
            assert!(USAGE.contains(step), "USAGE does not name the `{step}` step");
            assert!(include_str!("main.rs").contains(&format!("\"{step}\" =>")), "`{step}` is not a step");
        }
    }
}
