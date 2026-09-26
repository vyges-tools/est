// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation from a JSON job.
//!
//! ```text
//! vyges-est estimate <job.json> [-o FILE]
//! ```
//!
//! The job: `steps`, the case's commands in order as `{cmd, args}` with the arguments as Tcl
//! evaluated them (`read_lef`, `read_def`, `read_db`, `read_liberty [-corner c]`,
//! `define_corners`, `create_clock <sources>`, `set_propagated_clock`, `set_layer_rc`,
//! `set_wire_rc`, `estimate_parasitics -placement [-spef_file F]`); `alpha` (the Steiner
//! builder's, 0.3 by default) and `trace` (where to write the RC state and the per-net decisions,
//! one `VYGE|…` line each).

use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_est::liberty::LibertyClocks;
use vyges_est::placement::{estimate_wire_parasitics, trace, Branch, SttTree, Timing};
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
    let (mut clock_sources, mut propagated) = (Vec::new(), false);
    let mut trace_text = String::new();
    let mut estimated = 0usize;
    let mut calls = 0usize;
    let mut spef_files: Vec<String> = Vec::new();
    let alpha = job["alpha"].as_f64().unwrap_or(0.3) as f32;
    let units = |lib: &LibertyClocks| -> Result<Units, String> {
        let u = lib.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    for step in job["steps"].as_array().ok_or("steps")? {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
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
            "define_corners" => rc.define_corners(&args),
            "create_clock" => clock_sources.extend(args.iter().cloned()),
            "set_propagated_clock" => propagated = true,
            "set_layer_rc" => rc.set_layer_rc(&mut db, units(&lib)?, &args)?,
            "set_wire_rc" => rc.set_wire_rc(&db, units(&lib)?, &args)?,
            "estimate_parasitics" => {
                if !args.iter().any(|a| a == "-placement") {
                    return Err("estimate_parasitics without -placement: not modelled".into());
                }
                // check_corner_wire_caps (EST-0018): a corner with no signal wire capacitance, and
                // nothing is estimated at all; estimateWireParasitics then also needs one resolved.
                let tech = db.tech_get_name();
                let zero: Vec<usize> = (0..rc.scenes.len()).filter(|&k| { let v = rc.resolved(&tech, k); (v[2] + v[3]) / 2.0 == 0.0 }).collect();
                if !zero.is_empty() || rc.resolve(&tech, |w| &w.signal_cap).is_empty() {
                    continue;
                }
                calls += 1;
                rc.sort_clk_and_signal_layers();
                trace_text.push_str(&rc.trace(&tech));
                let timing = Timing { liberty: have_lib.then_some(&lib), clock_sources: clock_sources.clone(), propagated };
                let nets = estimate_wire_parasitics(&db, &timing, alpha, &stt)?;
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
    }
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, &trace_text).map_err(|e| format!("{path}: {e}"))?;
    }
    // ⛔ A run that estimated no net is VACUOUS, never `estimated`: a pass word must not come
    // from a run that did nothing. That includes a correct nothing — EST-0018, where the reference
    // estimates nothing either — because the word asserts that work was done.
    let status = if estimated == 0 { "vacuous" } else { "estimated" };
    Ok(json!({ "tool": "vyges-est", "status": status, "estimates": calls, "nets": estimated,
               "spef_files": spef_files, "log": rc.log }))
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
              read_lef, read_def, read_db, read_liberty [-corner c], define_corners,
              create_clock <sources>, set_propagated_clock, set_layer_rc, set_wire_rc,
              estimate_parasitics -placement [-spef_file F]
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
    "Correlated against OpenROAD at pin da9f29f18b6487825aa880597176e0fa97110b31, 2026-09-23, comparing every SPEF byte for byte against a fresh reference run: 8 of 9 cases of the reference's own estimate_parasitics -placement tests, and 180 of 186 cases across the rest of its regression suite that call it, 21,289 nets, 0 differing lines. The one own case not scored, prima_net_recycle, edits the design through the database's Tcl API and selects a delay calculator, which a job cannot express. Of the six corpus cases not scored, four are refusals named below, one sets set_layer_rc after a set_wire_rc read from the database (an ordering the job replay does not reproduce), and one fails in the reference itself. A NUMBER HERE MEANS NOTHING WITHOUT THE BUILD: the pin is part of the claim.",
    "Only -placement is modelled. estimate_parasitics -global_routing is refused as a command; the library's global-route network (the wire module) is used by the router, not by this command.",
    "REFUSED rather than approximated: liberty bus, bundle, ff_bank and latch_bank groups; a net driven by a tie cell's constant output; set_case_analysis, set_logic_*, set_disable_timing, create_generated_clock, set_sense and set_clock_sense, which change which pins are clocks without touching the database.",
    "Units, cell clock pins and constant outputs come from the FIRST liberty library read, as the timer's do. RC commands before any liberty read are refused: the timer's default units are not modelled.",
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

fn main() -> ExitCode {
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
    use super::{describe, exit_for, CRATE_PIN, DESCRIBE, PIN_TOKEN, USAGE};

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
        for step in ["read_lef", "read_def", "read_db", "read_liberty", "define_corners", "create_clock",
                     "set_propagated_clock", "set_layer_rc", "set_wire_rc", "estimate_parasitics"] {
            assert!(USAGE.contains(step), "USAGE does not name the `{step}` step");
            assert!(include_str!("main.rs").contains(&format!("\"{step}\" =>")), "`{step}` is not a step");
        }
    }
}
