// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation from a JSON job.
//!
//! ```text
//! vyges-est estimate <job.json>
//! ```
//!
//! The job: `steps`, the case's commands in order as `{cmd, args}` with the arguments as Tcl
//! evaluated them (`read_lef`, `read_def`, `read_liberty [-corner c]`, `define_corners`,
//! `create_clock <sources>`, `set_propagated_clock`, `set_layer_rc`, `set_wire_rc`,
//! `estimate_parasitics -placement`); `alpha` (the Steiner builder's, 0.3 by default) and `trace`
//! (where to write the RC state and per-net decisions, `VYGE|…`).
//!
//! ⬜ The RC network and its SPEF are not built yet: a run reports the decisions only.

use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_est::liberty::LibertyClocks;
use vyges_est::placement::{estimate_wire_parasitics, trace, Branch, SttTree, Timing};
use vyges_est::rc::{Rc, Units};
use vyges_opendb::Db;

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr, alpha)`.
fn stt(x: &[i32], y: &[i32], drvr: usize, alpha: f32) -> SttTree {
    LUT.with(|lut| {
        let t = vyges_stt::make_steiner_tree(lut, x, y, drvr, alpha).0.expect("a Steiner tree");
        SttTree { deg: t.deg, branch: t.branch.iter().map(|b| Branch { x: b.x, y: b.y, n: b.n as usize }).collect() }
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
    let alpha = job["alpha"].as_f64().unwrap_or(0.3) as f32;
    let units = |lib: &LibertyClocks| -> Result<Units, String> {
        let u = lib.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    for step in job["steps"].as_array().ok_or("steps")? {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
        match cmd {
            "read_lef" => db.read_lef(args.last().ok_or("read_lef path")?).map_err(|e| e.to_string())?,
            "read_def" => db.read_def(args.last().ok_or("read_def path")?, "default").map_err(|e| e.to_string())?,
            // -corner reads a corner's library; the cells and units taken are the first read's.
            "read_liberty" => {
                let path = args.last().ok_or("read_liberty path")?;
                let text = if path.ends_with(".gz") {
                    return Err(format!("{path}: a compressed library is not modelled"));
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
                // estimateWireParasitics does nothing unless a signal capacitance resolves.
                let tech = db.tech_get_name();
                if rc.resolve(&tech, |w| &w.signal_cap).is_empty() {
                    continue;
                }
                rc.sort_clk_and_signal_layers();
                trace_text.push_str(&rc.trace(&tech));
                let timing = Timing { liberty: have_lib.then_some(&lib), clock_sources: clock_sources.clone(), propagated };
                let nets = estimate_wire_parasitics(&db, &timing, alpha, &stt)?;
                estimated += nets.len();
                trace_text.push_str(&trace(&nets));
            }
            other => return Err(format!("step {other}: not modelled")),
        }
    }
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, &trace_text).map_err(|e| format!("{path}: {e}"))?;
    }
    Ok(json!({ "tool": "vyges-est", "status": "estimated", "nets": estimated, "log": rc.log }))
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [cmd, path] if cmd == "estimate" => {
            let job: Value = match std::fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string())) {
                Ok(j) => j,
                Err(e) => {
                    println!("{}", json!({ "tool": "vyges-est", "status": "error", "reason": e }));
                    return ExitCode::from(2);
                }
            };
            match run(&job) {
                Ok(r) => {
                    println!("{r}");
                    ExitCode::SUCCESS
                }
                Err(e) => {
                    // A refusal names what is not modelled; anything else is an error.
                    let refused = e.contains("not modelled");
                    println!("{}", json!({ "tool": "vyges-est", "status": if refused { "refused" } else { "error" }, "reason": e }));
                    ExitCode::from(if refused { 1 } else { 2 })
                }
            }
        }
        _ => {
            eprintln!("usage: vyges-est estimate <job.json>");
            ExitCode::from(2)
        }
    }
}
