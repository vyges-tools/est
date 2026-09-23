// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation from a JSON job.
//!
//! ```text
//! vyges-est estimate <job.json>
//! ```
//!
//! The job: `lefs`, `def`, `liberty` (paths, in read order), `clock_ports` (every `create_clock`'s
//! source ports), `propagated` (`set_propagated_clock` on the clocks), `alpha` (the Steiner
//! builder's, 0.3 by default) and `trace` (where to write the per-net decisions, `VYGE|…`).
//!
//! ⬜ The RC network and its SPEF are not built yet: a run reports the decisions only.

use std::process::ExitCode;

use serde_json::{json, Value};
use vyges_est::liberty::LibertyClocks;
use vyges_est::placement::{estimate_wire_parasitics, trace, Branch, SttTree, Timing};
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
    for lef in job["lefs"].as_array().map(Vec::as_slice).unwrap_or(&[]) {
        db.read_lef(lef.as_str().ok_or("a LEF path")?).map_err(|e| e.to_string())?;
    }
    let def = job["def"].as_str().ok_or("def")?;
    db.read_def(def, "default").map_err(|e| e.to_string())?;
    let mut lib = LibertyClocks::default();
    let libs = job["liberty"].as_array().map(Vec::as_slice).unwrap_or(&[]);
    for path in libs {
        let path = path.as_str().ok_or("a liberty path")?;
        lib.read(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?)?;
    }
    let timing = Timing {
        liberty: (!libs.is_empty()).then_some(&lib),
        clock_sources: job["clock_ports"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default(),
        propagated: job["propagated"].as_bool().unwrap_or(false),
    };
    let alpha = job["alpha"].as_f64().unwrap_or(0.3) as f32;
    let nets = estimate_wire_parasitics(&db, &timing, alpha, &stt)?;
    if let Some(path) = job["trace"].as_str() {
        std::fs::write(path, trace(&nets)).map_err(|e| format!("{path}: {e}"))?;
    }
    Ok(json!({ "tool": "vyges-est", "status": "estimated", "nets": nets.len() }))
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
