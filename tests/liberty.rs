// SPDX-License-Identifier: Apache-2.0
//! The register-clock rule against the timer's own answer, port by port, on the libraries the
//! reference suite reads. Oracle: `grt-regclk-oracle.tcl` (property `is_register_clock`, which is
//! `LibertyPort::isRegClk`), one `<name>.regclk` per library in `GRT_REGCLK_DIR`, beside the
//! library path in `<name>.lib` (a one-line file). Skipped when the variable is unset.

use std::collections::BTreeMap;
use vyges_est::liberty::LibertyClocks;

#[test]
fn every_port_is_a_register_clock_exactly_when_the_timer_says() {
    let Ok(dir) = std::env::var("GRT_REGCLK_DIR") else {
        eprintln!("GRT_REGCLK_DIR unset — skipped");
        return;
    };
    let (mut libs, mut reg_clocks) = (0, 0);
    for e in std::fs::read_dir(&dir).expect("dir") {
        let p = e.expect("entry").path();
        if p.extension().is_none_or(|x| x != "regclk") {
            continue;
        }
        let lib_path = std::fs::read_to_string(p.with_extension("lib")).expect("lib path");
        let mut ours = LibertyClocks::default();
        let lib_path = lib_path.trim();
        let text = if lib_path.ends_with(".gz") {
            String::from_utf8(std::process::Command::new("zcat").arg(lib_path).output().expect("zcat").stdout).expect("utf-8")
        } else {
            std::fs::read_to_string(lib_path).expect("lib")
        };
        ours.read(&text).expect("read");
        let text = std::fs::read_to_string(&p).expect("oracle");
        let mut want = BTreeMap::new();
        let mut total = None;
        for line in text.lines() {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f[0] == "#total" {
                total = Some((f[1].parse::<usize>().unwrap(), f[2].parse::<usize>().unwrap()));
            } else {
                want.insert((f[0].to_string(), f[1].to_string()), f[2] == "1");
            }
        }
        // The oracle's own aggregate checks this parser.
        let (n, k) = total.expect("#total");
        assert_eq!((want.len(), want.values().filter(|&&v| v).count()), (n, k), "{}: parsed rows vs the oracle's totals", p.display());
        reg_clocks += k;
        let mut diffs = Vec::new();
        for ((cell, port), &v) in &want {
            // Ports the timer lists that are no `pin` group (a cell's internal ff/latch outputs)
            // must then be no register clock either.
            let got = ours.cells.get(cell).and_then(|c| c.ports.get(port)).copied().unwrap_or(false);
            if got != v {
                diffs.push(format!("{cell}/{port}: timer {v}, ours {got}"));
            }
        }
        for (cell, c) in &ours.cells {
            for port in c.ports.keys() {
                assert!(want.contains_key(&(cell.clone(), port.clone())), "{cell}/{port}: a port the timer does not have");
            }
        }
        assert!(diffs.is_empty(), "{}: {} of {} ports differ: {:?}", p.display(), diffs.len(), n, &diffs[..diffs.len().min(20)]);
        eprintln!("{}: {n} ports, {k} register clocks — exact", p.display());
        libs += 1;
    }
    assert!(libs > 0, "no oracle in {dir}");
    // A combinational library has none; the corpus as a whole must, or the test cannot fail.
    assert!(reg_clocks > 0, "no register clock in any oracle — the test cannot fail");
}
