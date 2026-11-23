// SPDX-License-Identifier: Apache-2.0
//! Test support: a hand-written design read through the same calls a `Db` answers, and a
//! generator of small designs with the shapes the timer's network reads have to get right.

use std::collections::BTreeMap;

use crate::sim::{Constraints, Pin, Source};

/// A hand-written design, read through the same [`Source`] calls a `Db` answers.
#[derive(Default, Clone)]
pub(crate) struct Design {
    /// Instances in block order, each with its master.
    pub(crate) insts: Vec<(String, String)>,
    /// Each master's terminals: name, signal type, direction.
    pub(crate) masters: BTreeMap<String, Vec<(String, String, String)>>,
    pub(crate) nets: Vec<NetDef>,
}

#[derive(Default, Clone)]
pub(crate) struct NetDef {
    pub(crate) name: String,
    pub(crate) sig: String,
    pub(crate) special: bool,
    /// Ports: name, signal type, direction.
    pub(crate) bterms: Vec<(String, String, String)>,
    /// `inst/term`.
    pub(crate) iterms: Vec<String>,
}

impl Design {
    fn net(&self, net: &str) -> &NetDef {
        self.nets.iter().find(|n| n.name == net).expect("net")
    }
    fn bterm(&self, b: &str) -> (String, String, String) {
        self.nets.iter().flat_map(|n| &n.bterms).find(|t| t.0 == b).cloned().expect("bterm")
    }
}

impl Source for Design {
    fn inst_names(&self) -> Vec<String> {
        self.insts.iter().map(|(i, _)| i.clone()).collect()
    }
    fn inst_master(&self, inst: &str) -> String {
        self.insts.iter().find(|(i, _)| i == inst).map(|(_, m)| m.clone()).unwrap_or_default()
    }
    fn master_terms(&self, master: &str) -> Vec<(String, String)> {
        self.masters.get(master).into_iter().flatten().map(|(t, s, _)| (t.clone(), s.clone())).collect()
    }
    fn mterm_io(&self, master: &str, term: &str) -> String {
        self.masters.get(master).into_iter().flatten().find(|t| t.0 == term).map(|t| t.2.clone()).unwrap_or_default()
    }
    fn net_names(&self) -> Vec<String> {
        self.nets.iter().map(|n| n.name.clone()).collect()
    }
    fn net_sigtype(&self, net: &str) -> String {
        self.net(net).sig.clone()
    }
    fn net_is_special(&self, net: &str) -> bool {
        self.net(net).special
    }
    fn net_bterms(&self, net: &str) -> Vec<String> {
        self.net(net).bterms.iter().map(|b| b.0.clone()).collect()
    }
    fn bterm_sig(&self, b: &str) -> String {
        self.bterm(b).1
    }
    fn bterm_io(&self, b: &str) -> String {
        self.bterm(b).2
    }
    fn net_iterms(&self, net: &str) -> Vec<String> {
        self.net(net).iterms.clone()
    }
}

pub(crate) const LIB: &str = r#"library (t) {
    cell (buf) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A"; timing () { related_pin : "A"; } } }
    cell (inv) { pin (A) { direction : input; } pin (ZN) { direction : output; function : "!A"; timing () { related_pin : "A"; } } }
    cell (and2) { pin (A) { direction : input; } pin (B) { direction : input; } pin (Z) { direction : output; function : "A&B"; timing () { related_pin : "A B"; } } }
    cell (or2) { pin (A) { direction : input; } pin (B) { direction : input; } pin (Z) { direction : output; function : "A|B"; timing () { related_pin : "A B"; } } }
    cell (tie0) { pin (Z) { direction : output; function : "0"; } }
    cell (tie1) { pin (Z) { direction : output; function : "1"; } }
    cell (dff) { ff ("IQ","IQN") { clocked_on : "CK"; next_state : "D"; }
        pin (CK) { direction : input; clock : true; } pin (D) { direction : input; }
        pin (Q) { direction : output; function : "IQ"; timing () { related_pin : "CK"; timing_type : rising_edge; } }
        pin (QN) { direction : output; function : "IQN"; timing () { related_pin : "CK"; timing_type : rising_edge; } } }
    cell (tbuf) { pin (A) { direction : input; } pin (EN) { direction : input; }
        pin (Z) { direction : output; function : "A"; three_state : "!EN";
            timing () { related_pin : "A"; } timing () { related_pin : "EN"; timing_type : three_state_enable; } } }
    cell (icg) { pin (CK) { direction : input; clock_gate_clock_pin : true; } pin (E) { direction : input; clock_gate_enable_pin : true; }
        pin (GCK) { direction : output; clock_gate_out_pin : true; timing () { related_pin : "CK"; } } }
    cell (bidi) { pin (A) { direction : input; } pin (EN) { direction : input; }
        pin (Y) { direction : inout; function : "A"; three_state : "!EN";
            timing () { related_pin : "A"; } timing () { related_pin : "EN"; timing_type : three_state_enable; } } } }"#;

/// A generated design over [`LIB`]'s cells (plus a master the library lacks): supply
/// terminals on every master, special and ordinary supply nets, ports both ways, unconnected
/// terminals, hierarchical names; and constraints on ports, terminals and pins the network
/// does not have.
pub(crate) fn random_design(seed: u64) -> (Design, Constraints) {
    random_design_with(seed, true)
}

/// [`random_design`], with or without the cell that has a bidirect terminal.
pub(crate) fn random_design_with(seed: u64, bidirect_cell: bool) -> (Design, Constraints) {
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut rnd = |n: usize| {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 33) as usize) % n.max(1)
    };
    let cells: [(&str, &[&str], &[&str]); 11] = [
        ("buf", &["A"], &["Z"]),
        ("inv", &["A"], &["ZN"]),
        ("and2", &["A", "B"], &["Z"]),
        ("or2", &["A", "B"], &["Z"]),
        ("tie0", &[], &["Z"]),
        ("tie1", &[], &["Z"]),
        ("dff", &["CK", "D"], &["Q", "QN"]),
        ("tbuf", &["A", "EN"], &["Z"]),
        ("icg", &["CK", "E"], &["GCK"]),
        ("nolib", &["A"], &["Z"]),
        ("bidi", &["A", "EN"], &["Y"]),
    ];
    let mut d = Design::default();
    for (m, ins, outs) in cells {
        let mut terms: Vec<(String, String, String)> = vec![("VDD".into(), "POWER".into(), "INOUT".into())];
        terms.extend(ins.iter().map(|t| (t.to_string(), "SIGNAL".to_string(), "INPUT".to_string())));
        terms.extend(outs.iter().map(|t| (t.to_string(), "SIGNAL".to_string(), if *t == "Y" { "INOUT" } else { "OUTPUT" }.to_string())));
        terms.push(("VSS".into(), "GROUND".into(), "INOUT".into()));
        d.masters.insert(m.into(), terms);
    }
    let n_inst = 1 + rnd(24);
    for k in 0..n_inst {
        let name = if rnd(4) == 0 { format!("top/sub{}/u{k}", rnd(3)) } else { format!("u{k}") };
        let n_cells = if bidirect_cell { cells.len() } else { cells.len() - 1 };
        d.insts.push((name, cells[rnd(n_cells)].0.to_string()));
    }
    let vdd_special = rnd(2) == 0;
    let mut supply = vec![
        NetDef { name: "VDD".into(), sig: "POWER".into(), special: vdd_special, ..Default::default() },
        NetDef { name: "VSS".into(), sig: "GROUND".into(), special: true, ..Default::default() },
        NetDef { name: "tie_hi".into(), sig: "POWER".into(), special: false, ..Default::default() },
        NetDef { name: "tie_lo".into(), sig: "GROUND".into(), special: false, ..Default::default() },
    ];
    // Driven nets: input ports, then instance outputs.
    let mut nets: Vec<NetDef> = Vec::new();
    for k in 0..rnd(4) {
        let p = format!("in{k}");
        nets.push(NetDef { name: format!("n_{p}"), sig: "SIGNAL".into(), bterms: vec![(p, "SIGNAL".into(), "INPUT".into())], ..Default::default() });
    }
    for (inst, m) in d.insts.clone() {
        for (t, sig, io) in d.masters[&m].clone() {
            if sig != "SIGNAL" {
                let net = if t == "VDD" { 0 } else { 1 };
                if rnd(5) != 0 {
                    supply[net].iterms.push(format!("{inst}/{t}"));
                }
            } else if io != "INPUT" && rnd(5) != 0 {
                nets.push(NetDef { name: format!("n_{inst}_{t}"), sig: "SIGNAL".into(), iterms: vec![format!("{inst}/{t}")], ..Default::default() });
            }
        }
    }
    // Loads: each input onto a driven net, a tie net, or nothing.
    for (inst, m) in d.insts.clone() {
        for (t, sig, io) in d.masters[&m].clone() {
            if sig != "SIGNAL" || io != "INPUT" {
                continue;
            }
            match rnd(10) {
                0 => {}
                1 => supply[2].iterms.push(format!("{inst}/{t}")),
                2 => supply[3].iterms.push(format!("{inst}/{t}")),
                _ if !nets.is_empty() => {
                    let k = rnd(nets.len());
                    nets[k].iterms.push(format!("{inst}/{t}"));
                }
                _ => {}
            }
        }
    }
    for k in 0..rnd(3) {
        if nets.is_empty() {
            break;
        }
        let n = rnd(nets.len());
        nets[n].bterms.push((format!("out{k}"), "SIGNAL".into(), if rnd(3) == 0 { "INOUT" } else { "OUTPUT" }.into()));
    }
    if rnd(3) == 0 {
        supply[0].bterms.push(("vdd_pad".into(), "POWER".into(), "INOUT".into()));
    }
    for n in &mut nets {
        n.iterms.reverse();
    }
    d.nets = nets;
    d.nets.extend(supply);
    let k = rnd(d.nets.len() + 1).min(d.nets.len());
    d.nets.rotate_left(k);
    // Constraints: ports, connected and unconnected terminals, pins the network lacks.
    let mut pins: Vec<Pin> = d.nets.iter().flat_map(|n| n.bterms.iter().map(|b| Pin::Port(b.0.clone()))).collect();
    for (inst, m) in &d.insts {
        pins.extend(d.masters[m].iter().map(|t| Pin::ITerm(inst.clone(), t.0.clone())));
    }
    pins.push(Pin::Port("ghost_port".into()));
    pins.push(Pin::ITerm("ghost".into(), "A".into()));
    pins.push(Pin::ITerm(d.insts[0].0.clone(), "NOT_A_TERM".into()));
    let mut c = Constraints::default();
    for _ in 0..rnd(6) {
        let p = pins[rnd(pins.len())].clone();
        let v = [Some(false), Some(true), None][rnd(3)];
        if rnd(2) == 0 { c.logic.insert(p, v) } else { c.case.insert(p, v) };
    }
    (d, c)
}
