// SPDX-License-Identifier: Apache-2.0
//! The timer's logic simulation (`Sim::ensureConstantsPropagated`), as far as `isConstant` asks:
//! which pins hold a logic 0 or 1 whatever the rest of the design does. A net whose driver is
//! constant carries no parasitic-dependent timing, so `isSkipPin` gives it no network.
//!
//! One function per stage, in the reference's call order:
//! - [`constant_pins`] — `clearSimValues`, then `seedConstants` (network constants, then the
//!   constant-function pins; SDC constants are refused before estimation), then
//!   `propagateConstants(false)`;
//! - [`Sim::set_pin_value`] — a changed value on an instance INPUT queues the instance; on a
//!   DRIVER it is set on every load of the net;
//! - [`Sim::eval_instance`] — each output: its tristate enable first, then its function with the
//!   known pin values substituted (sequential outputs are not looked through); a clock-gate output
//!   is 0 when its clock or enable is.
//!
//! A pin's value only ever goes from unknown to a constant here, so the fixed point does not
//! depend on the queue's order.

#![cfg(feature = "odb")]

use std::collections::{BTreeMap, VecDeque};
use vyges_opendb::Db;

use crate::liberty::{eval_constant, Expr, LibertyClocks};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// A pin: a top-level port, or an instance terminal.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Pin {
    Port(String),
    ITerm(String, String),
}

impl Pin {
    /// `pathName`: `inst/term`, or the port's name.
    pub fn path_name(&self) -> String {
        match self {
            Pin::Port(p) => p.clone(),
            Pin::ITerm(i, t) => format!("{i}/{t}"),
        }
    }
}

struct Sim<'a> {
    lib: &'a LibertyClocks,
    /// Each pin's net, and each net's pins.
    net_of: BTreeMap<Pin, String>,
    pins_of: BTreeMap<String, Vec<Pin>>,
    /// The database direction of each pin (`network_->direction`): a port's, or its master
    /// terminal's.
    dir: BTreeMap<Pin, String>,
    /// Each instance's master, and its terminals in the master's order.
    master: BTreeMap<String, String>,
    terms: BTreeMap<String, Vec<String>>,
    values: BTreeMap<Pin, bool>,
    queue: VecDeque<String>,
    /// The constraint value of each pin `set_case_analysis` (first) or `set_logic_*` names:
    /// `None` for a transition or don't-care, which holds no constant but still blocks one.
    constraints: BTreeMap<Pin, Option<bool>>,
    /// `propagateConstants(thru_sequentials)`: whether a sequential output takes its value from
    /// the sequential's data input (`findLogicConstants` does; `ensureConstantsPropagated` not).
    thru_sequentials: bool,
}

fn any_input(dir: &str) -> bool {
    matches!(dir, "INPUT" | "INOUT")
}

fn any_output(dir: &str) -> bool {
    matches!(dir, "OUTPUT" | "INOUT")
}

impl Sim<'_> {
    /// `Network::isDriver`: an instance output, or a top-level input port.
    fn is_driver(&self, p: &Pin) -> bool {
        let d = self.dir.get(p).map(String::as_str).unwrap_or("");
        match p {
            Pin::ITerm(..) => any_output(d),
            Pin::Port(_) => any_input(d),
        }
    }

    /// `Network::isLoad`: an instance input, or a top-level output port.
    fn is_load(&self, p: &Pin) -> bool {
        let d = self.dir.get(p).map(String::as_str).unwrap_or("");
        match p {
            Pin::ITerm(..) => any_input(d),
            Pin::Port(_) => any_output(d),
        }
    }

    /// `Sim::setPinValue`: a pin with a constraint value takes no other value (the reference
    /// warns and keeps the constraint's).
    fn set_pin_value(&mut self, pin: &Pin, value: Option<bool>) {
        if self.constraints.get(pin).is_some_and(|c| *c != value) {
            return;
        }
        if self.values.get(pin).copied() == value {
            return;
        }
        match value {
            Some(v) => self.values.insert(pin.clone(), v),
            None => self.values.remove(pin),
        };
        let d = self.dir.get(pin).cloned().unwrap_or_default();
        match pin {
            // A leaf instance's input: re-evaluate the instance (once in a row).
            Pin::ITerm(inst, _) if any_input(&d) => {
                if self.queue.back() != Some(inst) {
                    self.queue.push_back(inst.clone());
                }
            }
            _ if self.is_driver(pin) => {
                let Some(net) = self.net_of.get(pin).cloned() else { return };
                let loads: Vec<Pin> = self.pins_of.get(&net).into_iter().flatten().filter(|p| *p != pin && self.is_load(p)).cloned().collect();
                for l in loads {
                    self.set_pin_value(&l, value);
                }
            }
            _ => {}
        }
    }

    /// `setConstraintConstPins`: the `set_logic_*` values, then the `set_case_analysis` values.
    fn set_constraint_const_pins(&mut self, c: &Constraints) {
        for map in [&c.logic, &c.case] {
            for (p, v) in map {
                self.set_pin_value(p, *v);
            }
        }
    }

    /// `Sim::evalInstance(inst, thru_sequentials)`. Through sequentials, an output whose function
    /// is a lone state variable (`expr->port()`) a sequential drives is that sequential's DATA
    /// expression, inverted for its inverted output.
    fn eval_instance(&mut self, inst: &str) {
        let master = self.master.get(inst).cloned().unwrap_or_default();
        let Some(cell) = self.lib.cells.get(&master) else { return };
        let terms = self.terms.get(inst).cloned().unwrap_or_default();
        let known = |values: &BTreeMap<Pin, bool>, port: &str| values.get(&Pin::ITerm(inst.to_string(), port.to_string())).copied();
        for term in &terms {
            if !cell.directions.get(term).is_some_and(|d| matches!(d.as_str(), "output" | "inout")) {
                continue;
            }
            let value = if let Some(f) = cell.functions.get(term) {
                match cell.tristate.get(term) {
                    Some(en) => {
                        if eval_constant(en, &|p| known(&self.values, p)) == Some(true) {
                            eval_constant(f, &|p| known(&self.values, p))
                        } else {
                            None
                        }
                    }
                    None => function_value(cell, f, self.thru_sequentials, &|p| known(&self.values, p)),
                }
            } else if cell.clock_gate_out.contains(term) {
                // `clockGateOutValue`: 0 when the gated clock or the enable is 0.
                if cell.clock_gate_in.iter().any(|p| known(&self.values, p) == Some(false)) { Some(false) } else { None }
            } else {
                None
            };
            let pin = Pin::ITerm(inst.to_string(), term.clone());
            if value != self.values.get(&pin).copied() {
                self.set_pin_value(&pin, value);
            }
        }
    }
}

/// The value of an output's (non-tristate) function under the known pin values. Through
/// sequentials, a function that is a lone state variable (`expr->port()`) a sequential drives is
/// that sequential's DATA expression, inverted for its inverted output (`outputPortSequential`).
fn function_value(cell: &crate::liberty::CellClock, f: &Expr, thru_sequentials: bool, known: &dyn Fn(&str) -> Option<bool>) -> Option<bool> {
    match f {
        Expr::Port(state) if thru_sequentials && cell.seq_outputs.contains_key(state) => {
            let (data, inverted) = &cell.seq_outputs[state];
            data.as_ref().and_then(|d| eval_constant(d, known)).map(|v| v != *inverted)
        }
        _ => eval_constant(f, known),
    }
}

/// `Sim::ensureConstantsPropagated` on a freshly read design: every pin holding a constant.
pub fn constant_pins(db: &Db, lib: &LibertyClocks) -> Res<BTreeMap<Pin, bool>> {
    propagate(db, lib, false, &Constraints::default())
}

/// The constraint constants `seedConstants` reads: `set_logic_*` pins and `set_case_analysis`
/// pins, each with its value (`None`: a transition or don't-care). A pin in both takes its case
/// value (`setPinValue` asks the case analysis first).
#[derive(Clone, Debug, Default)]
pub struct Constraints {
    pub logic: BTreeMap<Pin, Option<bool>>,
    pub case: BTreeMap<Pin, Option<bool>>,
}

/// [`constant_pins`] under constraints: `Sim::ensureConstantsPropagated` on a freshly read design
/// with `set_case_analysis` / `set_logic_*` in force.
pub fn constant_pins_constrained(db: &Db, lib: &LibertyClocks, c: &Constraints) -> Res<BTreeMap<Pin, bool>> {
    propagate(db, lib, false, c)
}

/// `Sta::findLogicConstants` (`Sim::findLogicConstants`): the same seeds — the constant-function
/// pins, then the pins on constant nets; no SDC constant, which that call never reads — propagated
/// THROUGH sequentials. A pin's value only goes from unknown to a constant, so seeding order does
/// not change the fixed point.
pub fn logic_constants(db: &Db, lib: &LibertyClocks) -> Res<BTreeMap<Pin, bool>> {
    propagate(db, lib, true, &Constraints::default())
}

fn propagate(db: &Db, lib: &LibertyClocks, thru_sequentials: bool, c: &Constraints) -> Res<BTreeMap<Pin, bool>> {
    let mut constraints = c.logic.clone();
    constraints.extend(c.case.iter().map(|(p, v)| (p.clone(), *v)));
    let mut sim = Sim {
        lib,
        net_of: BTreeMap::new(),
        pins_of: BTreeMap::new(),
        dir: BTreeMap::new(),
        master: BTreeMap::new(),
        terms: BTreeMap::new(),
        values: BTreeMap::new(),
        queue: VecDeque::new(),
        constraints,
        thru_sequentials,
    };
    let mut supply: Vec<(String, bool)> = Vec::new();
    let is_supply = |sig: &str| matches!(sig, "POWER" | "GROUND");
    for net in db.net_names() {
        let sig = db.net_sigtype(&net);
        match sig.as_str() {
            // `dbNetwork::findConstantNets`: a ground net is a constant 0, a power net a 1.
            "GROUND" => supply.push((net.clone(), false)),
            "POWER" => supply.push((net.clone(), true)),
            _ => {}
        }
        // 🔑 `dbNetwork::isPGSupply`: a terminal is no network pin at all when it is a supply
        // terminal, or sits on a SPECIAL supply net — so a power grid's pins are never seeded,
        // and only a signal pin tied to an ordinary supply net becomes a constant.
        let pg_net = db.net_is_special(&net) && is_supply(&sig);
        let mut pins = Vec::new();
        for b in db.net_bterms(&net) {
            if pg_net || is_supply(&db.bterm_get_sig_type(&b)) {
                continue;
            }
            let p = Pin::Port(b.clone());
            sim.dir.insert(p.clone(), db.bterm_get_io_type(&b));
            pins.push(p);
        }
        for it in db.net_iterms(&net) {
            // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself.
            let Some((inst, mterm)) = it.rsplit_once('/') else { continue };
            if pg_net || is_supply(&db.iterm_get_sig_type(inst, mterm)) {
                continue;
            }
            let master = sim.master.entry(inst.to_string()).or_insert_with(|| db.inst_master(inst)).clone();
            let p = Pin::ITerm(inst.to_string(), mterm.to_string());
            sim.dir.insert(p.clone(), db.mterm_get_io_type(&master, mterm));
            pins.push(p);
        }
        for p in &pins {
            sim.net_of.insert(p.clone(), net.clone());
        }
        sim.pins_of.insert(net, pins);
    }
    // `ensureConstantFuncPins` walks EVERY leaf instance — a tie cell whose output reaches no net
    // still holds its constant.
    for inst in db.inst_names() {
        if let std::collections::btree_map::Entry::Vacant(e) = sim.master.entry(inst) {
            let m = db.inst_master(e.key());
            e.insert(m);
        }
    }
    let mut by_master: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (inst, master) in &sim.master {
        let terms = by_master
            .entry(master.clone())
            .or_insert_with(|| db.master_mterms(master).map(|v| v.into_iter().map(|(n, _)| n).collect()).unwrap_or_default())
            .clone();
        sim.terms.insert(inst.clone(), terms);
    }
    // seedConstants: the network's constant pins, ground nets first, then power nets.
    for value in [false, true] {
        for (net, v) in supply.iter().filter(|(_, v)| *v == value) {
            for p in sim.pins_of.get(net).cloned().unwrap_or_default() {
                sim.set_pin_value(&p, Some(*v));
            }
        }
    }
    sim.set_constraint_const_pins(c);
    // setConstFuncPins: every instance output whose function is a constant (a tie cell).
    let insts: Vec<(String, String)> = sim.master.iter().map(|(i, m)| (i.clone(), m.clone())).collect();
    for (inst, master) in insts {
        let Some(cell) = lib.cells.get(&master) else { continue };
        for term in sim.terms.get(&inst).cloned().unwrap_or_default() {
            if cell.constant_outputs.contains(&term) {
                let one = cell.functions.get(&term).is_some_and(|f| eval_constant(f, &|_| None) == Some(true));
                sim.set_pin_value(&Pin::ITerm(inst.clone(), term), Some(one));
            }
        }
    }
    // propagateConstants(false).
    while let Some(inst) = sim.queue.pop_front() {
        sim.eval_instance(&inst);
    }
    Ok(sim.values)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(body: &str) -> crate::liberty::CellClock {
        let mut l = LibertyClocks::default();
        l.read(&format!("library (t) {{ cell (c) {{ {body} }} }}")).unwrap();
        l.cells.remove("c").unwrap()
    }

    const FF: &str = r#"ff ("IQ","IQN") { clocked_on : "CK"; next_state : "D"; }
        pin (CK) { direction : input; clock : true; } pin (D) { direction : input; }
        pin (Q) { direction : output; function : "IQ"; } pin (QN) { direction : output; function : "IQN"; }"#;

    /// Rule (`Sim::evalInstance`, `findLogicConstants` = `propagateConstants(true)`): through
    /// sequentials, Q = IQ is the ff's `next_state` and QN = IQN its inverse; without, both unknown.
    #[test]
    fn a_register_output_follows_its_data_only_through_sequentials() {
        let c = cell(FF);
        let d0 = |p: &str| (p == "D").then_some(false);
        let q = c.functions["Q"].clone();
        let qn = c.functions["QN"].clone();
        assert_eq!(function_value(&c, &q, true, &d0), Some(false));
        assert_eq!(function_value(&c, &qn, true, &d0), Some(true));
        assert_eq!(function_value(&c, &q, false, &d0), None);
        // An unknown data input leaves the output unknown.
        assert_eq!(function_value(&c, &q, true, &|_| None), None);
    }

    /// A latch's `data_in` is its data the same way.
    #[test]
    fn a_latch_output_follows_data_in() {
        let c = cell(r#"latch ("IQ","IQN") { enable : "G"; data_in : "D"; }
            pin (G) { direction : input; } pin (D) { direction : input; }
            pin (Q) { direction : output; function : "IQ"; }"#);
        let d1 = |p: &str| (p == "D").then_some(true);
        assert_eq!(function_value(&c, &c.functions["Q"].clone(), true, &d1), Some(true));
    }

    /// `kld` -> u1 (buffer) -> u2 (inverter), as the Sim holds a netlist.
    fn chain(lib: &LibertyClocks, constraints: BTreeMap<Pin, Option<bool>>) -> Sim<'_> {
        let it = |i: &str, t: &str| Pin::ITerm(i.into(), t.into());
        let nets = [("kld", vec![Pin::Port("kld".into()), it("u1", "A")]), ("n1", vec![it("u1", "Z"), it("u2", "A")]), ("n2", vec![it("u2", "ZN")])];
        let mut sim = Sim {
            lib,
            net_of: BTreeMap::new(),
            pins_of: BTreeMap::new(),
            dir: BTreeMap::new(),
            master: [("u1".into(), "buf".into()), ("u2".into(), "inv".into())].into(),
            terms: [("u1".into(), vec!["A".into(), "Z".into()]), ("u2".into(), vec!["A".into(), "ZN".into()])].into(),
            values: BTreeMap::new(),
            queue: VecDeque::new(),
            constraints,
            thru_sequentials: false,
        };
        for (net, pins) in nets {
            for p in &pins {
                sim.net_of.insert(p.clone(), net.into());
                let d = match p {
                    Pin::Port(_) => "INPUT",
                    Pin::ITerm(_, t) if t == "A" => "INPUT",
                    _ => "OUTPUT",
                };
                sim.dir.insert(p.clone(), d.into());
            }
            sim.pins_of.insert(net.into(), pins);
        }
        sim
    }

    /// Rule (`Sim::seedConstants` / `setPinValue`): a `set_case_analysis` port value propagates
    /// like a tie; a pin holding a constraint value takes no other (a don't-care blocks a constant).
    #[test]
    fn a_case_value_propagates_and_a_constraint_pin_keeps_its_own() {
        let mut lib = LibertyClocks::default();
        lib.read(r#"library (t) {
            cell (buf) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A"; } }
            cell (inv) { pin (A) { direction : input; } pin (ZN) { direction : output; function : "!A"; } } }"#)
            .unwrap();
        let kld = Pin::Port("kld".into());
        let run = |constraints: BTreeMap<Pin, Option<bool>>| {
            let mut sim = chain(&lib, constraints.clone());
            sim.set_constraint_const_pins(&Constraints { logic: BTreeMap::new(), case: constraints });
            while let Some(inst) = sim.queue.pop_front() {
                sim.eval_instance(&inst);
            }
            sim.values
        };
        let free = run([(kld.clone(), Some(false))].into());
        assert_eq!(free.get(&Pin::ITerm("u2".into(), "ZN".into())), Some(&true));
        let held = run([(kld.clone(), Some(false)), (Pin::ITerm("u2".into(), "A".into()), None)].into());
        assert_eq!(held.get(&Pin::ITerm("u1".into(), "Z".into())), Some(&false));
        assert_eq!(held.get(&Pin::ITerm("u2".into(), "A".into())), None);
        assert_eq!(held.get(&Pin::ITerm("u2".into(), "ZN".into())), None);
    }
}
