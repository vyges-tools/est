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

use crate::liberty::{eval_constant, LibertyClocks};

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

    /// `Sim::setPinValue` (no SDC constant can disagree: those are refused before estimation).
    fn set_pin_value(&mut self, pin: &Pin, value: Option<bool>) {
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

    /// `Sim::evalInstance(inst, thru_sequentials = false)`.
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
                    None => eval_constant(f, &|p| known(&self.values, p)),
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

/// `Sim::ensureConstantsPropagated` on a freshly read design: every pin holding a constant.
pub fn constant_pins(db: &Db, lib: &LibertyClocks) -> Res<BTreeMap<Pin, bool>> {
    let mut sim = Sim {
        lib,
        net_of: BTreeMap::new(),
        pins_of: BTreeMap::new(),
        dir: BTreeMap::new(),
        master: BTreeMap::new(),
        terms: BTreeMap::new(),
        values: BTreeMap::new(),
        queue: VecDeque::new(),
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
        if !sim.master.contains_key(&inst) {
            let m = db.inst_master(&inst);
            sim.master.insert(inst, m);
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
