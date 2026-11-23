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

use std::collections::{BTreeMap, HashMap, VecDeque};
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


/// A pin's direction as the network reads it: `Network::isDriver` / `isLoad` ask only whether it
/// is an input (or bidirect) and whether an output (or bidirect). A pin no net reads is neither.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Dir {
    input: bool,
    output: bool,
}

impl Dir {
    fn of(io: &str) -> Dir {
        Dir { input: matches!(io, "INPUT" | "INOUT"), output: matches!(io, "OUTPUT" | "INOUT") }
    }
}

const NO_NET: u32 = u32::MAX;

/// What [`Network::read`] asks of the design — the `Db` accessors it calls, by name. A test
/// builds a network from a hand-written design through the same read.
pub(crate) trait Source {
    fn inst_names(&self) -> Vec<String>;
    fn inst_master(&self, inst: &str) -> String;
    /// Every terminal of a master in the master's order, with its signal type.
    fn master_terms(&self, master: &str) -> Vec<(String, String)>;
    fn mterm_io(&self, master: &str, term: &str) -> String;
    fn net_names(&self) -> Vec<String>;
    fn net_sigtype(&self, net: &str) -> String;
    fn net_is_special(&self, net: &str) -> bool;
    fn net_bterms(&self, net: &str) -> Vec<String>;
    fn bterm_sig(&self, bterm: &str) -> String;
    fn bterm_io(&self, bterm: &str) -> String;
    /// `inst/term` per instance terminal of the net.
    fn net_iterms(&self, net: &str) -> Vec<String>;
}

impl Source for Db {
    fn inst_names(&self) -> Vec<String> {
        Db::inst_names(self)
    }
    fn inst_master(&self, inst: &str) -> String {
        Db::inst_master(self, inst)
    }
    fn master_terms(&self, master: &str) -> Vec<(String, String)> {
        self.master_mterms(master).unwrap_or_default()
    }
    fn mterm_io(&self, master: &str, term: &str) -> String {
        self.mterm_get_io_type(master, term)
    }
    fn net_names(&self) -> Vec<String> {
        Db::net_names(self)
    }
    fn net_sigtype(&self, net: &str) -> String {
        Db::net_sigtype(self, net)
    }
    fn net_is_special(&self, net: &str) -> bool {
        Db::net_is_special(self, net)
    }
    fn net_bterms(&self, net: &str) -> Vec<String> {
        Db::net_bterms(self, net)
    }
    fn bterm_sig(&self, bterm: &str) -> String {
        self.bterm_get_sig_type(bterm)
    }
    fn bterm_io(&self, bterm: &str) -> String {
        self.bterm_get_io_type(bterm)
    }
    fn net_iterms(&self, net: &str) -> Vec<String> {
        Db::net_iterms(self, net)
    }
}

/// The design as the simulation reads it — read once, and read alike by every simulation of the
/// same design state (only the seeds differ between them). Pins are dense indices: every leaf
/// instance's terminals in its master's order (instances in name order), then the top-level ports
/// in the order the nets name them. A simulation keys its values by index, not by name.
#[derive(Debug, Default)]
pub struct Network {
    /// Leaf instances in name order, each one's master (an index into `masters`).
    insts: Vec<String>,
    inst_master: Vec<u32>,
    masters: Vec<String>,
    /// Each master's terminals in the master's order.
    master_terms: Vec<Vec<String>>,
    /// The first pin of each instance; `term_base[i] + k` is terminal `k` of instance `i`.
    term_base: Vec<u32>,
    /// Each instance pin's instance.
    pin_inst: Vec<u32>,
    /// The top-level ports; port `k` is pin `pin_inst.len() + k`.
    ports: Vec<String>,
    /// Each pin's direction and net (`NO_NET`: no net reads it), and each net's pins in read order.
    dir: Vec<Dir>,
    net_of: Vec<u32>,
    pins_of: Vec<Vec<u32>>,
    /// The supply nets (`findConstantNets`) and their values, in net order.
    supply: Vec<(u32, bool)>,
    inst_index: HashMap<String, u32>,
    port_index: HashMap<String, u32>,
}

impl Network {
    /// The design read once: every net's network pins, each pin's direction, every leaf
    /// instance's master and terminals, and the supply nets.
    pub fn read(db: &Db) -> Res<Network> {
        Network::read_from(db)
    }

    fn read_from(db: &impl Source) -> Res<Network> {
        let mut nw = Network::default();
        let is_supply = |sig: &str| matches!(sig, "POWER" | "GROUND");
        // `ensureConstantFuncPins` walks EVERY leaf instance — a tie cell whose output reaches no
        // net still holds its constant.
        let mut insts = db.inst_names();
        insts.sort();
        insts.dedup();
        let mut master_index: HashMap<String, u32> = HashMap::new();
        // 🔑 `dbITerm::getSigType` / `getIoType` read the terminal's MTERM, so both are facts of the
        // master: read once per master, not once per instance pin.
        let mut term_supply: Vec<Vec<bool>> = Vec::new();
        let mut term_dir: Vec<Vec<Dir>> = Vec::new();
        for inst in &insts {
            let master = db.inst_master(inst);
            let m = match master_index.get(&master) {
                Some(&m) => m,
                None => {
                    let terms = db.master_terms(&master);
                    term_supply.push(terms.iter().map(|(_, sig)| is_supply(sig)).collect());
                    term_dir.push(terms.iter().map(|(t, _)| Dir::of(&db.mterm_io(&master, t))).collect());
                    nw.master_terms.push(terms.into_iter().map(|(t, _)| t).collect());
                    let m = nw.masters.len() as u32;
                    nw.masters.push(master.clone());
                    master_index.insert(master, m);
                    m
                }
            };
            let i = nw.insts.len() as u32;
            nw.inst_index.insert(inst.clone(), i);
            nw.term_base.push(nw.pin_inst.len() as u32);
            nw.pin_inst.extend(std::iter::repeat_n(i, nw.master_terms[m as usize].len()));
            nw.inst_master.push(m);
            nw.insts.push(inst.clone());
        }
        nw.term_base.push(nw.pin_inst.len() as u32);
        nw.dir = vec![Dir::default(); nw.pin_inst.len()];
        nw.net_of = vec![NO_NET; nw.pin_inst.len()];
        for net in db.net_names() {
            let n = nw.pins_of.len() as u32;
            let sig = db.net_sigtype(&net);
            match sig.as_str() {
                // `dbNetwork::findConstantNets`: a ground net is a constant 0, a power net a 1.
                "GROUND" => nw.supply.push((n, false)),
                "POWER" => nw.supply.push((n, true)),
                _ => {}
            }
            // 🔑 `dbNetwork::isPGSupply`: a terminal is no network pin at all when it is a supply
            // terminal, or sits on a SPECIAL supply net — so a power grid's pins are never seeded,
            // and only a signal pin tied to an ordinary supply net becomes a constant.
            let pg_net = db.net_is_special(&net) && is_supply(&sig);
            let mut pins = Vec::new();
            for b in db.net_bterms(&net) {
                if pg_net || is_supply(&db.bterm_sig(&b)) {
                    continue;
                }
                let p = match nw.port_index.get(&b) {
                    Some(&k) => nw.pin_inst.len() as u32 + k,
                    None => {
                        let k = nw.ports.len() as u32;
                        nw.port_index.insert(b.clone(), k);
                        nw.ports.push(b.clone());
                        nw.dir.push(Dir::default());
                        nw.net_of.push(NO_NET);
                        nw.pin_inst.len() as u32 + k
                    }
                };
                nw.dir[p as usize] = Dir::of(&db.bterm_io(&b));
                pins.push(p);
            }
            for it in db.net_iterms(&net) {
                // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself.
                let Some((inst, mterm)) = it.rsplit_once('/') else { continue };
                let Some(&i) = nw.inst_index.get(inst) else { return Err(format!("sim: {it}: no such instance").into()) };
                let m = nw.inst_master[i as usize] as usize;
                let Some(k) = nw.master_terms[m].iter().position(|t| t == mterm) else {
                    return Err(format!("sim: {it}: not a terminal of {}", nw.masters[m]).into());
                };
                if pg_net || term_supply[m][k] {
                    continue;
                }
                let p = nw.term_base[i as usize] + k as u32;
                nw.dir[p as usize] = term_dir[m][k];
                pins.push(p);
            }
            for &p in &pins {
                nw.net_of[p as usize] = n;
            }
            nw.pins_of.push(pins);
        }
        Ok(nw)
    }

    /// The index of a pin, if the network has it: a port some net names, or a terminal of a
    /// leaf instance's master.
    fn index(&self, pin: &Pin) -> Option<u32> {
        match pin {
            Pin::Port(p) => self.port_index.get(p).map(|k| self.pin_inst.len() as u32 + k),
            Pin::ITerm(inst, term) => {
                let i = *self.inst_index.get(inst)? as usize;
                let k = self.master_terms[self.inst_master[i] as usize].iter().position(|t| t == term)?;
                Some(self.term_base[i] + k as u32)
            }
        }
    }

    /// The pin at an index, by name.
    fn pin(&self, p: u32) -> Pin {
        let p = p as usize;
        match self.pin_inst.get(p) {
            Some(&i) => {
                let i = i as usize;
                let k = p - self.term_base[i] as usize;
                Pin::ITerm(self.insts[i].clone(), self.master_terms[self.inst_master[i] as usize][k].clone())
            }
            None => Pin::Port(self.ports[p - self.pin_inst.len()].clone()),
        }
    }

    /// `Network::isDriver`: an instance output, or a top-level input port.
    fn is_driver(&self, p: u32) -> bool {
        let d = self.dir[p as usize];
        if (p as usize) < self.pin_inst.len() { d.output } else { d.input }
    }

    /// `Network::isLoad`: an instance input, or a top-level output port.
    fn is_load(&self, p: u32) -> bool {
        let d = self.dir[p as usize];
        if (p as usize) < self.pin_inst.len() { d.input } else { d.output }
    }
}

struct Sim<'a> {
    nw: &'a Network,
    /// Each master's library cell (`None`: the library has no such cell).
    cells: Vec<Option<&'a crate::liberty::CellClock>>,
    values: Vec<Option<bool>>,
    queue: VecDeque<u32>,
    /// The constraint value of each pin `set_case_analysis` (first) or `set_logic_*` names:
    /// `None` for a transition or don't-care, which holds no constant but still blocks one.
    constraints: Vec<Option<Option<bool>>>,
    /// A constraint pin the network does not have holds its value and reaches nothing. Only the
    /// constraints set one, so the last constraint naming it decides (case after logic).
    off_values: BTreeMap<Pin, bool>,
    /// `propagateConstants(thru_sequentials)`: whether a sequential output takes its value from
    /// the sequential's data input (`findLogicConstants` does; `ensureConstantsPropagated` not).
    thru_sequentials: bool,
}

impl Sim<'_> {
    /// `Sim::setPinValue`: a pin with a constraint value takes no other value (the reference
    /// warns and keeps the constraint's).
    fn set_pin_value(&mut self, pin: u32, value: Option<bool>) {
        let p = pin as usize;
        if self.constraints[p].is_some_and(|c| c != value) {
            return;
        }
        if self.values[p] == value {
            return;
        }
        self.values[p] = value;
        let nw = self.nw;
        let d = nw.dir[p];
        match nw.pin_inst.get(p) {
            // A leaf instance's input: re-evaluate the instance (once in a row).
            Some(&inst) if d.input => {
                if self.queue.back() != Some(&inst) {
                    self.queue.push_back(inst);
                }
            }
            _ if nw.is_driver(pin) => {
                let net = nw.net_of[p];
                if net == NO_NET {
                    return;
                }
                for &l in &nw.pins_of[net as usize] {
                    if l != pin && nw.is_load(l) {
                        self.set_pin_value(l, value);
                    }
                }
            }
            _ => {}
        }
    }

    /// [`Sim::set_pin_value`] by name: a pin the network does not have only holds the value.
    fn set_named_pin_value(&mut self, pin: &Pin, value: Option<bool>) {
        match self.nw.index(pin) {
            Some(p) => self.set_pin_value(p, value),
            None => {
                match value {
                    Some(v) => self.off_values.insert(pin.clone(), v),
                    None => self.off_values.remove(pin),
                };
            }
        }
    }

    /// `setConstraintConstPins`: the `set_logic_*` values, then the `set_case_analysis` values.
    fn set_constraint_const_pins(&mut self, c: &Constraints) {
        for map in [&c.logic, &c.case] {
            for (p, v) in map {
                self.set_named_pin_value(p, *v);
            }
        }
    }

    /// `Sim::evalInstance(inst, thru_sequentials)`. Through sequentials, an output whose function
    /// is a lone state variable (`expr->port()`) a sequential drives is that sequential's DATA
    /// expression, inverted for its inverted output.
    fn eval_instance(&mut self, inst: u32) {
        let nw = self.nw;
        let i = inst as usize;
        let m = nw.inst_master[i] as usize;
        let Some(cell) = self.cells[m] else { return };
        let terms = &nw.master_terms[m];
        let base = nw.term_base[i];
        for (k, term) in terms.iter().enumerate() {
            if !cell.directions.get(term).is_some_and(|d| matches!(d.as_str(), "output" | "inout")) {
                continue;
            }
            let value = {
                let (values, off) = (&self.values, &self.off_values);
                let known = |port: &str| match terms.iter().position(|t| t == port) {
                    Some(k) => values[(base + k as u32) as usize],
                    None => off.get(&Pin::ITerm(nw.insts[i].clone(), port.to_string())).copied(),
                };
                if let Some(f) = cell.functions.get(term) {
                    match cell.tristate.get(term) {
                        Some(en) => {
                            if eval_constant(en, &known) == Some(true) {
                                eval_constant(f, &known)
                            } else {
                                None
                            }
                        }
                        None => function_value(cell, f, self.thru_sequentials, &known),
                    }
                } else if cell.clock_gate_out.contains(term) {
                    // `clockGateOutValue`: 0 when the gated clock or the enable is 0.
                    if cell.clock_gate_in.iter().any(|p| known(p) == Some(false)) { Some(false) } else { None }
                } else {
                    None
                }
            };
            let pin = base + k as u32;
            if value != self.values[pin as usize] {
                self.set_pin_value(pin, value);
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

/// [`constant_pins`] and [`constant_pins_constrained`] for one design state, the design read once
/// for both: `(without, with)` the constraints.
pub fn constant_pins_without_and_with(db: &Db, lib: &LibertyClocks, c: &Constraints) -> Res<(BTreeMap<Pin, bool>, BTreeMap<Pin, bool>)> {
    let nw = Network::read(db)?;
    Ok((run(&nw, lib, false, &Constraints::default()), run(&nw, lib, false, c)))
}

/// `Sta::findLogicConstants` (`Sim::findLogicConstants`): the same seeds — the constant-function
/// pins, then the pins on constant nets; no SDC constant, which that call never reads — propagated
/// THROUGH sequentials. A pin's value only goes from unknown to a constant, so seeding order does
/// not change the fixed point.
pub fn logic_constants(db: &Db, lib: &LibertyClocks) -> Res<BTreeMap<Pin, bool>> {
    propagate(db, lib, true, &Constraints::default())
}

fn propagate(db: &Db, lib: &LibertyClocks, thru_sequentials: bool, c: &Constraints) -> Res<BTreeMap<Pin, bool>> {
    Ok(run(&Network::read(db)?, lib, thru_sequentials, c))
}


/// One simulation over a read [`Network`]: the seeds (supplies, then the constraints, then the
/// constant-function outputs), then `propagateConstants(thru_sequentials)`.
fn run(nw: &Network, lib: &LibertyClocks, thru_sequentials: bool, c: &Constraints) -> BTreeMap<Pin, bool> {
    let pins = nw.dir.len();
    let mut sim = Sim {
        nw,
        cells: nw.masters.iter().map(|m| lib.cells.get(m)).collect(),
        values: vec![None; pins],
        queue: VecDeque::new(),
        constraints: vec![None; pins],
        off_values: BTreeMap::new(),
        thru_sequentials,
    };
    // A pin in both takes its case value (`setPinValue` asks the case analysis first).
    for (p, v) in c.logic.iter().chain(&c.case) {
        if let Some(i) = nw.index(p) {
            sim.constraints[i as usize] = Some(*v);
        }
    }
    // seedConstants: the network's constant pins, ground nets first, then power nets.
    for value in [false, true] {
        for &(net, v) in nw.supply.iter().filter(|(_, v)| *v == value) {
            for &p in &nw.pins_of[net as usize] {
                sim.set_pin_value(p, Some(v));
            }
        }
    }
    sim.set_constraint_const_pins(c);
    // setConstFuncPins: every instance output whose function is a constant (a tie cell).
    for i in 0..nw.insts.len() {
        let m = nw.inst_master[i] as usize;
        let Some(cell) = sim.cells[m] else { continue };
        for (k, term) in nw.master_terms[m].iter().enumerate() {
            if cell.constant_outputs.contains(term) {
                let one = cell.functions.get(term).is_some_and(|f| eval_constant(f, &|_| None) == Some(true));
                sim.set_pin_value(nw.term_base[i] + k as u32, Some(one));
            }
        }
    }
    // propagateConstants(false).
    while let Some(inst) = sim.queue.pop_front() {
        sim.eval_instance(inst);
    }
    let mut out = sim.off_values;
    for (p, v) in sim.values.iter().enumerate() {
        if let Some(v) = v {
            out.insert(nw.pin(p as u32), *v);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_design::{random_design, Design, NetDef, LIB};

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

    /// The simulation keyed by NAME, as it ran before pins were indexed: the test oracle for the
    /// indexed one. An instance pin's signal type is read per pin here (`dbITerm::getSigType`),
    /// per master in the indexed read.
    mod by_name {
        #![allow(dead_code)]
        use super::super::{eval_constant, function_value, Constraints, LibertyClocks, Pin, Res, Source};
        use std::collections::{BTreeMap, VecDeque};

        fn iterm_sig(db: &impl Source, inst: &str, mterm: &str) -> String {
            db.master_terms(&db.inst_master(inst)).into_iter().find(|(t, _)| t == mterm).map(|(_, s)| s).unwrap_or_default()
        }

        /// The design as the simulation reads it — read once, and read alike by every simulation of the
        /// same design state (only the seeds differ between them).
        #[derive(Debug, Default)]
        pub struct Network {
            /// Each pin's net, and each net's pins.
            net_of: BTreeMap<Pin, String>,
            pins_of: BTreeMap<String, Vec<Pin>>,
            /// The database direction of each pin (`network_->direction`): a port's, or its master
            /// terminal's.
            dir: BTreeMap<Pin, String>,
            /// Each instance's master, and its terminals in the master's order.
            master: BTreeMap<String, String>,
            terms: BTreeMap<String, Vec<String>>,
            /// The supply nets (`findConstantNets`) and their values, in net order.
            supply: Vec<(String, bool)>,
        }

        struct Sim<'a> {
            lib: &'a LibertyClocks,
            nw: &'a Network,
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
                let d = self.nw.dir.get(p).map(String::as_str).unwrap_or("");
                match p {
                    Pin::ITerm(..) => any_output(d),
                    Pin::Port(_) => any_input(d),
                }
            }

            /// `Network::isLoad`: an instance input, or a top-level output port.
            fn is_load(&self, p: &Pin) -> bool {
                let d = self.nw.dir.get(p).map(String::as_str).unwrap_or("");
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
                let d = self.nw.dir.get(pin).cloned().unwrap_or_default();
                match pin {
                    // A leaf instance's input: re-evaluate the instance (once in a row).
                    Pin::ITerm(inst, _) if any_input(&d) => {
                        if self.queue.back() != Some(inst) {
                            self.queue.push_back(inst.clone());
                        }
                    }
                    _ if self.is_driver(pin) => {
                        let Some(net) = self.nw.net_of.get(pin).cloned() else { return };
                        let loads: Vec<Pin> = self.nw.pins_of.get(&net).into_iter().flatten().filter(|p| *p != pin && self.is_load(p)).cloned().collect();
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
                let master = self.nw.master.get(inst).cloned().unwrap_or_default();
                let Some(cell) = self.lib.cells.get(&master) else { return };
                let terms = self.nw.terms.get(inst).cloned().unwrap_or_default();
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

        impl Network {
            /// The design read once: every net's network pins, each pin's direction, every leaf
            /// instance's master and terminals, and the supply nets.
            pub fn read(db: &impl Source) -> Res<Network> {
                let mut sim = Network::default();
                let is_supply = |sig: &str| matches!(sig, "POWER" | "GROUND");
                for net in db.net_names() {
                    let sig = db.net_sigtype(&net);
                    match sig.as_str() {
                        // `dbNetwork::findConstantNets`: a ground net is a constant 0, a power net a 1.
                        "GROUND" => sim.supply.push((net.clone(), false)),
                        "POWER" => sim.supply.push((net.clone(), true)),
                        _ => {}
                    }
                    // 🔑 `dbNetwork::isPGSupply`: a terminal is no network pin at all when it is a supply
                    // terminal, or sits on a SPECIAL supply net — so a power grid's pins are never seeded,
                    // and only a signal pin tied to an ordinary supply net becomes a constant.
                    let pg_net = db.net_is_special(&net) && is_supply(&sig);
                    let mut pins = Vec::new();
                    for b in db.net_bterms(&net) {
                        if pg_net || is_supply(&db.bterm_sig(&b)) {
                            continue;
                        }
                        let p = Pin::Port(b.clone());
                        sim.dir.insert(p.clone(), db.bterm_io(&b));
                        pins.push(p);
                    }
                    for it in db.net_iterms(&net) {
                        // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself.
                        let Some((inst, mterm)) = it.rsplit_once('/') else { continue };
                        if pg_net || is_supply(&iterm_sig(db, inst, mterm)) {
                            continue;
                        }
                        let master = sim.master.entry(inst.to_string()).or_insert_with(|| db.inst_master(inst)).clone();
                        let p = Pin::ITerm(inst.to_string(), mterm.to_string());
                        sim.dir.insert(p.clone(), db.mterm_io(&master, mterm));
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
                        .or_insert_with(|| db.master_terms(master).into_iter().map(|(n, _)| n).collect())
                        .clone();
                    sim.terms.insert(inst.clone(), terms);
                }
                Ok(sim)
            }
        }

        /// One simulation over a read [`Network`]: the seeds (supplies, then the constraints, then the
        /// constant-function outputs), then `propagateConstants(thru_sequentials)`.
        pub fn run(nw: &Network, lib: &LibertyClocks, thru_sequentials: bool, c: &Constraints) -> BTreeMap<Pin, bool> {
            let mut constraints = c.logic.clone();
            constraints.extend(c.case.iter().map(|(p, v)| (p.clone(), *v)));
            let mut sim = Sim { lib, nw, values: BTreeMap::new(), queue: VecDeque::new(), constraints, thru_sequentials };
            // seedConstants: the network's constant pins, ground nets first, then power nets.
            for value in [false, true] {
                for (net, v) in nw.supply.iter().filter(|(_, v)| *v == value) {
                    for p in nw.pins_of.get(net).cloned().unwrap_or_default() {
                        sim.set_pin_value(&p, Some(*v));
                    }
                }
            }
            sim.set_constraint_const_pins(c);
            // setConstFuncPins: every instance output whose function is a constant (a tie cell).
            let insts: Vec<(String, String)> = nw.master.iter().map(|(i, m)| (i.clone(), m.clone())).collect();
            for (inst, master) in insts {
                let Some(cell) = lib.cells.get(&master) else { continue };
                for term in nw.terms.get(&inst).cloned().unwrap_or_default() {
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
            sim.values
        }
    }

    /// `kld` -> u1 (buffer) -> u2 (inverter), as the Sim reads a netlist.
    fn chain_design() -> Design {
        let sig = |t: &str, io: &str| (t.to_string(), "SIGNAL".to_string(), io.to_string());
        let net = |name: &str, bterms: Vec<(String, String, String)>, iterms: &[&str]| NetDef {
            name: name.into(),
            sig: "SIGNAL".into(),
            special: false,
            bterms,
            iterms: iterms.iter().map(|s| s.to_string()).collect(),
        };
        Design {
            insts: vec![("u1".into(), "buf".into()), ("u2".into(), "inv".into())],
            masters: [("buf".into(), vec![sig("A", "INPUT"), sig("Z", "OUTPUT")]), ("inv".into(), vec![sig("A", "INPUT"), sig("ZN", "OUTPUT")])].into(),
            nets: vec![net("kld", vec![sig("kld", "INPUT")], &["u1/A"]), net("n1", vec![], &["u1/Z", "u2/A"]), net("n2", vec![], &["u2/ZN"])],
        }
    }

    fn chain() -> Network {
        Network::read_from(&chain_design()).unwrap()
    }

    /// Rule (`Sim::setPinValue` / `evalInstance` / `seedConstants` read pins through the network,
    /// whatever names them; `dbITerm::getSigType` / `getIoType` are the MTERM's): the simulation
    /// keyed by pin index gives, pin for pin, what the simulation keyed by name gives — over
    /// generated designs, with and without constraints, both `propagateConstants` modes.
    #[test]
    fn the_indexed_simulation_gives_what_the_named_one_gives() {
        let mut lib = LibertyClocks::default();
        lib.read(LIB).unwrap();
        let (mut nonempty, mut off) = (0, 0);
        for seed in 0..400 {
            let (d, c) = random_design(seed);
            let nw = Network::read_from(&d).unwrap();
            let named = by_name::Network::read(&d).unwrap();
            for thru in [false, true] {
                for c in [&Constraints::default(), &c] {
                    let ours = run(&nw, &lib, thru, c);
                    let want = by_name::run(&named, &lib, thru, c);
                    assert_eq!(ours, want, "seed {seed} thru {thru} constraints {c:?}");
                    nonempty += usize::from(!ours.is_empty());
                    off += usize::from(ours.keys().any(|p| nw.index(p).is_none()));
                }
            }
        }
        assert!(nonempty > 1000 && off > 10, "the generated designs exercise the simulation: {nonempty} non-empty, {off} with an off-network pin");
    }

    fn buf_inv() -> LibertyClocks {
        let mut lib = LibertyClocks::default();
        lib.read(r#"library (t) {
            cell (buf) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A"; } }
            cell (inv) { pin (A) { direction : input; } pin (ZN) { direction : output; function : "!A"; } } }"#)
            .unwrap();
        lib
    }

    /// Rule (`Sim::ensureConstantsPropagated`: each propagation clears its values and seeds them
    /// anew from the network and the constraints in force, so the network read is the same for
    /// every propagation of one design state): a simulation over an already-simulated network
    /// is the simulation over a fresh read, whatever the earlier one's constraints.
    #[test]
    fn a_second_simulation_on_one_network_reads_as_a_fresh_one() {
        let lib = buf_inv();
        let nw = chain();
        let case = Constraints { logic: BTreeMap::new(), case: [(Pin::Port("kld".into()), Some(false))].into() };
        let without = run(&nw, &lib, false, &Constraints::default());
        let with = run(&nw, &lib, false, &case);
        assert!(without.is_empty());
        assert_eq!(with, run(&chain(), &lib, false, &case));
        assert_eq!(with.get(&Pin::ITerm("u2".into(), "ZN".into())), Some(&true));
        assert_eq!(run(&nw, &lib, false, &Constraints::default()), without);
    }

    /// Rule (`Sim::seedConstants` / `setPinValue`): a `set_case_analysis` port value propagates
    /// like a tie; a pin holding a constraint value takes no other (a don't-care blocks a constant).
    #[test]
    fn a_case_value_propagates_and_a_constraint_pin_keeps_its_own() {
        let lib = buf_inv();
        let nw = chain();
        let kld = Pin::Port("kld".into());
        let simulate = |constraints: BTreeMap<Pin, Option<bool>>| run(&nw, &lib, false, &Constraints { logic: BTreeMap::new(), case: constraints });
        let free = simulate([(kld.clone(), Some(false))].into());
        assert_eq!(free.get(&Pin::ITerm("u2".into(), "ZN".into())), Some(&true));
        let held = simulate([(kld.clone(), Some(false)), (Pin::ITerm("u2".into(), "A".into()), None)].into());
        assert_eq!(held.get(&Pin::ITerm("u1".into(), "Z".into())), Some(&false));
        assert_eq!(held.get(&Pin::ITerm("u2".into(), "A".into())), None);
        assert_eq!(held.get(&Pin::ITerm("u2".into(), "ZN".into())), None);
    }
}
