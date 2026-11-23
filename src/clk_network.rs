// SPDX-License-Identifier: Apache-2.0
//! `dbSta::findClkNets` — the nets the clock network reaches, which `initClockNets` re-types to
//! CLOCK before net discovery.
//!
//! The timer's clock network (`ClkNetwork::findClkPins`) is a breadth-first search over its timing
//! graph from each clock's source pins. A pin is visited once; from it the search follows
//! (`Graph::visitFanouts` with `ClkSearchPred`):
//!
//! - a WIRE edge — a net's driver to each of its loads (a top-level input port drives, a top-level
//!   output port loads, an instance pin by its master terminal's direction);
//! - an INSTANCE edge — one per liberty arc of the instance's cell, but only a COMBINATIONAL one
//!   (`searchThruAllow`): clock-to-Q, latch D-to-Q, set/clear and the tristate roles all stop it;
//!
//! and never INTO another clock's source pin (`searchTo`: `!isLeafPinClock`). The clock nets are
//! the flat nets of every visited pin.
//!
//! A bidirect TOP-LEVEL port has two vertices (`makePinVertices`: load, then driver): it is a load
//! of its net, and its driver vertex drives the net's other loads. The edge between them
//! (`isBidirectPortPath`) is never walked (`ClkTreeSearchPred::searchThru`), and nothing else
//! reaches a port's driver vertex, so the port drives only when it is itself a clock source
//! (`findClkPins` enqueues both vertices of a leaf pin).
//!
//! ⛔ Refused, not modelled: a bidirect INSTANCE pin (`isBidirectInstPath` edges), a
//! cycle among the traversed edges (the levelizer disables a loop edge — `isDisabledLoop`), a net
//! with ten or more drivers (`isIsolatedNet` may drop its edges), and a cell whose roles
//! `inferLatchRoles` may rewrite. No SDC is read, so nothing is disabled by constraint.

#![cfg(feature = "odb")]

use std::collections::{BTreeSet, HashMap, VecDeque};
use vyges_opendb::Db;

use crate::liberty::{LibertyClocks, Role};
use crate::sim::Source;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// A timing-graph pin: a top-level port, or an instance terminal.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Pin {
    Port(String),
    ITerm(String, String),
}

/// `findClkNets` for clocks defined on the top-level ports `sources`, in the database's net names.
pub fn find_clk_nets(db: &Db, lib: &LibertyClocks, sources: &[String]) -> Res<BTreeSet<String>> {
    find_clk_pins(db, lib, sources, sources, &BTreeSet::new())
}

/// The nets of the IDEAL clock pins (`ClkNetwork::findClkPins(ideal_only)`), in net names.
///
/// 🔑 Reference rule (OpenSTA at the 4155ac3 pin): the ideal walk starts only from the clocks that
/// are NOT propagated, and from each such clock's source pins that are not themselves propagated;
/// a propagated pin it reaches is neither marked nor walked through. Every clock's source pins,
/// propagated or not, still stop the walk (`isLeafPinClock`). `all_sources` are every clock's
/// sources, `ideal_sources` those of the clocks not propagated, `propagated_pins` the pins
/// `set_propagated_clock` named (a port's name, or `inst/term`).
pub fn find_ideal_clk_nets(db: &Db, lib: &LibertyClocks, all_sources: &[String], ideal_sources: &[String], propagated_pins: &BTreeSet<String>) -> Res<BTreeSet<String>> {
    find_clk_pins(db, lib, all_sources, ideal_sources, propagated_pins)
}

const NONE: u32 = u32::MAX;

/// The netlist as the timing graph sees it, pins by index: each instance met on a net owns a block
/// of its master's terminals (in the master's order), and a top-level port is one pin (a bidirect
/// port's load and driver vertices share it, as they share a name). A terminal's direction is its
/// MTERM's, read once per master.
#[derive(Default)]
struct ClkGraph<'a> {
    insts: Vec<String>,
    inst_index: HashMap<String, u32>,
    inst_master: Vec<u32>,
    term_base: Vec<u32>,
    masters: Vec<String>,
    master_index: HashMap<String, u32>,
    master_terms: Vec<Vec<String>>,
    master_dirs: Vec<Vec<String>>,
    cells: Vec<Option<&'a crate::liberty::CellClock>>,
    /// Each pin's instance (`NONE`: a port) and terminal position (a port: its index in `ports`).
    pin_inst: Vec<u32>,
    pin_k: Vec<u32>,
    ports: Vec<String>,
    port_index: HashMap<String, u32>,
    /// Each pin's net (`NONE`: no net), each net's name, drivers and loads.
    net_of: Vec<u32>,
    nets: Vec<String>,
    drivers: Vec<Vec<u32>>,
    loads: Vec<Vec<u32>>,
    bidirect: Vec<bool>,
}

impl<'a> ClkGraph<'a> {
    fn new_pin(&mut self, inst: u32, k: u32) -> u32 {
        let p = self.pin_inst.len() as u32;
        self.pin_inst.push(inst);
        self.pin_k.push(k);
        self.net_of.push(NONE);
        self.bidirect.push(false);
        p
    }

    fn port(&mut self, b: &str) -> u32 {
        if let Some(&p) = self.port_index.get(b) {
            return p;
        }
        let k = self.ports.len() as u32;
        self.ports.push(b.to_string());
        let p = self.new_pin(NONE, k);
        self.port_index.insert(b.to_string(), p);
        p
    }

    /// The instance, its master read the first time the instance is met.
    fn inst(&mut self, db: &impl Source, lib: &'a LibertyClocks, inst: &str) -> u32 {
        if let Some(&i) = self.inst_index.get(inst) {
            return i;
        }
        let master = db.inst_master(inst);
        let m = match self.master_index.get(&master) {
            Some(&m) => m,
            None => {
                let terms: Vec<String> = db.master_terms(&master).into_iter().map(|(n, _)| n).collect();
                self.master_dirs.push(terms.iter().map(|t| db.mterm_io(&master, t)).collect());
                self.master_terms.push(terms);
                self.cells.push(lib.cells.get(&master));
                let m = self.masters.len() as u32;
                self.masters.push(master.clone());
                self.master_index.insert(master, m);
                m
            }
        };
        let i = self.insts.len() as u32;
        self.insts.push(inst.to_string());
        self.inst_index.insert(inst.to_string(), i);
        self.inst_master.push(m);
        self.term_base.push(self.pin_inst.len() as u32);
        for k in 0..self.master_terms[m as usize].len() {
            self.new_pin(i, k as u32);
        }
        i
    }

    /// The pin by name, for messages and for the visited order.
    fn pin(&self, p: u32) -> Pin {
        let (i, k) = (self.pin_inst[p as usize], self.pin_k[p as usize] as usize);
        if i == NONE {
            Pin::Port(self.ports[k].clone())
        } else {
            Pin::ITerm(self.insts[i as usize].clone(), self.master_terms[self.inst_master[i as usize] as usize][k].clone())
        }
    }

    /// The index of a named pin (a port's name, or `inst/term`), if the graph has it.
    fn named(&self, name: &str) -> Vec<u32> {
        let mut out: Vec<u32> = self.port_index.get(name).copied().into_iter().collect();
        if let Some((inst, term)) = name.rsplit_once('/') {
            if let Some(&i) = self.inst_index.get(inst) {
                if let Some(k) = self.master_terms[self.inst_master[i as usize] as usize].iter().position(|t| t == term) {
                    out.push(self.term_base[i as usize] + k as u32);
                }
            }
        }
        out
    }
}

fn find_clk_pins(db: &impl Source, lib: &LibertyClocks, sources: &[String], start: &[String], propagated: &BTreeSet<String>) -> Res<BTreeSet<String>> {
    // The netlist as the graph sees it: each pin's net, each net's drivers and loads.
    let mut g = ClkGraph::default();
    for net in db.net_names() {
        let sig = db.net_sigtype(&net);
        if sig == "POWER" || sig == "GROUND" {
            continue; // power and ground pins have no vertices
        }
        let mut pins = Vec::new();
        for b in db.net_bterms(&net) {
            let dir = db.bterm_io(&b);
            let p = g.port(&b);
            match dir.as_str() {
                "INPUT" => pins.push((p, true)),
                "OUTPUT" => pins.push((p, false)),
                // Both vertices: the driver counts towards the net's drivers, as it does upstream.
                "INOUT" => {
                    g.bidirect[p as usize] = true;
                    pins.push((p, true));
                    pins.push((p, false));
                }
                d => return Err(format!("clock network: port on net {net} has direction {d} — not modelled").into()),
            }
        }
        for it in db.net_iterms(&net) {
            // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself
            // (`u_mid1/dff_load1/CK`); the terminal is only what follows the final one.
            let Some((inst, mterm)) = it.rsplit_once('/') else { continue };
            let i = g.inst(db, lib, inst) as usize;
            let m = g.inst_master[i] as usize;
            let k = g.master_terms[m].iter().position(|t| t == mterm);
            let dir = k.map(|k| g.master_dirs[m][k].as_str()).unwrap_or("");
            let is_driver = match dir {
                "OUTPUT" => true,
                "INPUT" => false,
                d => return Err(format!("clock network: {inst}/{mterm} has direction {d} — not modelled").into()),
            };
            pins.push((g.term_base[i] + k.unwrap_or(0) as u32, is_driver));
        }
        let n_drivers = pins.iter().filter(|(_, d)| *d).count();
        if n_drivers >= 10 {
            return Err(format!("clock network: net {net} has {n_drivers} drivers — isIsolatedNet is not modelled").into());
        }
        let n = g.nets.len() as u32;
        let (mut ds, mut ls) = (Vec::new(), Vec::new());
        for (p, is_driver) in pins {
            g.net_of[p as usize] = n;
            if is_driver { ds.push(p) } else { ls.push(p) }
        }
        g.nets.push(net);
        g.drivers.push(ds);
        g.loads.push(ls);
    }
    let pins = g.pin_inst.len();
    let mut leaf = vec![false; pins];
    let mut sorted_sources: Vec<&String> = sources.iter().collect();
    sorted_sources.sort();
    sorted_sources.dedup();
    for s in sorted_sources {
        match g.port_index.get(s.as_str()) {
            Some(&p) if g.net_of[p as usize] != NONE => leaf[p as usize] = true,
            _ => return Err(format!("clock source {:?}: no such connected port", Pin::Port(s.clone())).into()),
        }
    }
    // A bidirect port's driver vertex is reached only as a clock source: drop it from the others.
    for ds in &mut g.drivers {
        ds.retain(|&p| !g.bidirect[p as usize] || leaf[p as usize]);
    }
    let mut stop = vec![false; pins];
    for name in propagated {
        for p in g.named(name) {
            stop[p as usize] = true;
        }
    }

    // The search, and the traversed instance edges (for the loop tripwire).
    let mut visited = vec![false; pins];
    let mut order: Vec<u32> = Vec::new();
    let mut queue: VecDeque<u32> = VecDeque::new();
    let mut inst_edges: Vec<(u32, u32)> = Vec::new();
    for s in start {
        let Some(&p) = g.port_index.get(s.as_str()) else { continue };
        if !leaf[p as usize] || stop[p as usize] {
            continue;
        }
        if !visited[p as usize] {
            visited[p as usize] = true;
            order.push(p);
            queue.push_back(p);
        }
    }
    let mut fanout: Vec<u32> = Vec::new();
    while let Some(pin) = queue.pop_front() {
        fanout.clear();
        // Wire edges: from a driver to every load of its net.
        let net = g.net_of[pin as usize];
        if net != NONE && g.drivers[net as usize].contains(&pin) {
            fanout.extend(g.loads[net as usize].iter().filter(|&&l| l != pin));
        }
        // Instance edges: the cell's combinational arcs from this terminal.
        let i = g.pin_inst[pin as usize];
        if i != NONE {
            let m = g.inst_master[i as usize] as usize;
            if let Some(cell) = g.cells[m] {
                let terms = &g.master_terms[m];
                let mterm = &terms[g.pin_k[pin as usize] as usize];
                for arc in cell.arcs.iter().filter(|a| &a.from == mterm && a.role == Role::Combinational) {
                    let Some(k) = terms.iter().position(|t| *t == arc.to) else {
                        continue; // findPin finds no pin: no edge
                    };
                    if cell.latch_roles_may_be_inferred {
                        return Err(format!("clock network through {} ({}): inferLatchRoles may re-type its arcs — not modelled", g.insts[i as usize], g.masters[m]).into());
                    }
                    let to = g.term_base[i as usize] + k as u32;
                    inst_edges.push((pin, to));
                    fanout.push(to);
                }
            }
        }
        for &to in &fanout {
            if leaf[to as usize] {
                continue; // searchTo: never into a clock's source pin
            }
            if stop[to as usize] {
                continue; // a propagated pin is not an ideal clock pin, nor walked through
            }
            if !visited[to as usize] {
                visited[to as usize] = true;
                order.push(to);
                queue.push_back(to);
            }
        }
    }
    refuse_loops(&g, &order, &visited, &inst_edges)?;
    Ok(order.iter().filter(|&&p| g.net_of[p as usize] != NONE).map(|&p| g.nets[g.net_of[p as usize] as usize].clone()).collect())
}

/// ⛔ A cycle among the traversed edges would have a loop edge the levelizer disables; refused.
/// The search visits the pins in name order, each pin's instance edges in traversal order and
/// then its net's loads, so the loop reported is the one the name-keyed search reports.
fn refuse_loops(g: &ClkGraph<'_>, order: &[u32], visited: &[bool], inst_edges: &[(u32, u32)]) -> Res<()> {
    let mut by_name: Vec<(Pin, u32)> = order.iter().map(|&p| (g.pin(p), p)).collect();
    by_name.sort();
    let mut succ: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in inst_edges {
        succ.entry(a).or_default().push(b);
    }
    for &(_, p) in &by_name {
        let net = g.net_of[p as usize];
        if net != NONE && g.drivers[net as usize].contains(&p) {
            // a bidirect port is a load of its own net: no wire edge to itself
            succ.entry(p).or_default().extend(g.loads[net as usize].iter().filter(|&&l| l != p && visited[l as usize]));
        }
    }
    // Iterative three-colour DFS.
    let mut colour: HashMap<u32, u8> = HashMap::new();
    for &(_, start) in &by_name {
        if colour.contains_key(&start) {
            continue;
        }
        let mut stack: Vec<(u32, usize)> = vec![(start, 0)];
        colour.insert(start, 1);
        while let Some((p, i)) = stack.pop() {
            let next = succ.get(&p).and_then(|s| s.get(i)).copied();
            match next {
                Some(q) => {
                    stack.push((p, i + 1));
                    match colour.get(&q) {
                        Some(1) => return Err(format!("clock network: a combinational loop through {:?} — isDisabledLoop is not modelled", g.pin(q)).into()),
                        Some(_) => {}
                        None => {
                            colour.insert(q, 1);
                            stack.push((q, 0));
                        }
                    }
                }
                None => {
                    colour.insert(p, 2);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The search keyed by NAME, as it ran before pins were indexed: the test oracle.
    mod by_name {
        use super::super::{LibertyClocks, Pin, Res, Role, Source};

        fn pin_name(p: &Pin) -> String {
            match p {
                Pin::Port(n) => n.clone(),
                Pin::ITerm(i, t) => format!("{i}/{t}"),
            }
        }
        use std::collections::{BTreeMap, BTreeSet, VecDeque};

        pub fn find_clk_pins(db: &impl Source, lib: &LibertyClocks, sources: &[String], start: &[String], propagated: &BTreeSet<String>) -> Res<BTreeSet<String>> {
            // The netlist as the graph sees it: each pin's net, each net's drivers and loads.
            let mut net_of: BTreeMap<Pin, String> = BTreeMap::new();
            let mut drivers: BTreeMap<String, Vec<Pin>> = BTreeMap::new();
            let mut loads: BTreeMap<String, Vec<Pin>> = BTreeMap::new();
            let mut masters: BTreeMap<String, String> = BTreeMap::new();
            // Bidirect top-level ports: a load always, a driver only as a clock source (see the head).
            let mut bidirect: BTreeSet<Pin> = BTreeSet::new();
            for net in db.net_names() {
                let sig = db.net_sigtype(&net);
                if sig == "POWER" || sig == "GROUND" {
                    continue; // power and ground pins have no vertices
                }
                let mut pins = Vec::new();
                for b in db.net_bterms(&net) {
                    let dir = db.bterm_io(&b);
                    match dir.as_str() {
                        "INPUT" => pins.push((Pin::Port(b), true)),
                        "OUTPUT" => pins.push((Pin::Port(b), false)),
                        // Both vertices: the driver counts towards the net's drivers, as it does upstream.
                        "INOUT" => {
                            bidirect.insert(Pin::Port(b.clone()));
                            pins.push((Pin::Port(b.clone()), true));
                            pins.push((Pin::Port(b), false));
                        }
                        d => return Err(format!("clock network: port on net {net} has direction {d} — not modelled").into()),
                    }
                }
                for it in db.net_iterms(&net) {
                    // ⛔ The LAST '/': a flattened hierarchical instance name contains '/' itself
                    // (`u_mid1/dff_load1/CK`); the terminal is only what follows the final one.
                    let Some((inst, mterm)) = it.rsplit_once('/') else { continue };
                    let master = masters.entry(inst.to_string()).or_insert_with(|| db.inst_master(inst)).clone();
                    let dir = db.mterm_io(&master, mterm);
                    pins.push((Pin::ITerm(inst.to_string(), mterm.to_string()), match dir.as_str() {
                        "OUTPUT" => true,
                        "INPUT" => false,
                        d => return Err(format!("clock network: {inst}/{mterm} has direction {d} — not modelled").into()),
                    }));
                }
                let n_drivers = pins.iter().filter(|(_, d)| *d).count();
                if n_drivers >= 10 {
                    return Err(format!("clock network: net {net} has {n_drivers} drivers — isIsolatedNet is not modelled").into());
                }
                for (p, is_driver) in pins {
                    net_of.insert(p.clone(), net.clone());
                    if is_driver { drivers.entry(net.clone()).or_default().push(p) } else { loads.entry(net.clone()).or_default().push(p) }
                }
            }
            let leaf: BTreeSet<Pin> = sources.iter().map(|s| Pin::Port(s.clone())).collect();
            // A bidirect port's driver vertex is reached only as a clock source: drop it from the others.
            for ds in drivers.values_mut() {
                ds.retain(|p| !bidirect.contains(p) || leaf.contains(p));
            }
            for s in &leaf {
                if !net_of.contains_key(s) {
                    return Err(format!("clock source {s:?}: no such connected port").into());
                }
            }
            let mut mterms_of: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();

            // The search, and the traversed instance edges (for the loop tripwire).
            let mut visited: BTreeSet<Pin> = BTreeSet::new();
            let mut queue: VecDeque<Pin> = VecDeque::new();
            let mut inst_edges: Vec<(Pin, Pin)> = Vec::new();
            for s in start.iter().map(|s| Pin::Port(s.clone())) {
                if !leaf.contains(&s) || propagated.contains(&pin_name(&s)) {
                    continue;
                }
                if visited.insert(s.clone()) {
                    queue.push_back(s);
                }
            }
            while let Some(pin) = queue.pop_front() {
                let mut fanout: Vec<Pin> = Vec::new();
                // Wire edges: from a driver to every load of its net.
                if let Some(net) = net_of.get(&pin) {
                    if drivers.get(net).is_some_and(|d| d.contains(&pin)) {
                        fanout.extend(loads.get(net).into_iter().flatten().filter(|l| **l != pin).cloned());
                    }
                }
                // Instance edges: the cell's combinational arcs from this terminal.
                if let Pin::ITerm(inst, mterm) = &pin {
                    let master = masters.entry(inst.clone()).or_insert_with(|| db.inst_master(inst)).clone();
                    if let Some(cell) = lib.cells.get(&master) {
                        let terms = mterms_of.entry(master.clone()).or_insert_with(|| db.master_terms(&master).into_iter().map(|(n, _)| n).collect());
                        for arc in cell.arcs.iter().filter(|a| &a.from == mterm && a.role == Role::Combinational) {
                            if !terms.contains(&arc.to) {
                                continue; // findPin finds no pin: no edge
                            }
                            if cell.latch_roles_may_be_inferred {
                                return Err(format!("clock network through {inst} ({master}): inferLatchRoles may re-type its arcs — not modelled").into());
                            }
                            let to = Pin::ITerm(inst.clone(), arc.to.clone());
                            inst_edges.push((pin.clone(), to.clone()));
                            fanout.push(to);
                        }
                    }
                }
                for to in fanout {
                    if leaf.contains(&to) {
                        continue; // searchTo: never into a clock's source pin
                    }
                    if propagated.contains(&pin_name(&to)) {
                        continue; // a propagated pin is not an ideal clock pin, nor walked through
                    }
                    if visited.insert(to.clone()) {
                        queue.push_back(to);
                    }
                }
            }
            refuse_loops(&visited, &inst_edges, &net_of, &drivers, &loads)?;
            Ok(visited.iter().filter_map(|p| net_of.get(p).cloned()).collect())
        }

        /// ⛔ A cycle among the traversed edges would have a loop edge the levelizer disables; refused.
        fn refuse_loops(
            visited: &BTreeSet<Pin>,
            inst_edges: &[(Pin, Pin)],
            net_of: &BTreeMap<Pin, String>,
            drivers: &BTreeMap<String, Vec<Pin>>,
            loads: &BTreeMap<String, Vec<Pin>>,
        ) -> Res<()> {
            let mut succ: BTreeMap<&Pin, Vec<&Pin>> = BTreeMap::new();
            for (a, b) in inst_edges {
                succ.entry(a).or_default().push(b);
            }
            for p in visited {
                if let Some(net) = net_of.get(p) {
                    if drivers.get(net).is_some_and(|d| d.contains(p)) {
                        // a bidirect port is a load of its own net: no wire edge to itself
                        succ.entry(p).or_default().extend(loads.get(net).into_iter().flatten().filter(|l| *l != p && visited.contains(*l)));
                    }
                }
            }
            // Iterative three-colour DFS.
            let mut colour: BTreeMap<&Pin, u8> = BTreeMap::new();
            for start in visited {
                if colour.contains_key(start) {
                    continue;
                }
                let mut stack: Vec<(&Pin, usize)> = vec![(start, 0)];
                colour.insert(start, 1);
                while let Some((p, i)) = stack.pop() {
                    let next = succ.get(p).and_then(|s| s.get(i)).copied();
                    match next {
                        Some(q) => {
                            stack.push((p, i + 1));
                            match colour.get(q) {
                                Some(1) => return Err(format!("clock network: a combinational loop through {q:?} — isDisabledLoop is not modelled").into()),
                                Some(_) => {}
                                None => {
                                    colour.insert(q, 1);
                                    stack.push((q, 0));
                                }
                            }
                        }
                        None => {
                            colour.insert(p, 2);
                        }
                    }
                }
            }
            Ok(())
        }
    }


    /// Rule (`ClkNetwork::findClkPins` walks the timing graph's vertices, whatever names them; a
    /// terminal's direction is its MTERM's, `dbITerm::getIoType`): the search keyed by pin index
    /// reaches, net for net, what the search keyed by name reaches — and refuses what it refuses,
    /// with the same message — over generated designs, sources, ideal starts and propagated pins.
    #[test]
    fn the_indexed_clock_search_reaches_what_the_named_one_reaches() {
        let mut lib = LibertyClocks::default();
        lib.read(crate::test_design::LIB).unwrap();
        let (mut reached, mut refused) = (0, 0);
        for seed in 0..600 {
            let (d, _) = crate::test_design::random_design_with(seed, seed % 3 == 0);
            let mut s = seed.wrapping_mul(0x2545_F491_4F6C_DD1D) | 1;
            let mut rnd = |n: usize| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((s >> 33) as usize) % n.max(1)
            };
            let ports: Vec<String> = d.nets.iter().filter(|n| n.sig == "SIGNAL").flat_map(|n| n.bterms.iter().map(|b| b.0.clone())).collect();
            let mut sources: Vec<String> = ports.iter().filter(|_| rnd(3) != 0).cloned().collect();
            if rnd(20) == 0 {
                sources.push("ghost_clk".into());
            }
            let mut start: Vec<String> = sources.iter().filter(|_| rnd(4) != 0).cloned().collect();
            if rnd(5) == 0 {
                start.push("not_a_source".into());
            }
            let mut names: Vec<String> = ports.clone();
            for (inst, m) in &d.insts {
                names.extend(d.masters[m].iter().map(|t| format!("{inst}/{}", t.0)));
            }
            let propagated: BTreeSet<String> = (0..rnd(3)).map(|_| names[rnd(names.len())].clone()).collect();
            let ours = find_clk_pins(&d, &lib, &sources, &start, &propagated).map_err(|e| e.to_string());
            let want = by_name::find_clk_pins(&d, &lib, &sources, &start, &propagated).map_err(|e| e.to_string());
            assert_eq!(ours, want, "seed {seed}");
            match &ours {
                Ok(n) => reached += usize::from(n.len() > 1),
                Err(_) => refused += 1,
            }
        }
        assert!(reached > 150 && refused > 30, "the generated designs exercise the search: {reached} reaching beyond their source net, {refused} refused");
    }

    /// Upstream rules (OpenSTA at the 4155ac3 pin): `Graph::makePinVertices` gives a bidirect
    /// top-level port a load and a driver vertex; `makeInstDrvrWireEdges` on the top instance joins
    /// them with an `isBidirectPortPath` wire edge, which `ClkTreeSearchPred::searchThru` never
    /// walks; `ClkNetwork::findClkPins` enqueues BOTH vertices of a clock's leaf pin. So a bidirect
    /// port that is a clock source drives its net's loads, one alone on its net is no clock pin and
    /// no refusal, and an instance's bidirect pin is still refused.
    #[test]
    fn a_bidirect_port_is_a_load_and_drives_only_as_a_clock_source() {
        const LEF: &str = "VERSION 5.8 ;\nUNITS\n  DATABASE MICRONS 1000 ;\nEND UNITS\n\
            LAYER metal1\n  TYPE ROUTING ;\n  DIRECTION HORIZONTAL ;\n  PITCH 0.2 ;\n  WIDTH 0.1 ;\nEND metal1\n\
            SITE core\n  CLASS CORE ;\n  SIZE 0.2 BY 1.4 ;\nEND core\n\
            MACRO BUF\n  CLASS CORE ;\n  ORIGIN 0 0 ;\n  SIZE 1 BY 1.4 ;\n  SITE core ;\n\
            PIN A\n    DIRECTION INPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0 0 0.1 0.1 ;\n    END\n  END A\n\
            PIN Z\n    DIRECTION OUTPUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0.5 0 0.6 0.1 ;\n    END\n  END Z\nEND BUF\n\
            MACRO PAD\n  CLASS CORE ;\n  ORIGIN 0 0 ;\n  SIZE 1 BY 1.4 ;\n  SITE core ;\n\
            PIN Y\n    DIRECTION INOUT ;\n    PORT\n      LAYER metal1 ;\n        RECT 0 0 0.1 0.1 ;\n    END\n  END Y\nEND PAD\nEND LIBRARY\n";
        const LIB: &str = r#"library (t) { capacitive_load_unit (1,ff); pulling_resistance_unit : "1kohm";
            cell (BUF) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A";
              timing () { related_pin : "A"; } } } }"#;
        let def = |pad: bool| {
            let (ncomp, comp, nnet, net) = if pad { (3, "  - p1 PAD + PLACED ( 5000 1000 ) N ;\n", 6, "  - n4 ( p1 Y ) ;\n") } else { (2, "", 5, "") };
            format!(
                "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN top ;\nUNITS DISTANCE MICRONS 1000 ;\n\
                 DIEAREA ( 0 0 ) ( 10000 10000 ) ;\nCOMPONENTS {ncomp} ;\n  - u1 BUF + PLACED ( 1000 1000 ) N ;\n  - u2 BUF + PLACED ( 3000 1000 ) N ;\n{comp}END COMPONENTS\n\
                 PINS 5 ;\n\
                 - clk + NET clk + DIRECTION INPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 1000 ) N ;\n\
                 - io1 + NET n1 + DIRECTION INOUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 3000 ) N ;\n\
                 - io2 + NET n2 + DIRECTION INOUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 5000 ) N ;\n\
                 - io3 + NET n3 + DIRECTION INOUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 7000 ) N ;\n\
                 - out + NET n5 + DIRECTION OUTPUT + USE SIGNAL\n    + LAYER metal1 ( 0 0 ) ( 100 100 )\n    + PLACED ( 0 9000 ) N ;\nEND PINS\n\
                 NETS {nnet} ;\n  - clk ( PIN clk ) ( u1 A ) ;\n  - n1 ( u1 Z ) ( PIN io1 ) ;\n  - n2 ( PIN io2 ) ( u2 A ) ;\n  - n3 ( PIN io3 ) ;\n  - n5 ( u2 Z ) ( PIN out ) ;\n{net}END NETS\nEND DESIGN\n"
            )
        };
        let dir = std::env::temp_dir().join(format!("vyges-est-bidir-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let db_of = |pad: bool| {
            let (lef, d) = (dir.join("t.lef"), dir.join(format!("t{pad}.def")));
            std::fs::write(&lef, LEF).unwrap();
            std::fs::write(&d, def(pad)).unwrap();
            let mut db = Db::new();
            db.read_lef(lef.to_str().unwrap()).unwrap();
            db.read_def(d.to_str().unwrap(), "default").unwrap();
            db
        };
        let mut lib = LibertyClocks::default();
        lib.read(LIB).unwrap();
        let db = db_of(false);
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<BTreeSet<_>>();
        // io1 is a load the clock reaches; its driver vertex is not walked, and io3 (alone) is nothing
        assert_eq!(find_clk_nets(&db, &lib, &["clk".into()]).unwrap(), s(&["clk", "n1"]));
        // a bidirect clock source drives its net: io2's driver vertex reaches u2/A, and on to n5
        assert_eq!(find_clk_nets(&db, &lib, &["io2".into()]).unwrap(), s(&["n2", "n5"]));
        assert_eq!(find_clk_nets(&db, &lib, &["clk".into(), "io2".into()]).unwrap(), s(&["clk", "n1", "n2", "n5"]));
        // an instance's bidirect pin is still refused
        let e = find_clk_nets(&db_of(true), &lib, &["clk".into()]).unwrap_err().to_string();
        assert!(e.contains("p1/Y has direction INOUT"), "{e}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
