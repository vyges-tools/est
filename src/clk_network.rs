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
//! ⛔ Refused, not modelled: a bidirect pin on the way (`isBidirectInstPath`/`PortPath` edges), a
//! cycle among the traversed edges (the levelizer disables a loop edge — `isDisabledLoop`), a net
//! with ten or more drivers (`isIsolatedNet` may drop its edges), and a cell whose roles
//! `inferLatchRoles` may rewrite. No SDC is read, so nothing is disabled by constraint.

#![cfg(feature = "odb")]

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use vyges_opendb::Db;

use crate::liberty::{LibertyClocks, Role};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// A timing-graph pin: a top-level port, or an instance terminal.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Pin {
    Port(String),
    ITerm(String, String),
}

/// `findClkNets` for clocks defined on the top-level ports `sources`, in the database's net names.
pub fn find_clk_nets(db: &Db, lib: &LibertyClocks, sources: &[String]) -> Res<BTreeSet<String>> {
    // The netlist as the graph sees it: each pin's net, each net's drivers and loads.
    let mut net_of: BTreeMap<Pin, String> = BTreeMap::new();
    let mut drivers: BTreeMap<String, Vec<Pin>> = BTreeMap::new();
    let mut loads: BTreeMap<String, Vec<Pin>> = BTreeMap::new();
    let mut masters: BTreeMap<String, String> = BTreeMap::new();
    for net in db.net_names() {
        let sig = db.net_sigtype(&net);
        if sig == "POWER" || sig == "GROUND" {
            continue; // power and ground pins have no vertices
        }
        let mut pins = Vec::new();
        for b in db.net_bterms(&net) {
            let dir = db.bterm_get_io_type(&b);
            pins.push((Pin::Port(b), match dir.as_str() {
                "INPUT" => true,
                "OUTPUT" => false,
                d => return Err(format!("clock network: port on net {net} has direction {d} — not modelled").into()),
            }));
        }
        for it in db.net_iterms(&net) {
            let Some((inst, mterm)) = it.split_once('/') else { continue };
            let master = masters.entry(inst.to_string()).or_insert_with(|| db.inst_master(inst)).clone();
            let dir = db.mterm_get_io_type(&master, mterm);
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
    for s in &leaf {
        if visited.insert(s.clone()) {
            queue.push_back(s.clone());
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
                let terms = mterms_of.entry(master.clone()).or_insert_with(|| db.master_mterms(&master).map(|v| v.into_iter().map(|(n, _)| n).collect()).unwrap_or_default());
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
                succ.entry(p).or_default().extend(loads.get(net).into_iter().flatten().filter(|l| visited.contains(*l)));
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
