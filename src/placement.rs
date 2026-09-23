// SPDX-License-Identifier: Apache-2.0
//! `estimate_parasitics -placement` — `EstimateParasitics::estimateWireParasitics` and the calls
//! under it, one function per reference function, in its order:
//!
//! [`estimate_wire_parasitics`] (every `dbNet`, block order) → [`estimate_wire_parasitic`] (the
//! net's first driver) → [`estimate_wire_parasitic_drvr`] (power / ground / special / pad) →
//! [`estimate_wire_parasitic_steiner`] (`isSkipPin`, then [`make_steiner_tree`]).
//!
//! This stage decides WHICH nets get a network and builds each one's Steiner tree; the network
//! itself (branches to resistors and capacitors, pin connection) is [`crate::network`].
//!
//! Every decision can also be written as one `VYGE|…` line ([`trace`]), a format a reference run
//! instrumented to print the same fields can be compared against line for line.

#![cfg(feature = "odb")]

use std::collections::BTreeSet;
use vyges_opendb::Db;

use crate::liberty::LibertyClocks;

/// One point of a Steiner tree as `stt::Branch` holds it: a location and the index of the point it
/// connects to (itself for the root).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Branch {
    pub x: i32,
    pub y: i32,
    pub n: usize,
}

/// `stt::Tree`: the first `deg` branches are the input points, the rest Steiner points.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SttTree {
    pub deg: usize,
    pub branch: Vec<Branch>,
}

/// `SteinerTreeBuilder::makeSteinerTree(x, y, drvr_index, alpha)` — injected, so this crate does
/// not carry the FLUTE tables.
pub type SteinerBuilder<'a> = &'a dyn Fn(&[i32], &[i32], usize, f32) -> SttTree;

/// What the timer knows that estimation reads.
pub struct Timing<'a> {
    /// The libraries, when any were read: pin directions and the clock network's arcs.
    pub liberty: Option<&'a LibertyClocks>,
    /// Every `create_clock`'s source ports.
    pub clock_sources: Vec<String>,
    /// `set_propagated_clock` on the clocks: a clock pin is then NOT ideal.
    pub propagated: bool,
}

/// One connected pin, as `connectedPins` / `dbNetwork::location` see it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinLoc {
    /// `pathName`: `inst/term` or the port's name.
    pub name: String,
    pub is_port: bool,
    pub x: i32,
    pub y: i32,
    pub placed: bool,
}

/// `est::SteinerTree` as the network stage reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SteinerTree {
    /// The net's pins in the (x, y) sort — the order `stt` received them.
    pub pinlocs: Vec<PinLoc>,
    pub tree: SttTree,
    /// `drvr_steiner_pt_`: the first branch at the driver's location, or `None` (`kNullPt`).
    pub drvr_pt: Option<usize>,
}

/// What became of one net.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Decision {
    /// `drivers(net)` is empty: no call at all.
    NoDriver,
    Power,
    Ground,
    Special,
    /// `isPadNet`: a port straight to a pad — `makePadParasitic` (a 1 mΩ link, no capacitance)
    /// between the net's first two connected pins.
    Pad { pins: Vec<PinLoc> },
    /// `isSkipPin(driver)`: an ideal clock.
    Skip,
    /// `makeSteinerTree` returned null: fewer than two pins, or one not placed. The pins as sorted.
    NoTree { pinlocs: Vec<PinLoc>, drvr_idx: Option<usize> },
    Tree { tree: SteinerTree, drvr_idx: Option<usize>, non_leaf_clock: bool },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetEstimate {
    pub net: String,
    pub drivers: usize,
    pub drvr: Option<String>,
    pub decision: Decision,
}

type Res<T> = Result<T, String>;

/// `PinIdLess`: `dbNetwork::id(pin)` — an instance terminal's odb id × 2, a block terminal's × 2 + 1.
pub fn pin_key(db: &Db, pin: &PinLoc) -> Res<u64> {
    if pin.is_port {
        Ok((u64::from(db.bterm_id(&pin.name).map_err(|e| e.to_string())?) << 1) | 1)
    } else {
        let (inst, term) = pin.name.rsplit_once('/').unwrap_or((&pin.name, ""));
        Ok(u64::from(db.iterm_id(inst, term).map_err(|e| e.to_string())?) << 1)
    }
}

/// The flat pins of a net in `connectedPinIterator` order — ⛔ a `PinSet`, so by `PinIdLess`
/// ([`pin_key`]), not the database's instance-terminals-then-ports order. It decides `net2Pins`
/// (the pad test and the pad resistor's direction) and the pins' order before `makeSteinerTree`'s
/// (x, y) sort, hence ties at one location. (`getFirstDriverTerm` walks the database's own lists.)
fn connected_pins(db: &Db, net: &str) -> Res<Vec<PinLoc>> {
    let mut out = Vec::new();
    for it in db.net_iterms(net) {
        let (inst, term) = it.rsplit_once('/').unwrap_or((&it, ""));
        // dbNetwork::location: the terminal's average XY, else the instance's origin.
        let (x, y) = db.iterm_avg_xy(inst, term).unwrap_or_else(|| (db.inst_get_origin_x(inst), db.inst_get_origin_y(inst)));
        out.push(PinLoc { name: it.clone(), is_port: false, x, y, placed: db.inst_is_placed(inst) });
    }
    for bt in db.net_bterms(net) {
        // A port with no placed pin reads (0, 0).
        let (x, y) = db.bterm_first_pin_location(&bt).unwrap_or((0, 0));
        let status = db.bterm_get_first_pin_placement_status(&bt);
        out.push(PinLoc { name: bt.clone(), is_port: true, x, y, placed: matches!(status.as_str(), "PLACED" | "FIRM" | "LOCKED" | "COVER") });
    }
    Ok(out)
}

/// [`connected_pins`] in `PinSet` order.
fn connected_pins_by_id(db: &Db, net: &str) -> Res<Vec<PinLoc>> {
    let mut keyed = connected_pins(db, net)?.into_iter().map(|p| pin_key(db, &p).map(|k| (k, p))).collect::<Res<Vec<_>>>()?;
    keyed.sort_by_key(|(k, _)| *k);
    Ok(keyed.into_iter().map(|(_, p)| p).collect())
}

/// `dbNetwork::drivers(net)` — ⛔ NOT OpenSTA's visitor over every driver: `dbNetwork` overrides it
/// with odb's `dbNet::getFirstDriverTerm`, so a net has AT MOST ONE driver here (`report_net`,
/// which walks the generic visitor, can list two — `make_parasitics4`'s `in1`: the input port and a
/// bidirect pad pin — while `est` sees only the pad pin).
///
/// `getFirstDriverTerm`: none on a supply net; else the first instance terminal (net order) that is
/// not supply, not clocked (`isClocked`: a CLOCK master terminal or the terminal's flag) and whose
/// LEF IO type is OUTPUT or INOUT; else the first block terminal, not supply, INPUT or INOUT. The
/// LEF types, not liberty's.
///
/// ⚠️ The `isClocked` exclusion is UNWITNESSED: dropping it changes no corpus case — no net there
/// has a clocked terminal that is also an output or inout.
fn first_driver_term<'a>(db: &Db, net: &str, pins: &'a [PinLoc]) -> Option<&'a PinLoc> {
    let supply = |s: &str| s == "POWER" || s == "GROUND";
    if supply(&db.net_sigtype(net)) {
        return None;
    }
    let iterm = pins.iter().filter(|p| !p.is_port).find(|p| {
        let (inst, term) = p.name.rsplit_once('/').unwrap_or((&p.name, ""));
        !supply(&db.iterm_get_sig_type(inst, term))
            && !db.iterm_is_clocked(inst, term)
            && matches!(db.iterm_get_io_type(inst, term).as_str(), "OUTPUT" | "INOUT")
    });
    iterm.or_else(|| {
        pins.iter().filter(|p| p.is_port).find(|p| !supply(&db.bterm_get_sig_type(&p.name)) && matches!(db.bterm_get_io_type(&p.name).as_str(), "INPUT" | "INOUT"))
    })
}

/// `isPadNet`: the net's FIRST TWO connected pins (`net2Pins`) are a top-level port and a pad pin —
/// an instance of a PAD-class or COVER_BUMP master (`isPad`), in either order. Only those two are
/// read, whatever else the net connects.
fn is_pad_net(db: &Db, pins: &[PinLoc]) -> Res<bool> {
    let [p1, p2, ..] = pins else { return Ok(false) };
    let is_pad_pin = |p: &PinLoc| -> Res<bool> {
        if p.is_port {
            return Ok(false);
        }
        let inst = p.name.rsplit_once('/').map_or(p.name.as_str(), |(i, _)| i);
        let ty = db.master_get_type(&db.inst_get_master(inst)).map_err(|e| e.to_string())?;
        Ok(ty.starts_with("PAD") || ty == "COVER_BUMP" || ty == "COVER BUMP")
    };
    Ok((p1.is_port && is_pad_pin(p2)?) || (p2.is_port && is_pad_pin(p1)?))
}

/// `estimateWireParasitics`: every `dbNet` in block order.
///
/// ⛔ Refused rather than guessed: constant pins (`isConstant`: tie cells, case analysis) — a driver
/// whose cell's output function is a constant is refused.
pub fn estimate_wire_parasitics(db: &Db, timing: &Timing<'_>, alpha: f32, stt: SteinerBuilder<'_>) -> Res<Vec<NetEstimate>> {
    let clock_nets: BTreeSet<String> = match timing.liberty {
        Some(lib) if !timing.clock_sources.is_empty() => {
            crate::clk_network::find_clk_nets(db, lib, &timing.clock_sources).map_err(|e| e.to_string())?
        }
        _ => BTreeSet::new(),
    };
    let mut out = Vec::new();
    for net in db.net_names() {
        out.push(estimate_wire_parasitic(db, timing, &clock_nets, &net, alpha, stt)?);
    }
    Ok(out)
}

/// `estimateWireParasitic(net)`: the net's first driver, if it has one.
fn estimate_wire_parasitic(db: &Db, timing: &Timing<'_>, clock_nets: &BTreeSet<String>, net: &str, alpha: f32, stt: SteinerBuilder<'_>) -> Res<NetEstimate> {
    // getFirstDriverTerm walks the database's lists: instance terminals, then block terminals.
    let db_order = connected_pins(db, net)?;
    let Some(drvr) = first_driver_term(db, net, &db_order).cloned() else {
        return Ok(NetEstimate { net: net.to_string(), drivers: 0, drvr: None, decision: Decision::NoDriver });
    };
    let pins = connected_pins_by_id(db, net)?;
    let decision = estimate_wire_parasitic_drvr(db, timing, clock_nets, net, &drvr, &pins, alpha, stt)?;
    Ok(NetEstimate { net: net.to_string(), drivers: 1, drvr: Some(drvr.name.clone()), decision })
}

/// `estimateWireParasitic(drvr, net)`: power, ground and special nets get nothing; a pad net its
/// own model; every other net the Steiner estimate.
#[allow(clippy::too_many_arguments)]
fn estimate_wire_parasitic_drvr(db: &Db, timing: &Timing<'_>, clock_nets: &BTreeSet<String>, net: &str, drvr: &PinLoc, pins: &[PinLoc], alpha: f32, stt: SteinerBuilder<'_>) -> Res<Decision> {
    match db.net_sigtype(net).as_str() {
        "POWER" => return Ok(Decision::Power),
        "GROUND" => return Ok(Decision::Ground),
        _ => {}
    }
    if db.net_is_special(net) {
        return Ok(Decision::Special);
    }
    if is_pad_net(db, pins)? {
        return Ok(Decision::Pad { pins: pins.to_vec() });
    }
    estimate_wire_parasitic_steiner(db, timing, clock_nets, net, drvr, pins, alpha, stt)
}

/// `estimateWireParasiticSteiner`, up to the tree: `isSkipPin(driver)`, then `makeSteinerTree`.
///
/// `isSkipPin`: a pin that is a clock (in the clock network — its net is one `findClkNets`
/// reaches) and ideal (no `set_propagated_clock`) gets no network.
#[allow(clippy::too_many_arguments)]
fn estimate_wire_parasitic_steiner(db: &Db, timing: &Timing<'_>, clock_nets: &BTreeSet<String>, net: &str, drvr: &PinLoc, pins: &[PinLoc], alpha: f32, stt: SteinerBuilder<'_>) -> Res<Decision> {
    if clock_nets.contains(net) && !timing.propagated {
        return Ok(Decision::Skip);
    }
    // isConstant: a tie cell's output is a constant pin — skipped by the reference, not modelled.
    if !drvr.is_port {
        let (inst, term) = drvr.name.rsplit_once('/').unwrap_or((&drvr.name, ""));
        if timing.liberty.and_then(|l| l.cells.get(&db.inst_get_master(inst))).is_some_and(|c| c.constant_outputs.contains(term)) {
            return Err(format!("net {net}: driven by a constant (tie) output — isConstant is not modelled"));
        }
    }
    let (pinlocs, drvr_idx, tree) = make_steiner_tree(drvr, pins, alpha, stt);
    Ok(match tree {
        Some(tree) => {
            // isNonLeafClock: a CLOCK-typed net none of whose terminals is a clock terminal.
            let facts: Vec<crate::liberty::ITermClockFacts> = db
                .net_iterms(net)
                .iter()
                .map(|it| {
                    let (inst, term) = it.rsplit_once('/').unwrap_or((it, ""));
                    timing.liberty.map_or(crate::liberty::ITermClockFacts { has_liberty_port: false, is_reg_clk: false, cell_is_pad: false }, |l| l.iterm_facts(&db.inst_get_master(inst), term))
                })
                .collect();
            Decision::Tree { tree, drvr_idx, non_leaf_clock: crate::liberty::is_non_leaf_clock(db.net_sigtype(net) == "CLOCK", &facts) }
        }
        None => Decision::NoTree { pinlocs, drvr_idx },
    })
}

/// `makeSteinerTree(drvr_pin)`: the pins sorted by (x, y), the tree built from the driver's index,
/// then `setTree` (the driver's Steiner point: the first branch at its location). Returns the sorted
/// pins, the driver's index among them, and the tree — `None` with fewer than two pins or one not
/// placed.
///
/// ⚠️ The sort is `std::sort` — unstable. On pins sharing a location the reference's order is its
/// algorithm's; a tie is kept in `connectedPinIterator` order here (libc++'s small-range insertion
/// sort does the same), which the corpus has not yet contradicted.
pub fn make_steiner_tree(drvr: &PinLoc, pins: &[PinLoc], alpha: f32, stt: SteinerBuilder<'_>) -> (Vec<PinLoc>, Option<usize>, Option<SteinerTree>) {
    let mut pinlocs = pins.to_vec();
    pinlocs.sort_by_key(|p| (p.x, p.y));
    let drvr_idx = pinlocs.iter().position(|p| p.name == drvr.name);
    if pinlocs.len() < 2 || pinlocs.iter().any(|p| !p.placed) {
        return (pinlocs, drvr_idx, None);
    }
    let x: Vec<i32> = pinlocs.iter().map(|p| p.x).collect();
    let y: Vec<i32> = pinlocs.iter().map(|p| p.y).collect();
    // `drvr_idx` starts at 0 in the reference and is overwritten only when the driver is found.
    let tree = stt(&x, &y, drvr_idx.unwrap_or(0), alpha);
    let drvr_pt = tree.branch.iter().position(|b| b.x == drvr.x && b.y == drvr.y);
    let t = SteinerTree { pinlocs: pinlocs.clone(), tree, drvr_pt };
    (pinlocs, drvr_idx, Some(t))
}

/// The decisions as the instrumented reference prints them (`VYGE|…`).
pub fn trace(nets: &[NetEstimate]) -> String {
    let pins_line = |net: &str, pins: &[PinLoc], drvr_idx: Option<usize>| {
        let ps: String = pins.iter().map(|p| format!("{}@{},{};", p.name, p.x, p.y)).collect();
        format!("VYGE|pins|{net}|drvr_idx={}|{ps}\n", drvr_idx.map_or(-1, |i| i as i64))
    };
    let mut t = String::new();
    for n in nets {
        t.push_str(&format!("VYGE|net|{}|drivers={}|drvr={}\n", n.net, n.drivers, n.drvr.as_deref().unwrap_or("")));
        let kind = match &n.decision {
            Decision::NoDriver => continue,
            Decision::Power => "power",
            Decision::Ground => "ground",
            Decision::Special => "special",
            Decision::Pad { .. } => "pad",
            _ => "steiner",
        };
        t.push_str(&format!("VYGE|kind|{}|{kind}\n", n.net));
        match &n.decision {
            Decision::Skip => t.push_str(&format!("VYGE|skip|{}\n", n.net)),
            Decision::NoTree { pinlocs, drvr_idx } => {
                t.push_str(&pins_line(&n.net, pinlocs, *drvr_idx));
                if pinlocs.len() >= 2 {
                    t.push_str(&format!("VYGE|noplace|{}\n", n.net));
                }
            }
            Decision::Tree { tree, drvr_idx, non_leaf_clock } => {
                t.push_str(&pins_line(&n.net, &tree.pinlocs, *drvr_idx));
                let br: String = tree.tree.branch.iter().map(|b| format!("{},{},{};", b.x, b.y, b.n)).collect();
                let dp = tree.drvr_pt.map_or(-1, |p| p as i64);
                t.push_str(&format!("VYGE|tree|{}|pins={}|drvr_pt={dp}|{br}\n", n.net, tree.pinlocs.len()));
                t.push_str(&format!("VYGE|clk|{}|{}\n", n.net, *non_leaf_clock as i32));
            }
            _ => {}
        }
    }
    t
}
