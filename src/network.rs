// SPDX-License-Identifier: Apache-2.0
//! Stage 3 — the RC network of one net for one corner: `estimateWireParasiticSteiner`'s corner
//! loop, `parasiticNodeConnectPins`, `insertViaResistances`, `computeAverageCutResistance`, and
//! `makePadParasitic` — into a [`Parasitic`] laid out as OpenSTA's `ConcreteParasiticNetwork`
//! holds it, because the SPEF writer walks that storage in its own order.
//!
//! ⛔ **The storage order is the output order.** Pin nodes live in a map keyed by `PinIdLess` (an
//! instance terminal's odb id × 2, a block terminal's × 2 + 1); Steiner nodes in a map keyed by
//! index; resistors in creation order. `*CONN` walks the pin nodes, `*CAP` the Steiner nodes with a
//! non-zero capacitance, `*RES` the resistors.
//!
//! ⛔ **Everything stored is `float`**: `incrCap(node, float)`, `makeResistor(…, float res, …)`.
//! The arithmetic before it is `double` (`dbuToMeters`, the H/V weighting, `cap / 2.0`).

#![cfg(feature = "odb")]

use std::collections::{BTreeMap, BTreeSet};
use vyges_opendb::Db;

use crate::placement::{PinLoc, SteinerTree};
use crate::rc::Rc;

type Res<T> = Result<T, String>;

/// A node of the network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Node {
    /// A pin, by its `PinIdLess` key.
    Pin(u64),
    /// A Steiner (or via-chain) point of the net: `net:<id>`.
    Sub(u32),
}

/// A pin node's facts the writer prints.
#[derive(Debug, Clone, PartialEq)]
pub struct PinNode {
    /// `pathName`: `inst/term` or the port.
    pub name: String,
    pub is_port: bool,
    /// The terminal's LEF IO type (`INPUT`, `OUTPUT`, `INOUT`, …).
    pub io_type: String,
    /// The instance's master, for an instance terminal.
    pub master: String,
    pub cap: f32,
}

/// `ConcreteParasiticNetwork`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Parasitic {
    pub pin_nodes: BTreeMap<u64, PinNode>,
    pub sub_nodes: BTreeMap<u32, f32>,
    pub resistors: Vec<(Node, Node, f32)>,
}

impl Parasitic {
    fn ensure_sub(&mut self, id: u32) -> Node {
        self.sub_nodes.entry(id).or_insert(0.0);
        Node::Sub(id)
    }
    fn incr_cap(&mut self, n: Node, cap: f32) {
        match n {
            Node::Sub(id) => *self.sub_nodes.get_mut(&id).expect("ensured") += cap,
            Node::Pin(k) => self.pin_nodes.get_mut(&k).expect("ensured").cap += cap,
        }
    }
    fn make_resistor(&mut self, a: Node, b: Node, res: f64) {
        self.resistors.push((a, b, res as f32));
    }
    /// `ConcreteParasiticNetwork::capacitance`: a FLOAT sum, Steiner nodes (by id) first, then pin
    /// nodes (by key).
    ///
    /// ⚠️ The order is transcribed but EQUIVALENT here: `est` never adds capacitance to a pin node,
    /// so the pin terms are all zero and summing them first changes nothing (a mutant survives).
    pub fn capacitance(&self) -> f32 {
        let mut c = 0.0f32;
        for v in self.sub_nodes.values() {
            c += *v;
        }
        for p in self.pin_nodes.values() {
            c += p.cap;
        }
        c
    }
}

/// What the network stage reads beyond the tree.
pub struct NetCtx<'a> {
    pub db: &'a Db,
    pub rc: &'a Rc,
    pub tech: &'a str,
    pub corner: usize,
    pub is_clk: bool,
}

/// `ensureParasiticNode(parasitic, pin)`, with the facts the writer needs.
fn ensure_pin(db: &Db, g: &mut Parasitic, pin: &PinLoc) -> Res<Node> {
    let key = crate::placement::pin_key(db, pin)?;
    if !g.pin_nodes.contains_key(&key) {
        let (io_type, master) = if pin.is_port {
            (db.bterm_get_io_type(&pin.name), String::new())
        } else {
            let (inst, term) = pin.name.rsplit_once('/').unwrap_or((&pin.name, ""));
            (db.iterm_get_io_type(inst, term), db.inst_get_master(inst))
        };
        g.pin_nodes.insert(key, PinNode { name: pin.name.clone(), is_port: pin.is_port, io_type, master, cap: 0.0 });
    }
    Ok(Node::Pin(key))
}

/// `dbuToMeters`: `dist / (dbu * 1e6)`.
fn dbu_to_meters(db: &Db, d: i32) -> f64 {
    f64::from(d) / (f64::from(db.tech_get_db_units_per_micron()) * 1e6)
}

/// `estimateWireParasiticSteiner`'s corner body: per branch a pi model (or a 1 mΩ link for a
/// zero-length branch), then each end's pins connected once.
pub fn make_steiner_parasitic(cx: &NetCtx<'_>, net: &str, tree: &SteinerTree) -> Res<Parasitic> {
    let db = cx.db;
    let mut g = Parasitic::default();
    let mut connected: BTreeSet<String> = BTreeSet::new();
    let v = cx.rc.resolved(cx.tech, cx.corner);
    // [sig_h_r, sig_v_r, sig_h_c, sig_v_c, clk_h_r, clk_v_r, clk_h_c, clk_v_c]
    let (h_r, v_r, h_c, v_c) = if cx.is_clk { (v[4], v[5], v[6], v[7]) } else { (v[0], v[1], v[2], v[3]) };
    let br = &tree.tree.branch;
    // getMaxIndex: the largest point index a branch names.
    let mut max_node_index: i64 = br.iter().enumerate().map(|(i, b)| (i as i64).max(b.n as i64)).max().unwrap_or(-1);
    let ndr_ratio = ndr_ratio(db, net)?;
    for i in 0..br.len() {
        let (p1, p2) = (br[i], br[br[i].n]);
        let len = (p1.x - p2.x).abs() + (p1.y - p2.y).abs();
        let (wire_cap, wire_res) = if len != 0 {
            let dx = dbu_to_meters(db, (p1.x - p2.x).abs()) / dbu_to_meters(db, len);
            let dy = dbu_to_meters(db, (p1.y - p2.y).abs()) / dbu_to_meters(db, len);
            (dx * h_c + dy * v_c, dx * h_r + dy * v_r)
        } else {
            // wire{Signal,Clk}{Capacitance,Resistance}: (h + v) / 2.
            ((h_c + v_c) / 2.0, (h_r + v_r) / 2.0)
        };
        let n1 = g.ensure_sub(i as u32);
        let n2 = g.ensure_sub(br[i].n as u32);
        if len == 0 {
            g.make_resistor(n1, n2, 1.0e-3);
        } else {
            let length = dbu_to_meters(db, len);
            let cap = length * wire_cap;
            let mut res = length * wire_res;
            if let Some(r) = ndr_ratio {
                res /= f64::from(r);
            }
            g.incr_cap(n1, (cap / 2.0) as f32);
            g.make_resistor(n1, n2, res);
            g.incr_cap(n2, (cap / 2.0) as f32);
        }
        parasitic_node_connect_pins(cx, &mut g, n1, tree, i, &mut connected, &mut max_node_index)?;
        parasitic_node_connect_pins(cx, &mut g, n2, tree, br[i].n, &mut connected, &mut max_node_index)?;
    }
    Ok(g)
}

/// The NDR's width ratio (`makeWireParasitic`/`estimateWireParasiticSteiner`): the FIRST layer
/// rule's width over its layer's own, as `float`.
fn ndr_ratio(db: &Db, net: &str) -> Res<Option<f32>> {
    let ndr = db.net_get_non_default_rule(net);
    if ndr.is_empty() {
        return Ok(None);
    }
    let rules = db.ndr_layer_rules(&ndr).map_err(|e| e.to_string())?;
    let (layer, width, _) = rules.first().ok_or_else(|| format!("net {net}: an NDR with no layer rule"))?;
    Ok(Some(*width as f32 / db.layer_get_width(layer) as f32))
}

/// `parasiticNodeConnectPins`: the pins at a Steiner point that is a pin location (`pt < deg`), each
/// connected once — through a via chain when the wire layers and the layer table are known,
/// else one averaged cut resistance.
fn parasitic_node_connect_pins(cx: &NetCtx<'_>, g: &mut Parasitic, node: Node, tree: &SteinerTree, pt: usize, connected: &mut BTreeSet<String>, max_node_index: &mut i64) -> Res<()> {
    if pt >= tree.tree.deg {
        return Ok(());
    }
    let loc = (tree.tree.branch[pt].x, tree.tree.branch[pt].y);
    // loc_pin_map_[location(pt)]: every pin there, in the order the sorted pins were added.
    let pins: Vec<&PinLoc> = tree.pinlocs.iter().filter(|p| (p.x, p.y) == loc).collect();
    let layers = if cx.is_clk { cx.rc.resolve(cx.tech, |w| &w.clk_layers) } else { cx.rc.resolve(cx.tech, |w| &w.signal_layers) };
    let tree_layer = layers.first().copied();
    for pin in pins {
        let pin_node = ensure_pin(cx.db, g, pin)?;
        if connected.contains(&pin.name) {
            continue;
        }
        match tree_layer {
            Some(t) if !cx.rc.layer_res.is_empty() => {
                let p = get_pin_layer(cx.db, pin)?;
                insert_via_resistances(cx, g, p, t, pin_node, node, max_node_index)?;
            }
            _ => {
                let cut_res = compute_average_cut_resistance(cx)?.max(1.0e-3);
                g.make_resistor(node, pin_node, cut_res);
            }
        }
        connected.insert(pin.name.clone());
    }
    Ok(())
}

/// `getPinLayer`: an instance terminal's lowest ROUTING layer among its geometry; a port's first
/// pin shape's layer. Returned as the layer NUMBER.
fn get_pin_layer(db: &Db, pin: &PinLoc) -> Res<i32> {
    if pin.is_port {
        let boxes = db.bpin_layer_boxes(&pin.name, 0).map_err(|e| e.to_string())?;
        return boxes.first().map(|b| b.0 as i32).ok_or_else(|| format!("[ERROR EST-0164] sta::Pin {} has no placed iterm or bterm.", pin.name));
    }
    let (inst, term) = pin.name.rsplit_once('/').unwrap_or((&pin.name, ""));
    db.iterm_pin_boxes(inst, term).iter().map(|b| b.layer as i32).min().ok_or_else(|| format!("pin {}: no routing geometry", pin.name))
}

/// `insertViaResistances`: pin layer to tree layer (layer NUMBERS). Two numbers apart: one cut, one
/// resistor pin → tree. Equal: 1 mΩ. Otherwise a chain over every CUT layer between, a new node
/// (`++max_node_index`) at each step but the last. Each cut at least 1 mΩ.
fn insert_via_resistances(cx: &NetCtx<'_>, g: &mut Parasitic, p: i32, t: i32, pin_node: Node, node: Node, max_node_index: &mut i64) -> Res<()> {
    let cut_rc = |l: i32| cx.rc.layer_rc(l, cx.corner).0;
    if (p - t).abs() == 2 {
        let cut = if p < t { p + 1 } else { p - 1 };
        g.make_resistor(pin_node, node, cut_rc(cut).max(1.0e-3));
    } else if p == t {
        g.make_resistor(pin_node, node, 1.0e-3);
    } else {
        let (start, end) = (p.min(t), p.max(t));
        let pin_is_below = p < t;
        let mut prev: Option<Node> = None;
        for l in start..end {
            let name = cx.db.layer_name_by_number(i64::from(l));
            if cx.db.layer_get_type(&name).map_err(|e| e.to_string())? != "CUT" {
                continue;
            }
            let cut_res = cut_rc(l).max(1.0e-3);
            let mut from = prev;
            let mut to = None;
            let mut need_mid = true;
            if pin_is_below {
                if l - 1 == p {
                    from = Some(pin_node);
                }
                if l + 1 == t {
                    to = Some(node);
                    need_mid = false;
                }
            } else {
                if l - 1 == t {
                    from = Some(node);
                }
                if l + 1 == p {
                    to = Some(pin_node);
                    need_mid = false;
                }
            }
            let mut mid = None;
            if need_mid {
                *max_node_index += 1;
                let m = g.ensure_sub(*max_node_index as u32);
                mid = Some(m);
                to = Some(m);
            }
            let (Some(a), Some(b)) = (from, to) else {
                return Err(format!("via chain from layer {p} to {t}: a step with no node — not modelled"));
            };
            g.make_resistor(a, b, cut_res);
            prev = mid;
        }
    }
    Ok(())
}

/// `computeAverageCutResistance`: the mean table resistance of the CUT layers between the block's
/// min and max routing layers (0 with no table).
///
/// A block with no max routing layer (`< 0`, nothing set it) takes the MIDDLE of the frontside
/// stack: `first_level - 1 + (levels - first_level + 1) / 2` (integer), the first frontside level
/// being the first routing level that is not backside; with no frontside layer, half the levels.
fn compute_average_cut_resistance(cx: &NetCtx<'_>) -> Res<f64> {
    if cx.rc.layer_res.is_empty() {
        return Ok(0.0);
    }
    let db = cx.db;
    let (min, mut max) = (db.block_get_min_routing_layer(), db.block_get_max_routing_layer());
    if max < 0 {
        let total = db.tech_get_routing_layer_count();
        let first_front = db.tech_first_frontside_routing_layer();
        max = if first_front.is_empty() {
            total / 2
        } else {
            let first_level = db.layer_get_routing_level(&first_front);
            first_level - 1 + (total - first_level + 1) / 2
        };
    }
    let by_level = |lvl: i32| db.tech_get_layers().into_iter().find(|l| db.layer_get_routing_level(l) == lvl);
    let (lo, hi) = (by_level(min).ok_or("min routing layer")?, by_level(max).ok_or("max routing layer")?);
    let (mut total, mut count) = (0.0f64, 0);
    for n in db.layer_get_number(&lo)..=db.layer_get_number(&hi) {
        let name = db.layer_name_by_number(i64::from(n));
        if !name.is_empty() && db.layer_get_type(&name).map_err(|e| e.to_string())? == "CUT" {
            total += cx.rc.layer_rc(n, cx.corner).0;
            count += 1;
        }
    }
    Ok(if count > 0 { total / f64::from(count) } else { 0.0 })
}

/// `makePadParasitic`: the net's first two connected pins joined by 1 mΩ, nothing else.
pub fn make_pad_parasitic(db: &Db, pins: &[PinLoc]) -> Res<Parasitic> {
    let mut g = Parasitic::default();
    let (a, b) = (ensure_pin(db, &mut g, &pins[0])?, ensure_pin(db, &mut g, &pins[1])?);
    g.make_resistor(a, b, 0.001);
    Ok(g)
}
