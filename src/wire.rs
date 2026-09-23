// SPDX-License-Identifier: Apache-2.0
//! `MakeWireParasitics` — the RC network the timer reads for a globally routed net.
//!
//! One node per routing point the route visits, one resistor per segment, each segment's
//! capacitance split half to each end, and a resistor from every pin to the grid node it attaches
//! to. This is what `estimateAllGlobalRouteParasitics` builds for EVERY net before the router
//! reads a slack, and what the delay calculator then reduces.
//!
//! ⛔ Each layer's R and C per metre come from `set_layer_rc` (the estimator's own table), and
//! only fall back to the technology's own values when that table has none — the table is set in
//! the same command that writes the technology, so a design with an RC file uses the TABLE.
//!
//! ⛔ The precision is the reference's: `layerRC` works in `double` and narrows to `float` on
//! return, `incrCap` takes a `float` (so `cap / 2.0` narrows), and the pin attachment's floor is
//! a `float` compare.

use std::collections::BTreeMap;

/// A route segment as `grt::GSegment` gives it: dbu ends and their routing levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    pub init_x: i32,
    pub init_y: i32,
    pub init_layer: i32,
    pub final_x: i32,
    pub final_y: i32,
    pub final_layer: i32,
}

impl Segment {
    /// `GSegment::GSegment(x0, y0, l0, x1, y1, l1)`.
    ///
    /// ⛔ The COORDINATES are sorted — `init` takes the minimum of each, `final` the maximum — so
    /// a segment built from a route walked backwards is stored forwards. The LAYERS are not:
    /// `init_layer` is `l0` whichever way round it is. Without this the parasitic nodes are
    /// created in the other order and every node of the net is numbered differently.
    pub fn new(x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32) -> Self {
        Segment { init_x: x0.min(x1), init_y: y0.min(y1), init_layer: l0, final_x: x0.max(x1), final_y: y0.max(y1), final_layer: l1 }
    }
    /// `GSegment::length` — the Manhattan length in dbu, zero for a via.
    pub fn length(&self) -> i32 {
        (self.init_x - self.final_x).abs() + (self.init_y - self.final_y).abs()
    }
    /// `GSegment::isVia` — ⚠️ the two ENDS coincide; the layers are not consulted, so a
    /// zero-length segment on one layer is a via too.
    pub fn is_via(&self) -> bool {
        self.init_x == self.final_x && self.init_y == self.final_y
    }
}

/// One pin as `getPinGridPositions` reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinGridLocation {
    /// The pin's own name, as the parasitic node is named after it.
    pub name: String,
    /// Which of `PinGridLocation`'s two terminals this is — a block terminal (a port) or an
    /// instance terminal.
    pub is_port: bool,
    /// Whether the pin drives the net. ⚠️ Not read here; carried for a SPEF `*CONN` direction.
    pub is_driver: bool,
    /// `Pin::getPosition` — dbu.
    pub pt: (i32, i32),
    /// `Pin::getOnGridPosition` — dbu.
    pub grid_pt: (i32, i32),
    /// `Pin::getConnectionLayer`.
    pub conn_layer: i32,
}

/// The per-layer electrical data, indexed by ROUTING LEVEL (1-based, as the reference indexes it).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LayerRC {
    /// `EstimateParasitics::layerRC` — ohms per metre and farads per metre from `set_layer_rc`,
    /// zero where the command set none.
    pub table_res: BTreeMap<i32, f64>,
    pub table_cap: BTreeMap<i32, f64>,
    /// The technology's own values, the fallback: width (dbu), resistance (ohm/square),
    /// capacitance and edge capacitance (pF per square micron / per micron).
    pub width: BTreeMap<i32, i32>,
    pub resistance: BTreeMap<i32, f64>,
    pub capacitance: BTreeMap<i32, f64>,
    pub edge_capacitance: BTreeMap<i32, f64>,
    /// The cut layer ABOVE each routing level: its table resistance and its own.
    pub cut_table_res: BTreeMap<i32, f64>,
    pub cut_resistance: BTreeMap<i32, f64>,
    pub dbu_per_micron: i32,
}

impl LayerRC {
    /// `MakeWireParasitics::dbuToMeters`: `dbu / (dbu_per_micron * 1e6)`.
    fn dbu_to_meters(&self, dbu: i32) -> f64 {
        f64::from(dbu) / (f64::from(self.dbu_per_micron) * 1e6)
    }

    /// `layerRC(wire_length_dbu, layer, corner, net, res, cap)` — the resistance and capacitance
    /// of `length` dbu of wire on `layer`.
    ///
    /// ⛔ The table wins; the technology's values are read only where the table is zero.
    /// `layer_width` narrows to `float` first, so the fallback divides in `float`.
    /// ⚠️ `ndr_ratio` divides the resistance per metre, not the result.
    pub fn layer_rc(&self, wire_length_dbu: i32, layer: i32, ndr_width: Option<i32>) -> (f32, f32) {
        let width_dbu = self.width.get(&layer).copied().unwrap_or(0);
        let layer_width = (f64::from(width_dbu) / f64::from(self.dbu_per_micron)) as f32;
        let mut r_per_meter = self.table_res.get(&layer).copied().unwrap_or(0.0);
        let mut cap_per_meter = self.table_cap.get(&layer).copied().unwrap_or(0.0);
        if r_per_meter == 0.0 {
            let res_ohm_per_micron = (self.resistance.get(&layer).copied().unwrap_or(0.0) / f64::from(layer_width)) as f32;
            r_per_meter = 1e6 * f64::from(res_ohm_per_micron);
        }
        if cap_per_meter == 0.0 {
            let cap = self.capacitance.get(&layer).copied().unwrap_or(0.0);
            let edge = self.edge_capacitance.get(&layer).copied().unwrap_or(0.0);
            let cap_pf_per_micron = (f64::from(layer_width) * cap + 2.0 * edge) as f32;
            cap_per_meter = 1e6 * 1e-12 * f64::from(cap_pf_per_micron);
        }
        if let Some(ndr) = ndr_width {
            let ndr_ratio = ndr as f32 / width_dbu as f32;
            r_per_meter /= f64::from(ndr_ratio);
        }
        // ⛔ `const float wire_length = dbuToMeters(...)` — the LENGTH narrows to a float before
        // the multiply: 7200 dbu is 7.19999975e-6 m, not 7.2e-6, and the resistance follows it.
        let wire_length = f64::from(self.dbu_to_meters(wire_length_dbu) as f32);
        ((r_per_meter * wire_length) as f32, (cap_per_meter * wire_length) as f32)
    }

    /// `getCutLayerRes(cut_layer, corner, num_cuts = 1)` for the cut layer ABOVE `layer` — the
    /// table's value, else the cut layer's own. ⚠️ Divided by the cut count, one here.
    fn cut_layer_res_above(&self, layer: i32) -> f32 {
        let res = match self.cut_table_res.get(&layer).copied().unwrap_or(0.0) {
            0.0 => self.cut_resistance.get(&layer).copied().unwrap_or(0.0),
            r => r,
        };
        (res / 1.0) as f32
    }
}

/// A node of the network: the pin it is, or the routing point it sits on.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum NodeId {
    /// `ensureParasiticNode(parasitic, pin, network)` — named after the pin.
    Pin(String),
    /// `ensureParasiticNode(parasitic, net, id, network)` — `id` is the node map's SIZE when the
    /// node was made, so ids follow the order the route's points were first visited.
    Point(usize),
}

/// The network as the builder leaves it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Network {
    /// Ground capacitance per node, in the order the nodes were created.
    pub nodes: Vec<(NodeId, f32)>,
    /// `makeResistor(parasitic, id, value, n1, n2)`, in creation order.
    pub resistors: Vec<(NodeId, NodeId, f32)>,
    /// What the reference warns about and skips: a pin with no grid node (EST-26/EST-350), a
    /// segment that is neither wire nor via (EST-25).
    pub warnings: Vec<String>,
}

impl Network {
    fn node_index(&mut self, id: &NodeId) -> usize {
        if let Some(i) = self.nodes.iter().position(|(n, _)| n == id) {
            return i;
        }
        self.nodes.push((id.clone(), 0.0));
        self.nodes.len() - 1
    }
    /// `incrCap(node, cap)` — ⚠️ the sum is kept in `float`.
    fn incr_cap(&mut self, id: &NodeId, cap: f32) {
        let i = self.node_index(id);
        self.nodes[i].1 += cap;
    }
    fn make_resistor(&mut self, n1: &NodeId, n2: &NodeId, res: f32) {
        let (a, b) = (self.node_index(n1), self.node_index(n2));
        let (a, b) = (self.nodes[a].0.clone(), self.nodes[b].0.clone());
        self.resistors.push((a, b, res));
    }
}

/// Where a route point maps to its node — `NodeRoutePtMap`, keyed by `(x, y, layer)`.
type NodeMap = BTreeMap<(i32, i32, i32), NodeId>;

/// `ensureParasiticNode(x, y, layer, …)`: the node for a routing point, made on first visit with
/// the map's current SIZE as its id.
fn ensure_point_node(map: &mut NodeMap, x: i32, y: i32, layer: i32) -> NodeId {
    let key = (x, y, layer);
    if let Some(n) = map.get(&key) {
        return n.clone();
    }
    let node = NodeId::Point(map.len());
    map.insert(key, node.clone());
    node
}

/// Which builder made the route, and so where a pin attaches to it.
///
/// ⛔ `makePartialParasiticsToPin` (the planar routes, mid-run) tries `net_min_layer + 1`;
/// `makeParasiticsToPin` (the routes after layer assignment) tries the PIN's own
/// `conn_layer + 1`. Everything else about the two is the same.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinAttach {
    /// `estimateParasitics(net, route)` — a planar route.
    Planar,
    /// `estimateParasitics(net, route, spef_writer)` — a route with real layers.
    Routed,
}

/// One net's inputs.
#[derive(Debug, Clone)]
pub struct NetParasitics<'a> {
    pub name: &'a str,
    pub route: &'a [Segment],
    pub pins: &'a [PinGridLocation],
    /// `getNetLayerRange(net, min, max)`'s minimum — the PARTIAL path attaches pins relative to
    /// it, not to the pin's own connection layer.
    pub net_min_layer: i32,
    /// `getMinRoutingLayer()`.
    pub min_routing_layer: i32,
    /// The net's non-default rule width on a layer, when it has one.
    pub ndr_width: Option<&'a BTreeMap<i32, i32>>,
    pub attach: PinAttach,
}

/// `makeRouteParasitics` then `makePartialParasiticsToPins` — the network for one net, as
/// `estimateParasitics(net, route)` (the 2D path, which is what the router's own slack reads)
/// builds it.
pub fn estimate_net(net: &NetParasitics<'_>, rc: &LayerRC) -> Network {
    let mut g = Network::default();
    let mut map: NodeMap = BTreeMap::new();
    make_route_parasitics(net, rc, &mut g, &mut map);
    for pin in net.pins {
        make_partial_parasitics_to_pin(net, pin, rc, &mut g, &mut map);
    }
    g
}

/// `makeRouteParasitics`: a node per end, a resistor between them, half the capacitance each end.
///
/// ⛔ An end BELOW the minimum routing layer has no node unless the segment is a via, and a
/// segment with either end missing is skipped — the route stays disconnected there.
fn make_route_parasitics(net: &NetParasitics<'_>, rc: &LayerRC, g: &mut Network, map: &mut NodeMap) {
    for seg in net.route {
        let wire_length_dbu = seg.length();
        let valid = |layer: i32| layer >= net.min_routing_layer || seg.is_via();
        let n1 = valid(seg.init_layer).then(|| ensure_point_node(map, seg.init_x, seg.init_y, seg.init_layer));
        let n2 = valid(seg.final_layer).then(|| ensure_point_node(map, seg.final_x, seg.final_y, seg.final_layer));
        let (Some(n1), Some(n2)) = (n1, n2) else { continue };
        let (mut res, mut cap) = (0.0f32, 0.0f32);
        if wire_length_dbu == 0 {
            // A via: the cut layer above the LOWER of the two routing layers.
            res = rc.cut_layer_res_above(seg.init_layer.min(seg.final_layer));
        } else if seg.init_layer == seg.final_layer {
            (res, cap) = rc.layer_rc(wire_length_dbu, seg.init_layer, net.ndr_width.and_then(|m| m.get(&seg.init_layer).copied()));
        } else {
            g.warnings.push(format!("EST-0025: non wire or via route found on net {}", net.name));
        }
        g.incr_cap(&n1, (f64::from(cap) / 2.0) as f32);
        g.make_resistor(&n1, &n2, res);
        g.incr_cap(&n2, (f64::from(cap) / 2.0) as f32);
    }
}

/// `makePartialParasiticsToPin` / `makeParasiticsToPin`: the wire from the pin to the grid node it
/// attaches to.
///
/// ⛔ The layer tried FIRST is the net's minimum plus one on a planar route, the PIN's connection
/// layer plus one on a routed one ([`PinAttach`]). Finding a node there means the pin is
/// reached through a via, whose cut resistance (the layer BELOW that one) is lumped into the same
/// resistor; otherwise the pin's own layer is used and there is no via.
/// ⚠️ The resistance is floored at 1e-3 so a pin sitting on its grid node still has one.
fn make_partial_parasitics_to_pin(net: &NetParasitics<'_>, pin: &PinGridLocation, rc: &LayerRC, g: &mut Network, map: &mut NodeMap) {
    let pin_node = NodeId::Pin(pin.name.clone());
    let mut layer = match net.attach {
        PinAttach::Planar => net.net_min_layer + 1,
        PinAttach::Routed => pin.conn_layer + 1,
    };
    let mut via_res = 0.0f32;
    let mut grid_node = map.get(&(pin.grid_pt.0, pin.grid_pt.1, layer)).cloned();
    if grid_node.is_none() {
        layer -= 1;
        grid_node = map.get(&(pin.grid_pt.0, pin.grid_pt.1, layer)).cloned();
    } else {
        // The cut layer BELOW this routing layer — the one the via came up through.
        via_res = rc.cut_layer_res_above(layer - 1);
    }
    let Some(grid_node) = grid_node else {
        g.warnings.push(format!("EST-0350: missing route to pin {}", pin.name));
        return;
    };
    let wire_length_dbu = (pin.pt.0 - pin.grid_pt.0).abs() + (pin.pt.1 - pin.grid_pt.1).abs();
    let (res, cap) = rc.layer_rc(wire_length_dbu, layer, net.ndr_width.and_then(|m| m.get(&layer).copied()));
    g.incr_cap(&pin_node, (f64::from(cap) / 2.0) as f32);
    let pin_grid_r = (res + via_res).max(1.0e-3);
    g.make_resistor(&pin_node, &grid_node, pin_grid_r);
    g.incr_cap(&grid_node, (f64::from(cap) / 2.0) as f32);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rc() -> LayerRC {
        // met1 and met2 at 1000 dbu/µm: the table carries met1, met2 has none (the fallback path).
        LayerRC {
            table_res: BTreeMap::from([(1, 8.929e5)]),
            table_cap: BTreeMap::from([(1, 1.7e-10)]),
            width: BTreeMap::from([(1, 140), (2, 140)]),
            resistance: BTreeMap::from([(1, 0.125), (2, 0.125)]),
            capacitance: BTreeMap::from([(2, 1.0e-4)]),
            edge_capacitance: BTreeMap::from([(2, 1.0e-5)]),
            cut_table_res: BTreeMap::from([(1, 4.5)]),
            cut_resistance: BTreeMap::from([(1, 9.0), (2, 3.0)]),
            dbu_per_micron: 1000,
        }
    }

    // The table wins; the technology's values are the fallback (`r_per_meter == 0`).
    #[test]
    fn layer_rc_prefers_the_table() {
        let rc = rc();
        let (r, c) = rc.layer_rc(1000, 1, None); // 1 µm of met1
        let len = f64::from(1e-6f32); // ⛔ the length narrows to a float first
        assert_eq!((r, c), ((8.929e5 * len) as f32, (1.7e-10 * len) as f32));
        let (r2, c2) = rc.layer_rc(1000, 2, None);
        // 0.125 Ω/sq over a 0.14 µm width, one micron of it.
        assert!((f64::from(r2) - 0.125 / 0.14).abs() < 1e-4, "{r2}");
        // (width * cap + 2 * edge) pF/µm, one micron.
        assert!((f64::from(c2) - (0.14 * 1.0e-4 + 2.0 * 1.0e-5) * 1e-12).abs() < 1e-20, "{c2}");
    }

    // ⚠️ A wider non-default rule lowers the resistance by the width ratio.
    #[test]
    fn a_non_default_rule_width_scales_the_resistance() {
        let rc = rc();
        let (r, _) = rc.layer_rc(1000, 1, None);
        let (r_ndr, _) = rc.layer_rc(1000, 1, Some(280));
        assert!((f64::from(r) / 2.0 - f64::from(r_ndr)).abs() < 1e-9, "{r} {r_ndr}");
    }

    fn seg(x0: i32, y0: i32, l0: i32, x1: i32, y1: i32, l1: i32) -> Segment {
        Segment { init_x: x0, init_y: y0, init_layer: l0, final_x: x1, final_y: y1, final_layer: l1 }
    }

    // A wire's capacitance is split half to each end; its resistor joins the two point nodes.
    #[test]
    fn a_wire_segment_splits_its_capacitance() {
        let rc = rc();
        let route = [seg(0, 0, 1, 1000, 0, 1)];
        let net = NetParasitics { name: "n", route: &route, pins: &[], net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        let (r, c) = rc.layer_rc(1000, 1, None);
        assert_eq!(g.nodes, vec![(NodeId::Point(0), c / 2.0), (NodeId::Point(1), c / 2.0)]);
        assert_eq!(g.resistors, vec![(NodeId::Point(0), NodeId::Point(1), r)]);
    }

    // A via is a zero-length segment: the cut layer above the LOWER layer, and no capacitance.
    #[test]
    fn a_via_takes_the_cut_layer_resistance() {
        let rc = rc();
        let route = [seg(0, 0, 1, 0, 0, 2)];
        let net = NetParasitics { name: "n", route: &route, pins: &[], net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        assert_eq!(g.resistors, vec![(NodeId::Point(0), NodeId::Point(1), 4.5)]); // the table's, not 9.0
        assert!(g.nodes.iter().all(|(_, c)| *c == 0.0));
    }

    // ⛔ An end below the minimum routing layer has no node, and the segment is skipped whole.
    #[test]
    fn a_segment_below_the_minimum_routing_layer_is_skipped() {
        let rc = rc();
        let route = [seg(0, 0, 1, 1000, 0, 1)];
        let net = NetParasitics { name: "n", route: &route, pins: &[], net_min_layer: 2, min_routing_layer: 2, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        assert!(g.nodes.is_empty() && g.resistors.is_empty());
    }

    // The pin attaches at net_min_layer + 1 when a node is there — through a via, whose cut
    // resistance is lumped into the same resistor.
    #[test]
    fn a_pin_attaches_through_the_via_layer_when_one_is_routed() {
        let rc = rc();
        let route = [seg(0, 0, 2, 1000, 0, 2)];
        let pins = [PinGridLocation { name: "i/A".into(), is_port: false, is_driver: false, pt: (0, 500), grid_pt: (0, 0), conn_layer: 1 }];
        let net = NetParasitics { name: "n", route: &route, pins: &pins, net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        let (r, c) = rc.layer_rc(500, 2, None);
        assert_eq!(g.resistors.last(), Some(&(NodeId::Pin("i/A".into()), NodeId::Point(0), r + 4.5)));
        assert_eq!(g.nodes.iter().find(|(n, _)| *n == NodeId::Pin("i/A".into())).map(|(_, c)| *c), Some(c / 2.0));
    }

    // ⛔ On a ROUTED route the pin attaches from its OWN connection layer, not the net's minimum.
    #[test]
    fn a_routed_pin_attaches_from_its_connection_layer() {
        let rc = rc();
        let route = [seg(0, 0, 2, 1000, 0, 2)];
        let pins = [PinGridLocation { name: "i/A".into(), is_port: false, is_driver: false, pt: (0, 500), grid_pt: (0, 0), conn_layer: 1 }];
        // Planar: net_min_layer 1 → tries layer 2, finds the node, so the via resistance is added.
        let planar = NetParasitics { name: "n", route: &route, pins: &pins, net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        // Routed: conn_layer 1 → tries layer 2 as well here, so the two agree on this net.
        let routed = NetParasitics { attach: PinAttach::Routed, ..planar.clone() };
        assert_eq!(estimate_net(&planar, &rc), estimate_net(&routed, &rc));
        // With the net's minimum a layer lower, only the planar one falls back to layer 1.
        let pins2 = [PinGridLocation { conn_layer: 2, ..pins[0].clone() }];
        let planar2 = NetParasitics { pins: &pins2, net_min_layer: 0, ..planar.clone() };
        let routed2 = NetParasitics { attach: PinAttach::Routed, ..planar2.clone() };
        assert_ne!(estimate_net(&planar2, &rc), estimate_net(&routed2, &rc));
    }

    // With nothing routed on that layer the pin's own layer is used, and there is no via.
    #[test]
    fn a_pin_attaches_on_its_own_layer_without_a_via() {
        let rc = rc();
        let route = [seg(0, 0, 1, 1000, 0, 1)];
        let pins = [PinGridLocation { name: "i/A".into(), is_port: false, is_driver: false, pt: (0, 500), grid_pt: (0, 0), conn_layer: 1 }];
        let net = NetParasitics { name: "n", route: &route, pins: &pins, net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        let (r, _) = rc.layer_rc(500, 1, None);
        assert_eq!(g.resistors.last(), Some(&(NodeId::Pin("i/A".into()), NodeId::Point(0), r)));
    }

    // ⚠️ A pin on its own grid node still gets a resistor: the floor.
    #[test]
    fn a_zero_length_attachment_is_floored() {
        let rc = rc();
        let route = [seg(0, 0, 1, 1000, 0, 1)];
        let pins = [PinGridLocation { name: "i/A".into(), is_port: false, is_driver: false, pt: (0, 0), grid_pt: (0, 0), conn_layer: 1 }];
        let net = NetParasitics { name: "n", route: &route, pins: &pins, net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        assert_eq!(g.resistors.last(), Some(&(NodeId::Pin("i/A".into()), NodeId::Point(0), 1.0e-3)));
    }

    // A pin whose grid point no segment reached is warned about and left unconnected.
    #[test]
    fn a_pin_with_no_grid_node_is_warned_about() {
        let rc = rc();
        let route = [seg(0, 0, 1, 1000, 0, 1)];
        let pins = [PinGridLocation { name: "i/A".into(), is_port: false, is_driver: false, pt: (9000, 9000), grid_pt: (9000, 9000), conn_layer: 1 }];
        let net = NetParasitics { name: "n", route: &route, pins: &pins, net_min_layer: 1, min_routing_layer: 1, ndr_width: None, attach: PinAttach::Planar };
        let g = estimate_net(&net, &rc);
        assert!(g.warnings.iter().any(|w| w.contains("EST-0350")), "{:?}", g.warnings);
        assert!(!g.nodes.iter().any(|(n, _)| *n == NodeId::Pin("i/A".into())));
    }
}
