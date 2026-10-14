// SPDX-License-Identifier: Apache-2.0
//! `sta::SpefWriter` (dbSta/SpefWriter.cc) — the SPEF `estimate_parasitics -spef_file` writes: a
//! header with the timer's units, the top-level ports, then per net `*D_NET`, `*CONN`, `*CAP`,
//! `*RES`, in the network's storage order ([`crate::network::Parasitic`]).
//!
//! ⚠️ Values are `float`s printed by `std::ostream`'s defaults — `%g`, six significant digits —
//! after dividing by the unit's `float` scale in `float`. Names: only `$` is escaped; an instance
//! pin's LAST `/` becomes `:` (`fixPinDelimiter`), a port keeps its name.
//!
//! ⬜ This writer belongs beside `loom`'s SPEF reader once another engine needs it (grt's
//! `write_spef` step); it lives here until then.

#![cfg(feature = "odb")]

use vyges_opendb::Db;

use crate::network::{Node, Parasitic};
use crate::rc::fmt_g;

/// The timer's units as `Unit::scale()` (float).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpefUnits {
    pub time: f32,
    pub capacitance: f32,
    pub resistance: f32,
}

fn escape_special(s: &str) -> String {
    s.replace('$', "\\$")
}

fn fix_pin_delimiter(s: &str) -> String {
    match s.rfind('/') {
        Some(i) => format!("{}:{}", &s[..i], &s[i + 1..]),
        None => s.to_string(),
    }
}

/// `getIoDirectionText`: `I`, `O`, else `B`.
fn io_text(io: &str) -> char {
    match io {
        "INPUT" => 'I',
        "OUTPUT" => 'O',
        _ => 'B',
    }
}

/// `Unit::scaleAbbrevSuffix`, upper-cased: the scale's SI prefix, then the unit's suffix.
fn abbrev(scale: f32, suffix: &str) -> String {
    let prefixes = [(1e-15f32, "f"), (1e-12, "p"), (1e-9, "n"), (1e-6, "u"), (1e-3, "m"), (1.0, ""), (1e3, "k"), (1e6, "M")];
    let p = prefixes.iter().find(|(s, _)| (*s - scale).abs() <= s.abs() * 1e-6).map_or("", |(_, p)| p);
    format!("{p}{suffix}").to_uppercase()
}

/// `SpefWriter`'s constructor: the header, then `*PORTS` (the block's terminals, in order).
pub fn header(db: &Db, units: SpefUnits) -> String {
    let mut t = String::new();
    t.push_str("*SPEF \"ieee 1481-1999\"\n");
    t.push_str(&format!("*DESIGN \"{}\"\n", escape_special(&db.block_name())));
    t.push_str("*DATE \"11:11:11 Fri 11 11, 1111\"\n*VENDOR \"The OpenROAD Project\"\n*PROGRAM \"OpenROAD\"\n*VERSION \"1.0\"\n");
    t.push_str("*DESIGN_FLOW \"NAME_SCOPE LOCAL\" \"PIN_CAP NONE\"\n*DIVIDER /\n*DELIMITER :\n*BUS_DELIMITER []\n");
    t.push_str(&format!("*T_UNIT 1 {}\n*C_UNIT 1 {}\n*R_UNIT 1 {}\n*L_UNIT 1 HENRY\n\n", abbrev(units.time, "s"), abbrev(units.capacitance, "F"), abbrev(units.resistance, "ohm")));
    t.push_str("*PORTS\n");
    for bt in db.block_get_b_terms() {
        t.push_str(&format!("{} {}\n", escape_special(&bt), io_text(&db.bterm_get_io_type(&bt))));
    }
    t.push('\n');
    t
}

/// `SpefWriter::writeNet`.
pub fn write_net(net: &str, g: &Parasitic, units: SpefUnits) -> String {
    let g6 = |v: f32| fmt_g(f64::from(v), 6);
    let mut t = format!("*D_NET {} {}\n*CONN\n", escape_special(net), g6(g.capacitance() / units.capacitance));
    for p in g.pin_nodes.values() {
        if p.is_port {
            t.push_str(&format!("*P {} {}\n", escape_special(&p.name), io_text(&p.io_type)));
        } else {
            t.push_str(&format!("*I {} {} *D {}\n", escape_special(&fix_pin_delimiter(&p.name)), io_text(&p.io_type), p.master));
        }
    }
    // `*CAP` walks `nodes()` — the pin nodes (by key), then the Steiner nodes (by id) — and writes
    // every node with a ground capacitance, PIN NODES INCLUDED: global-routing parasitics put the
    // pin-to-grid segment's cap on the pin node (upstream #11482). A placement estimate never
    // does, so its files are unchanged.
    let mut count = 1;
    let mut label = false;
    let pins = g.pin_nodes.values().map(|p| (escape_special(&if p.is_port { p.name.clone() } else { fix_pin_delimiter(&p.name) }), p.cap));
    let subs = g.sub_nodes.iter().map(|(id, cap)| (escape_special(&format!("{net}:{id}")), *cap));
    for (name, cap) in pins.chain(subs) {
        if cap == 0.0 {
            continue;
        }
        if !label {
            label = true;
            t.push_str("*CAP\n");
        }
        t.push_str(&format!("{count} {name} {}\n", g6(cap / units.capacitance)));
        count += 1;
    }
    let name = |n: &Node| match n {
        Node::Sub(id) => escape_special(&format!("{net}:{id}")),
        Node::Pin(k) => {
            let p = &g.pin_nodes[k];
            escape_special(&if p.is_port { p.name.clone() } else { fix_pin_delimiter(&p.name) })
        }
    };
    for (k, (a, b, r)) in g.resistors.iter().enumerate() {
        if k == 0 {
            t.push_str("*RES\n");
        }
        t.push_str(&format!("{} {} {} {}\n", k + 1, name(a), name(b), g6(*r / units.resistance)));
    }
    t.push_str("*END\n\n");
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PinNode;

    /// Rule (`SpefWriter::writeNet`, upstream #11482): `*CAP` walks `nodes()` — pin nodes by key,
    /// then Steiner nodes by id — and writes each one with a non-zero ground cap, pins included;
    /// one count runs through both.
    #[test]
    fn pin_node_caps_come_first_and_share_the_count() {
        let units = SpefUnits { time: 1e-9, capacitance: 1e-15, resistance: 1e3 };
        let pin = |name: &str, is_port: bool, cap: f32| PinNode { name: name.into(), is_port, io_type: "INPUT".into(), master: if is_port { String::new() } else { "BUF_X4".into() }, cap };
        let mut g = Parasitic::default();
        g.pin_nodes.insert(1, pin("clk", true, 0.5e-15));
        g.pin_nodes.insert(4, pin("u1/A", false, 0.0));
        g.pin_nodes.insert(6, pin("u2/A", false, 0.25e-15));
        g.sub_nodes.insert(1, 2.0e-15);
        g.sub_nodes.insert(2, 0.0);
        g.resistors.push((Node::Pin(1), Node::Sub(1), 10.0));
        let t = write_net("clk", &g, units);
        let cap: Vec<&str> = t.lines().skip_while(|l| *l != "*CAP").skip(1).take_while(|l| !l.starts_with('*')).collect();
        assert_eq!(cap, ["1 clk 0.5", "2 u2:A 0.25", "3 clk:1 2"]);
        assert!(t.starts_with("*D_NET clk 2.75\n"), "{t}");
    }
}
