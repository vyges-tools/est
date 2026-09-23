// SPDX-License-Identifier: Apache-2.0
//! What global routing asks of a Liberty library: per cell, whether it is a pad, which of its
//! ports are REGISTER CLOCKS (`isClkTerm`, the leaf-clock test), and every arc's ROLE (the clock
//! network's search passes only combinational arcs — `clk_network`).
//!
//! Each timing group's role is `LibertyBuilder::makeTimingArcs`':
//!
//! 1. the preamble — a group with no `timing_type` into a pin whose function is exactly ONE
//!    sequential output port is re-typed: `rising_edge` when the FROM port is in that sequential's
//!    clock with a unate sense (either sense), `clear` / `preset` when it is in those;
//! 2. combinational → latch D-to-Q when the FROM port is that latch's data, else combinational;
//! 3. `rising_edge` / `falling_edge` → `makeRegLatchArcs`: the first sequential (among the outputs
//!    the TO pin's function names) whose clock names the FROM port gives clock-to-Q (a register) or
//!    enable-to-Q (a latch); a latch whose data names it gives D-to-Q; a clear or preset naming it
//!    gives set/clear; none deciding → inferred clock-to-Q;
//! 4. clear / preset → set/clear; the tristate types keep their roles; checks are neither.
//!
//! A port is a register clock (`isRegClk`) when it is the FROM port of a clock-to-Q or enable-to-Q
//! arc (`makeTimingArcPortMaps`).
//!
//! ⛔ Step 3's ports are a `std::set<LibertyPort*>` — POINTER order. When a function names outputs
//! of two sequentials that decide differently the reference's answer is allocation order, so that
//! is refused rather than guessed. ⛔ `inferLatchRoles` is not transcribed: a cell it may rewrite
//! is flagged, and the clock search refuses to pass through it.
//!
//! ⚠️ Refused, not modelled: bus and bundle pins, `ff_bank` / `latch_bank` (a sequential per bit),
//! an unknown `timing_type`.

use std::collections::{BTreeMap, BTreeSet};

/// What the liberty lookup says about one instance terminal on a net.
///
/// ⚠️ These are **inputs**, not something this crate derives: they come from the timing library,
/// which is a different substrate entirely.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ITermClockFacts {
    /// Whether the terminal resolves to a liberty port at all.
    pub has_liberty_port: bool,
    /// Whether that port is a register's clock input.
    pub is_reg_clk: bool,
    /// Whether the cell it belongs to is a pad.
    pub cell_is_pad: bool,
}

/// Whether a terminal counts as a clock terminal.
///
/// ⚠️ **A pad terminal counts even when it is not a register clock pin.** The two conditions are
/// an OR, and both are gated on the port existing at all — a terminal with no liberty port is
/// never a clock terminal, whatever its cell.
pub fn is_clk_term(f: ITermClockFacts) -> bool {
    f.has_liberty_port && (f.is_reg_clk || f.cell_is_pad)
}

/// Whether a net is a clock net **above the leaves**.
///
/// ⛔ **"Clock net" is not the same as "clock-typed net".** A net typed as clock that reaches any
/// clock terminal is a LEAF and answers false; only one that reaches none of them is a non-leaf
/// clock. Getting this backwards puts the leaf nets at the front of the routing order.
///
/// ⚠️ A net that is not clock-typed answers false without looking at its terminals.
pub fn is_non_leaf_clock(sig_type_is_clock: bool, iterms: &[ITermClockFacts]) -> bool {
    sig_type_is_clock && !iterms.iter().copied().any(is_clk_term)
}

/// A Liberty group: `kind (args) { attrs; groups }`. Complex attributes (`values (…);`) are dropped.
#[derive(Debug, Default)]
struct Group {
    kind: String,
    args: Vec<String>,
    attrs: Vec<(String, String)>,
    /// `name (a, b);` — a complex attribute, kept for `capacitive_load_unit`.
    complex: Vec<(String, Vec<String>)>,
    groups: Vec<Group>,
}

impl Group {
    fn attr(&self, key: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
    fn complex_attr(&self, key: &str) -> Option<&[String]> {
        self.complex.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_slice())
    }
    fn children<'a>(&'a self, kind: &'a str) -> impl Iterator<Item = &'a Group> + 'a {
        self.groups.iter().filter(move |g| g.kind == kind)
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Word(String),
    Punct(u8),
}

fn lex(text: &str) -> Vec<Tok> {
    let b = text.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_whitespace() {
            i += 1;
        } else if c == b'\\' {
            i += 1; // a line continuation
        } else if c == b'/' && b.get(i + 1) == Some(&b'*') {
            i = text[i + 2..].find("*/").map_or(b.len(), |k| i + 2 + k + 2);
        } else if c == b'"' {
            let end = text[i + 1..].find('"').map_or(b.len(), |k| i + 1 + k);
            out.push(Tok::Word(text[i + 1..end].replace("\\\n", "").replace('\\', "")));
            i = end + 1;
        } else if b"(){}:;,".contains(&c) {
            out.push(Tok::Punct(c));
            i += 1;
        } else {
            let start = i;
            while i < b.len() && !b[i].is_ascii_whitespace() && !b"(){}:;,\"\\".contains(&b[i]) {
                i += 1;
            }
            out.push(Tok::Word(text[start..i].to_string()));
        }
    }
    out
}

fn parse_body(t: &[Tok], mut i: usize, g: &mut Group) -> Result<usize, String> {
    while i < t.len() {
        let name = match &t[i] {
            Tok::Punct(b'}') => return Ok(i + 1),
            Tok::Punct(b';') => {
                i += 1;
                continue;
            }
            Tok::Word(w) => w.clone(),
            Tok::Punct(p) => return Err(format!("liberty: unexpected '{}'", *p as char)),
        };
        i += 1;
        match t.get(i) {
            Some(Tok::Punct(b':')) => {
                // ONE value, the `;` optional (LibertyParse: a simple attribute ends at its value —
                // `area : 0.2` with no `;` is followed directly by the next statement); only a
                // voltage expression (`VDD + 0.1`) continues through its operators.
                i += 1;
                let mut v = Vec::new();
                if let Some(Tok::Word(w)) = t.get(i) {
                    v.push(w.clone());
                    i += 1;
                    while let (Some(Tok::Word(op)), Some(Tok::Word(w))) = (t.get(i), t.get(i + 1)) {
                        if !["+", "-", "*", "/"].contains(&op.as_str()) {
                            break;
                        }
                        v.push(op.clone());
                        v.push(w.clone());
                        i += 2;
                    }
                }
                g.attrs.push((name, v.join(" ")));
            }
            Some(Tok::Punct(b'(')) => {
                i += 1;
                let mut args = Vec::new();
                let mut depth = 1;
                while i < t.len() && depth > 0 {
                    match &t[i] {
                        Tok::Punct(b'(') => depth += 1,
                        Tok::Punct(b')') => depth -= 1,
                        Tok::Word(w) if depth == 1 => args.push(w.clone()),
                        _ => {}
                    }
                    i += 1;
                }
                if t.get(i) == Some(&Tok::Punct(b'{')) {
                    let mut child = Group { kind: name, args, ..Group::default() };
                    i = parse_body(t, i + 1, &mut child)?;
                    g.groups.push(child);
                } else {
                    g.complex.push((name, args));
                }
            }
            _ => return Err(format!("liberty: '{name}' is neither an attribute nor a group")),
        }
    }
    Ok(i)
}

/// The identifiers an expression names (`"!(A & B_N)"` → `A`, `B_N`) — `FuncExpr::hasPort`.
fn idents(expr: &str) -> BTreeSet<String> {
    expr.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '[' || c == ']'))
        .filter(|w| !w.is_empty() && !w.chars().next().is_some_and(|c| c.is_ascii_digit()))
        .map(str::to_string)
        .collect()
}

/// A parsed liberty function (`FuncExpr`).
#[derive(Debug, Clone, PartialEq)]
enum Expr {
    Port(String),
    Zero,
    One,
    Not(Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    Xor(Box<Expr>, Box<Expr>),
}

/// `LibExprParse.yy`: `+ |` lowest, then `* &`, then `^`, then implicit AND of terminals, then
/// `!` (prefix) and `'` (postfix), which apply to a TERMINAL only. All left-associative.
fn parse_expr(text: &str) -> Result<Expr, String> {
    let mut toks = Vec::new();
    let b: Vec<char> = text.chars().collect();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_whitespace() {
            i += 1;
        } else if "+|*&^!'()".contains(c) {
            toks.push(c.to_string());
            i += 1;
        } else {
            let st = i;
            while i < b.len() && !b[i].is_whitespace() && !"+|*&^!'()".contains(b[i]) {
                i += 1;
            }
            toks.push(b[st..i].iter().collect());
        }
    }
    let mut pos = 0;
    let e = parse_binary(&toks, &mut pos, 0)?;
    if pos != toks.len() {
        return Err(format!("function {text:?}: trailing input"));
    }
    Ok(e)
}

fn parse_binary(t: &[String], pos: &mut usize, level: usize) -> Result<Expr, String> {
    const OPS: [&[&str]; 3] = [&["+", "|"], &["*", "&"], &["^"]];
    if level == OPS.len() {
        return parse_implicit_and(t, pos);
    }
    let mut left = parse_binary(t, pos, level + 1)?;
    while *pos < t.len() && OPS[level].contains(&t[*pos].as_str()) {
        *pos += 1;
        let right = parse_binary(t, pos, level + 1)?;
        left = match level {
            0 => Expr::Or(Box::new(left), Box::new(right)),
            1 => Expr::And(Box::new(left), Box::new(right)),
            _ => Expr::Xor(Box::new(left), Box::new(right)),
        };
    }
    Ok(left)
}

fn starts_terminal(t: &[String], pos: usize) -> bool {
    t.get(pos).is_some_and(|s| !["+", "|", "*", "&", "^", "'", ")"].contains(&s.as_str()))
}

fn parse_implicit_and(t: &[String], pos: &mut usize) -> Result<Expr, String> {
    let mut left = parse_terminal_expr(t, pos)?;
    while starts_terminal(t, *pos) {
        let right = parse_terminal_expr(t, pos)?;
        left = Expr::And(Box::new(left), Box::new(right));
    }
    Ok(left)
}

fn parse_terminal_expr(t: &[String], pos: &mut usize) -> Result<Expr, String> {
    if t.get(*pos).map(String::as_str) == Some("!") {
        *pos += 1;
        return Ok(Expr::Not(Box::new(parse_terminal(t, pos)?)));
    }
    let e = parse_terminal(t, pos)?;
    if t.get(*pos).map(String::as_str) == Some("'") {
        *pos += 1;
        return Ok(Expr::Not(Box::new(e)));
    }
    Ok(e)
}

fn parse_terminal(t: &[String], pos: &mut usize) -> Result<Expr, String> {
    let tok = t.get(*pos).ok_or("function: unexpected end")?.clone();
    *pos += 1;
    Ok(match tok.as_str() {
        "(" => {
            let e = parse_binary(t, pos, 0)?;
            if t.get(*pos).map(String::as_str) != Some(")") {
                return Err("function: unbalanced '('".into());
            }
            *pos += 1;
            e
        }
        "0" => Expr::Zero,
        "1" => Expr::One,
        _ => Expr::Port(tok),
    })
}

/// `TimingSense`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sense {
    Positive,
    Negative,
    NonUnate,
    None,
    Unknown,
}

/// `FuncExpr::portTimingSense` — transcribed with its asymmetric AND/OR combination.
fn port_timing_sense(e: &Expr, port: &str) -> Sense {
    use Sense::*;
    match e {
        Expr::Port(p) => {
            if p == port {
                Positive
            } else {
                None
            }
        }
        Expr::Not(l) => match port_timing_sense(l, port) {
            Positive => Negative,
            Negative => Positive,
            s => s,
        },
        Expr::And(l, r) | Expr::Or(l, r) => {
            let (ls, rs) = (port_timing_sense(l, port), port_timing_sense(r, port));
            if ls == rs {
                ls
            } else if ls == NonUnate || rs == NonUnate || (ls == Positive && rs == Negative) || (ls == Negative && rs == Positive) {
                NonUnate
            } else if ls == None || ls == Unknown {
                rs
            } else if rs == None || rs == Unknown {
                ls
            } else {
                Unknown
            }
        }
        Expr::Xor(l, r) => {
            let (ls, rs) = (port_timing_sense(l, port), port_timing_sense(r, port));
            if matches!(ls, Positive | Negative | NonUnate) || matches!(rs, Positive | Negative | NonUnate) {
                NonUnate
            } else {
                Unknown
            }
        }
        Expr::Zero | Expr::One => None,
    }
}

/// One `ff` / `latch` group.
#[derive(Debug)]
struct Seq {
    is_register: bool,
    clock: Option<Expr>,
    data: BTreeSet<String>,
    clear: BTreeSet<String>,
    preset: BTreeSet<String>,
}

impl Seq {
    fn clock_has(&self, port: &str) -> bool {
        self.clock.as_ref().is_some_and(|c| expr_has(c, port))
    }
}

fn expr_has(e: &Expr, port: &str) -> bool {
    match e {
        Expr::Port(p) => p == port,
        Expr::Not(l) => expr_has(l, port),
        Expr::And(l, r) | Expr::Or(l, r) | Expr::Xor(l, r) => expr_has(l, port) || expr_has(r, port),
        Expr::Zero | Expr::One => false,
    }
}

/// `TimingRole` of an arc set, as far as the clock network and the register clocks ask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Combinational,
    RegClkToQ,
    LatchEnToQ,
    LatchDtoQ,
    RegSetClr,
    TristateEnable,
    TristateDisable,
    /// A timing check, or any role neither reader distinguishes.
    Other,
}

/// One arc set: `related_pin` → the pin, with its role.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CellArc {
    pub from: String,
    pub to: String,
    pub role: Role,
}

/// A cell's facts.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CellClock {
    /// `is_pad` or `pad_cell` true.
    pub is_pad: bool,
    /// Every port, and whether it is a register clock.
    pub ports: BTreeMap<String, bool>,
    /// Ports whose `function` is a constant (`"0"`, `"1"`): a tie cell's output, which the timer's
    /// logic simulation makes a CONSTANT pin (`Sta::isConstant`).
    pub constant_outputs: BTreeSet<String>,
    /// Every arc set, in the library's order.
    pub arcs: Vec<CellArc>,
    /// ⛔ `inferLatchRoles` may rewrite a combinational arc of this cell to latch D-to-Q: the cell
    /// has an inferred clock-to-Q arc, and a combinational arc into a pin with a clock-to-Q arc.
    /// Not transcribed — a clock search through such a cell is refused.
    pub latch_roles_may_be_inferred: bool,
}

/// `makeRegLatchArcs`' role (see the module). `Err` when pointer order would decide; the bool is
/// `setHasInferedRegTimingArcs`.
fn make_reg_latch_role(function: Option<&Expr>, from: &str, seq_of: &BTreeMap<String, usize>, seqs: &[Seq]) -> Result<(Role, bool), ()> {
    let mut verdicts = BTreeMap::new();
    if let Some(f) = function {
        let mut ports = BTreeSet::new();
        collect_ports(f, &mut ports);
        for port in ports {
            let Some(&k) = seq_of.get(&port) else { continue };
            let s = &seqs[k];
            let v = if s.clock.as_ref().is_some_and(|c| expr_has(c, from)) {
                Some(if s.is_register { Role::RegClkToQ } else { Role::LatchEnToQ })
            } else if !s.is_register && s.data.contains(from) {
                Some(Role::LatchDtoQ)
            } else if s.clear.contains(from) || s.preset.contains(from) {
                Some(Role::RegSetClr)
            } else {
                None
            };
            if let Some(v) = v {
                verdicts.insert(k, v);
            }
        }
    }
    let mut vs = verdicts.values();
    match vs.next() {
        None => Ok((Role::RegClkToQ, true)),
        Some(&v) if vs.all(|&w| w == v) => Ok((v, false)),
        Some(_) => Err(()),
    }
}

fn collect_ports(e: &Expr, out: &mut BTreeSet<String>) {
    match e {
        Expr::Port(p) => {
            out.insert(p.clone());
        }
        Expr::Not(l) => collect_ports(l, out),
        Expr::And(l, r) | Expr::Or(l, r) | Expr::Xor(l, r) => {
            collect_ports(l, out);
            collect_ports(r, out);
        }
        Expr::Zero | Expr::One => {}
    }
}

/// `LibertyBuilder::makeTimingArcs`' role for one timing group. The preamble first: a group with
/// no `timing_type` (combinational) into a pin whose function is exactly ONE sequential port is
/// re-typed — rising_edge when the FROM port is in that sequential's clock with a unate sense
/// (either sense: both become rising_edge), clear or preset when it is in those.
fn arc_role(timing_type: &str, function: Option<&Expr>, from: &str, seq_of: &BTreeMap<String, usize>, seqs: &[Seq]) -> Result<(Role, bool), String> {
    let seq = match function {
        Some(Expr::Port(p)) => seq_of.get(p).map(|&k| &seqs[k]),
        _ => None,
    };
    let mut tt = timing_type;
    if tt == "combinational" {
        if let Some(s) = seq {
            if s.clock_has(from) {
                if matches!(port_timing_sense(s.clock.as_ref().expect("clock"), from), Sense::Positive | Sense::Negative) {
                    tt = "rising_edge";
                }
            } else if s.clear.contains(from) {
                tt = "clear";
            } else if s.preset.contains(from) {
                tt = "preset";
            }
        }
    }
    Ok(match tt {
        "combinational" => {
            if seq.is_some_and(|s| !s.is_register && s.data.contains(from)) {
                (Role::LatchDtoQ, false)
            } else {
                (Role::Combinational, false)
            }
        }
        "combinational_rise" | "combinational_fall" => (Role::Combinational, false),
        "rising_edge" | "falling_edge" => make_reg_latch_role(function, from, seq_of, seqs).map_err(|()| format!("pin {from}: two sequentials decide differently (pointer order)"))?,
        "preset" | "clear" => (Role::RegSetClr, false),
        "three_state_enable" | "three_state_enable_rise" | "three_state_enable_fall" => (Role::TristateEnable, false),
        "three_state_disable" | "three_state_disable_rise" | "three_state_disable_fall" => (Role::TristateDisable, false),
        t if ["setup_", "hold_", "recovery_", "removal_", "skew_", "non_seq_", "nochange_", "min_pulse_width", "minimum_period", "max_clock_tree_path", "min_clock_tree_path"]
            .iter()
            .any(|k| t.starts_with(k)) =>
        {
            (Role::Other, false)
        }
        t => return Err(format!("timing_type {t:?} is not modelled")),
    })
}

fn read_cell(cell: &Group) -> Result<CellClock, String> {
    let name = cell.args.first().cloned().unwrap_or_default();
    for kind in ["bus", "bundle", "ff_bank", "latch_bank"] {
        if cell.children(kind).next().is_some() {
            return Err(format!("liberty cell {name}: a {kind} group is not modelled"));
        }
    }
    let truthy = |k: &str| cell.attr(k).is_some_and(|v| v.eq_ignore_ascii_case("true"));
    let mut seqs = Vec::new();
    let mut seq_of = BTreeMap::new();
    // makeSequentials: ff groups, then latch groups; a later output mapping overwrites.
    for (kind, is_register, clk, data) in [("ff", true, "clocked_on", "next_state"), ("latch", false, "enable", "data_in")] {
        for g in cell.children(kind) {
            let set = |k: &str| g.attr(k).map(idents).unwrap_or_default();
            let clock = g.attr(clk).map(parse_expr).transpose().map_err(|e| format!("liberty cell {name}: {e}"))?;
            seqs.push(Seq { is_register, clock, data: set(data), clear: set("clear"), preset: set("preset") });
            for out in &g.args {
                seq_of.insert(out.clone(), seqs.len() - 1);
            }
        }
    }
    let mut ports = BTreeMap::new();
    let mut constant_outputs = BTreeSet::new();
    for pin in cell.children("pin") {
        for p in &pin.args {
            ports.entry(p.clone()).or_insert(false);
            if let Some(f) = pin.attr("function") {
                if matches!(f.trim().trim_matches('"').trim(), "0" | "1") {
                    constant_outputs.insert(p.clone());
                }
            }
        }
    }
    let mut arcs = Vec::new();
    let mut has_inferred = false;
    for pin in cell.children("pin") {
        let function = pin.attr("function").map(parse_expr).transpose().map_err(|e| format!("liberty cell {name}: {e}"))?;
        for timing in pin.children("timing") {
            let tt = timing.attr("timing_type").unwrap_or("combinational");
            for from in timing.attr("related_pin").unwrap_or("").split_whitespace() {
                let (role, inferred) = arc_role(tt, function.as_ref(), from, &seq_of, &seqs).map_err(|e| format!("liberty cell {name}: {e}"))?;
                // makeTimingArcPortMaps: the FROM port of clock-to-Q or enable-to-Q.
                if matches!(role, Role::RegClkToQ | Role::LatchEnToQ) {
                    ports.insert(from.to_string(), true);
                }
                has_inferred |= inferred;
                for to in &pin.args {
                    arcs.push(CellArc { from: from.to_string(), to: to.clone(), role });
                }
            }
        }
    }
    // inferLatchRoles runs on a cell with ANY inferred arc, over every clock-to-Q arc: a combinational
    // arc into the same pin (unate, cond-matched — not checked here: refusing is the safe side).
    let latch_roles_may_be_inferred =
        has_inferred && arcs.iter().any(|a| a.role == Role::Combinational && arcs.iter().any(|q| q.role == Role::RegClkToQ && q.to == a.to));
    Ok(CellClock { is_pad: truthy("is_pad") || truthy("pad_cell"), ports, constant_outputs, arcs, latch_roles_may_be_inferred })
}

/// The timer's command units, as far as `set_layer_rc` converts through them.
///
/// ⛔ `Unit::scale_` is a `float`: 1e-6 enters as `9.99999997e-7`, and the layer resistances
/// `set_layer_rc` writes differ in their low bits if it is taken as a double.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Units {
    /// `pulling_resistance_unit` (default 1 ohm).
    pub resistance: f32,
    /// `distance_unit` (default 1 micron).
    pub distance: f32,
    /// `capacitive_load_unit (<scale>, ff|pf)` (default 1 pF).
    pub capacitance: f32,
}

/// `LibertyReader::readUnit`: `<1|10|100><scale char><suffix>`, the scale char one of k m u n p f.
/// ⚠️ An unknown multiplier, scale or suffix only WARNS: the multiplier or scale stays 1.
fn read_unit(value: Option<&str>, suffix: &str, default: f32) -> f32 {
    let Some(units) = value.filter(|v| !v.is_empty()) else { return default };
    let mult_end = units.find(|c: char| !c.is_ascii_digit());
    let (mult, scale_suffix) = match mult_end {
        Some(k) => (match &units[..k] { "1" => 1.0f32, "10" => 10.0, "100" => 100.0, _ => 1.0 }, &units[k..]),
        None => (1.0, units),
    };
    let mut scale_mult = 1.0f32;
    if scale_suffix.len() == suffix.len() + 1 && scale_suffix[1..].eq_ignore_ascii_case(suffix) {
        scale_mult = match scale_suffix.as_bytes()[0].to_ascii_lowercase() {
            b'k' => 1e3,
            b'm' => 1e-3,
            b'u' => 1e-6,
            b'n' => 1e-9,
            b'p' => 1e-12,
            b'f' => 1e-15,
            _ => 1.0,
        };
    }
    scale_mult * mult
}

/// `capacitive_load_unit (<scale>, ff|pf)` — the scale times 1e-15 or 1e-12, in `float`.
/// ⚠️ Anything else only warns in the reference; the default (1 pF) stands.
fn read_cap_unit(values: Option<&[String]>) -> f32 {
    let default = 1e-12f32;
    let Some([scale, suffix]) = values else { return default };
    let Ok(scale) = scale.parse::<f32>() else { return default };
    match suffix.to_ascii_lowercase().as_str() {
        "ff" => scale * 1e-15,
        "pf" => scale * 1e-12,
        _ => default,
    }
}

/// The cells of every library read, by name. ⛔ A cell in two libraries resolves to the FIRST
/// library read (the network's cell lookup walks the libraries in read order).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LibertyClocks {
    pub cells: BTreeMap<String, CellClock>,
    /// ⛔ The FIRST library's units become the timer's (`Sta::readLiberty`); later ones do not.
    pub units: Option<Units>,
}

impl LibertyClocks {
    /// Read one library's text and add its cells.
    pub fn read(&mut self, text: &str) -> Result<(), String> {
        let toks = lex(text);
        let mut top = Group::default();
        parse_body(&toks, 0, &mut top)?;
        for lib in top.children("library") {
            if self.units.is_none() {
                self.units = Some(Units {
                    resistance: read_unit(lib.attr("pulling_resistance_unit"), "ohm", 1.0),
                    distance: read_unit(lib.attr("distance_unit"), "m", 1e-6),
                    capacitance: read_cap_unit(lib.complex_attr("capacitive_load_unit")),
                });
            }
            for cell in lib.children("cell") {
                let name = cell.args.first().cloned().unwrap_or_default();
                if !self.cells.contains_key(&name) {
                    let c = read_cell(cell)?;
                    self.cells.insert(name, c);
                }
            }
        }
        Ok(())
    }

    /// `isClkTerm`'s inputs for one instance terminal: whether it has a liberty port, whether that
    /// port is a register clock, and whether its cell is a pad.
    pub fn iterm_facts(&self, master: &str, mterm: &str) -> ITermClockFacts {
        let cell = self.cells.get(master);
        let port = cell.and_then(|c| c.ports.get(mterm));
        ITermClockFacts { has_liberty_port: port.is_some(), is_reg_clk: port.copied().unwrap_or(false), cell_is_pad: cell.is_some_and(|c| c.is_pad) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(body: &str) -> CellClock {
        let mut l = LibertyClocks::default();
        l.read(&format!("library (t) {{ cell (c) {{ {body} }} }}")).expect("parse");
        l.cells["c"].clone()
    }

    // makeRegLatchArcs: the edge arc's FROM port is named by the ff's clocked_on → clock-to-Q.
    #[test]
    fn a_flop_clock_is_a_register_clock() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
            pin (CLK) { direction : input; clock : true; }
            pin (D) { direction : input; }
            pin (RESET_B) { direction : input; }
            pin (Q) { direction : output; function : "IQ";
              timing () { related_pin : "CLK"; timing_type : rising_edge; cell_rise (t) { values ("1, 2"); } }
              timing () { related_pin : "RESET_B"; timing_type : clear; } }"#);
        assert_eq!(c.ports, BTreeMap::from([("CLK".into(), true), ("D".into(), false), ("Q".into(), false), ("RESET_B".into(), false)]));
    }

    // A latch's enable arc is enable-to-Q — also a register clock (`isRegClk` covers both roles).
    #[test]
    fn a_latch_enable_is_a_register_clock_and_its_data_is_not() {
        let c = cell(r#"latch ("IQ","IQ_N") { enable : "!GATE_N"; data_in : "D"; }
            pin (GATE_N) { direction : input; } pin (D) { direction : input; }
            pin (Q) { function : "IQ";
              timing () { related_pin : "GATE_N"; timing_type : falling_edge; }
              timing () { related_pin : "D"; timing_type : rising_edge; } }"#);
        assert_eq!((c.ports["GATE_N"], c.ports["D"]), (true, false));
    }

    // An edge arc whose FROM port is the sequential's clear is set/clear, not clock-to-Q.
    #[test]
    fn an_edge_arc_from_a_clear_pin_is_not_a_register_clock() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; clear : "!R"; }
            pin (R) { direction : input; }
            pin (Q) { function : "IQ"; timing () { related_pin : "R"; timing_type : rising_edge; } }"#);
        assert!(!c.ports["R"]);
    }

    // No function, or a function naming no sequential (a statetable cell): inferred clock-to-Q.
    #[test]
    fn an_edge_arc_with_no_sequential_is_inferred_clock_to_q() {
        let c = cell(r#"statetable ("CLK GATE","M0") { table : "L L : - : L"; }
            pin (GCLK) { function : "(CLK & M0)"; timing () { related_pin : "CLK"; timing_type : rising_edge; } }
            pin (CLK) { direction : input; } pin (GATE) { direction : input; }"#);
        assert_eq!((c.ports["CLK"], c.ports["GATE"]), (true, false));
    }

    // Only rising_edge / falling_edge arcs make roles here: a combinational or check arc does not.
    #[test]
    fn combinational_and_check_arcs_make_no_register_clock() {
        let c = cell(r#"pin (A) { direction : input; timing () { related_pin : "CLK"; timing_type : setup_rising; } }
            pin (Y) { function : "!A"; timing () { related_pin : "A"; timing_sense : negative_unate; } }
            pin (CLK) { clock : true; }"#);
        assert!(c.ports.values().all(|&v| !v), "clock : true alone does not make a register clock");
    }

    // A test_cell's pins are a separate cell to the timer.
    #[test]
    fn a_test_cell_is_not_read_as_the_cell() {
        let c = cell(r#"pin (Q) { function : "Y"; }
            test_cell () { pin (TCK) { timing () { related_pin : "TCK"; timing_type : rising_edge; } } }"#);
        assert!(!c.ports.contains_key("TCK"));
    }

    // makeTimingArcs' preamble: a timing group with no timing_type into a pin whose function is ONE
    // sequential port, from that sequential's unate clock, becomes rising_edge → clock-to-Q.
    #[test]
    fn a_combinational_group_from_a_unate_clock_is_promoted_to_clock_to_q() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "!CLK_N"; next_state : "D"; }
            pin (CLK_N) { } pin (D) { }
            pin (Q) { function : "IQ"; timing () { related_pin : "CLK_N"; } }"#);
        assert!(c.ports["CLK_N"]);
        assert_eq!(c.arcs[0].role, Role::RegClkToQ);
    }

    // A non-unate clock (XOR) is not promoted: the group stays combinational.
    #[test]
    fn a_non_unate_clock_stays_combinational() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK ^ EN"; next_state : "D"; }
            pin (CLK) { } pin (EN) { }
            pin (Q) { function : "IQ"; timing () { related_pin : "CLK"; } }"#);
        assert!(!c.ports["CLK"]);
        assert_eq!(c.arcs[0].role, Role::Combinational);
    }

    // The preamble needs the function to be exactly one port: "IQ & A" is not promoted.
    #[test]
    fn the_preamble_needs_a_single_port_function() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; }
            pin (CLK) { } pin (A) { }
            pin (Q) { function : "IQ & A"; timing () { related_pin : "CLK"; } }"#);
        assert_eq!(c.arcs[0].role, Role::Combinational);
    }

    // Combinational from a clear pin is re-typed clear → set/clear; from a latch's data → D-to-Q.
    #[test]
    fn combinational_groups_from_clear_and_latch_data_take_those_roles() {
        let c = cell(r#"ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; clear : "!R"; }
            pin (R) { } pin (Q) { function : "IQ"; timing () { related_pin : "R"; } }"#);
        assert_eq!(c.arcs[0].role, Role::RegSetClr);
        let c = cell(r#"latch ("IQ","IQ_N") { enable : "G"; data_in : "D"; }
            pin (D) { } pin (Q) { function : "IQ"; timing () { related_pin : "D"; timing_sense : positive_unate; } }"#);
        assert_eq!(c.arcs[0].role, Role::LatchDtoQ);
    }

    // Tristate groups keep their own roles; a check is neither.
    #[test]
    fn tristate_and_check_roles() {
        let c = cell(r#"pin (Z) { function : "A"; three_state : "!TE";
              timing () { related_pin : "A"; } timing () { related_pin : "TE"; timing_type : three_state_enable; }
              timing () { related_pin : "TE"; timing_type : three_state_disable; } }
            pin (A) { timing () { related_pin : "CLK"; timing_type : hold_rising; } } pin (TE) { } pin (CLK) { }"#);
        let roles: Vec<Role> = c.arcs.iter().map(|a| a.role).collect();
        assert_eq!(roles, [Role::Combinational, Role::TristateEnable, Role::TristateDisable, Role::Other]);
    }

    // LibExprParse.yy precedence: `!` binds a terminal, then implicit AND, then ^, then * &, then + |.
    #[test]
    fn the_function_grammar_follows_the_reference_precedence() {
        let p = |s: &str| parse_expr(s).expect("parse");
        let port = |n: &str| Box::new(Expr::Port(n.into()));
        assert_eq!(p("!A&B"), Expr::And(Box::new(Expr::Not(port("A"))), port("B")));
        assert_eq!(p("A+B C"), Expr::Or(port("A"), Box::new(Expr::And(port("B"), port("C")))));
        assert_eq!(p("A&B^C"), Expr::And(port("A"), Box::new(Expr::Xor(port("B"), port("C")))));
        assert_eq!(p("A'"), Expr::Not(port("A")));
        assert_eq!(p("(A|B)&C"), Expr::And(Box::new(Expr::Or(port("A"), port("B"))), port("C")));
    }

    // FuncExpr::portTimingSense, including its asymmetric AND/OR combination.
    #[test]
    fn port_timing_sense_follows_the_reference() {
        let s = |e: &str, p: &str| port_timing_sense(&parse_expr(e).expect("parse"), p);
        assert_eq!(s("A", "A"), Sense::Positive);
        assert_eq!(s("!A", "A"), Sense::Negative);
        assert_eq!(s("A & !A", "A"), Sense::NonUnate);
        assert_eq!(s("A & B", "A"), Sense::Positive);
        assert_eq!(s("A ^ B", "A"), Sense::NonUnate);
        assert_eq!(s("B ^ C", "A"), Sense::Unknown);
        assert_eq!(s("B & C", "A"), Sense::None);
        assert_eq!(s("1", "A"), Sense::None);
    }

    // readUnit: multiplier × scale, all float; unknowns keep 1; a missing attribute keeps the default.
    #[test]
    fn units_follow_read_unit() {
        assert_eq!(read_unit(Some("1kohm"), "ohm", 1.0), 1e3);
        assert_eq!(read_unit(Some("100ohm"), "ohm", 1.0), 100.0);
        assert_eq!(read_unit(Some("10mohm"), "ohm", 1.0), 10.0 * 1e-3f32);
        assert_eq!(read_unit(Some("1xohm"), "ohm", 1.0), 1.0);
        assert_eq!(read_unit(None, "m", 1e-6), 1e-6f32);
        let mut l = LibertyClocks::default();
        l.read(r#"library (a) { pulling_resistance_unit : "1kohm"; }"#).expect("parse");
        l.read(r#"library (b) { pulling_resistance_unit : "1ohm"; }"#).expect("parse");
        assert_eq!(l.units, Some(Units { resistance: 1e3, distance: 1e-6, capacitance: 1e-12 }), "the first library's units stand");
    }

    // A simple attribute without its `;` ends at its value: the next statement still parses.
    #[test]
    fn an_attribute_without_a_semicolon_ends_at_its_value() {
        let c = cell(r#"area : 0.2
            pin (A) { direction : input }
            pin (Y) { function : "!A"; timing () { related_pin : "A"; } }"#);
        assert_eq!(c.arcs.len(), 1);
        assert!(c.ports.contains_key("A"));
    }

    // capacitive_load_unit is a COMPLEX attribute: `(scale, ff|pf)`.
    #[test]
    fn the_capacitance_unit_comes_from_capacitive_load_unit() {
        let mut l = LibertyClocks::default();
        l.read(r#"library (a) { capacitive_load_unit (1,ff); }"#).expect("parse");
        assert_eq!(l.units.expect("units").capacitance, 1e-15);
        let mut l = LibertyClocks::default();
        l.read(r#"library (b) { capacitive_load_unit (10,pf); }"#).expect("parse");
        assert_eq!(l.units.expect("units").capacitance, 10.0 * 1e-12);
        let mut l = LibertyClocks::default();
        l.read("library (c) { }").expect("parse");
        assert_eq!(l.units.expect("units").capacitance, 1e-12, "the default");
    }

    #[test]
    fn two_sequentials_that_disagree_are_refused() {
        let mut l = LibertyClocks::default();
        let e = l.read(r#"library (t) { cell (c) {
            ff ("IQ","IQ_N") { clocked_on : "CLK"; next_state : "D"; }
            latch ("L","L_N") { enable : "E"; data_in : "CLK"; }
            pin (Q) { function : "IQ & L"; timing () { related_pin : "CLK"; timing_type : rising_edge; } } } }"#);
        assert!(e.unwrap_err().contains("pointer order"));
    }

    #[test]
    fn a_bus_is_refused() {
        let mut l = LibertyClocks::default();
        assert!(l.read("library (t) { cell (c) { bus (D) { bus_type : b; } } }").unwrap_err().contains("bus"));
    }

    // isClkTerm: a terminal with no liberty port is never a clock terminal; a pad cell's is.
    #[test]
    fn iterm_facts_follow_the_lookup() {
        let mut l = LibertyClocks::default();
        l.read(r#"library (t) { cell (p) { pad_cell : true; pin (PAD) { } } }"#).expect("parse");
        let f = l.iterm_facts("p", "PAD");
        assert!(f.has_liberty_port && f.cell_is_pad && !f.is_reg_clk);
        assert!(!l.iterm_facts("p", "NOPE").has_liberty_port);
        assert!(!l.iterm_facts("absent", "A").has_liberty_port);
    }
}
