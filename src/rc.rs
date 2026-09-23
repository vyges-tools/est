// SPDX-License-Identifier: Apache-2.0
//! The estimator's wire R and C — `set_layer_rc`, `set_wire_rc` (EstimateParasitics.tcl) and the
//! state they write in `EstimateParasitics`: the per-corner layer table (`layer_res_` /
//! `layer_cap_`), the per-technology wire RC (`wire_rc_`: signal and clock, H and V, per corner) and
//! the clock / signal layer lists.
//!
//! ⛔ **The precision is the reference's.** The Tcl procs compute in `double`; every `*_cmd` the
//! procs call takes `float` (SWIG), so each stored value is the double NARROWED to float and then
//! widened back — `166666.6667` is stored as `166666.671875`. The database writes
//! (`set_dblayer_wire_rc`) take the full double.
//!
//! ⛔ **`set_layer_rc` without `-corner` rewrites the technology layer** (`set_dblayer_wire_rc`:
//! capacitance per square, resistance per square, edge capacitance ZEROED) — and does so with
//! `cap = 0.0` when only `-resistance` was given. A later `set_wire_rc -layer` with no corner reads
//! the rewritten layer (`dblayer_wire_rc`), not the table.
//!
//! Units: the timer's user units (the first liberty library's), `value × scale` in double with the
//! scales as `float`, per metre by dividing by the distance unit (`[distance_ui_sta 1.0]`).

#![cfg(feature = "odb")]

use std::collections::BTreeMap;
use vyges_opendb::Db;

type Res<T> = Result<T, String>;

/// The timer's user units (`sta::Units`), as `float` scales.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Units {
    pub resistance: f32,
    pub capacitance: f32,
    pub distance: f32,
}

/// One corner's H and V value.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct HV {
    pub h: f64,
    pub v: f64,
}

/// `EstimateParasitics::WireRC` — one technology's (or the shared, `nullptr`) wire RC.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WireRc {
    pub signal_res: Vec<HV>,
    pub signal_cap: Vec<HV>,
    pub clk_res: Vec<HV>,
    pub clk_cap: Vec<HV>,
    /// Layer numbers, in `add_*_layer_cmd` order until `sortClkAndSignalLayers`. ⚠️ Duplicates are
    /// kept: `set_wire_rc -corner ff -layer metal1` then `-corner ss -layer metal1` lists it twice.
    pub signal_layers: Vec<i32>,
    pub clk_layers: Vec<i32>,
}

/// The estimator's RC state.
#[derive(Debug, Clone, Default)]
pub struct Rc {
    /// The corners (`define_corners`), by index; `default` when none were defined.
    pub scenes: Vec<String>,
    /// `wire_rc_`, by technology name; `None` is the shared entry `set_wire_rc` without `-tech`
    /// writes.
    pub wire_rc: BTreeMap<Option<String>, WireRc>,
    /// `layer_res_` / `layer_cap_`: per layer NUMBER, per corner (an entry is sized to the corner
    /// count when first set).
    pub layer_res: BTreeMap<i32, Vec<f64>>,
    pub layer_cap: BTreeMap<i32, Vec<f64>>,
    pub log: Vec<String>,
}

/// A value through a `float` parameter.
fn narrow(v: f64) -> f64 {
    f64::from(v as f32)
}

/// `sta::parse_key_args`: `-key value` pairs and `-flag`s; returns (keys, flags, positional).
fn parse_key_args(args: &[String], keys: &[&str], flags: &[&str]) -> Res<(BTreeMap<String, String>, Vec<String>)> {
    let (mut k, mut f) = (BTreeMap::new(), Vec::new());
    let mut i = 0;
    while i < args.len() {
        let a = &args[i];
        if keys.contains(&a.as_str()) {
            let v = args.get(i + 1).ok_or_else(|| format!("{a} requires a value"))?;
            k.insert(a.clone(), v.clone());
            i += 2;
        } else if flags.contains(&a.as_str()) {
            f.push(a.clone());
            i += 1;
        } else {
            return Err(format!("unexpected argument {a}"));
        }
    }
    Ok((k, f))
}

fn num(s: &str) -> Res<f64> {
    s.trim().parse::<f64>().map_err(|_| format!("{s}: not a number"))
}

impl Rc {
    pub fn new() -> Rc {
        Rc { scenes: vec!["default".to_string()], ..Default::default() }
    }

    /// `define_corners` — replaces the corners.
    pub fn define_corners(&mut self, names: &[String]) {
        self.scenes = names.to_vec();
    }

    /// `parse_scene_or_null`: the corner a `-corner` names, or `None` (every corner).
    fn scene(&self, keys: &BTreeMap<String, String>) -> Res<Option<usize>> {
        match keys.get("-corner") {
            None => Ok(None),
            Some(n) => self.scenes.iter().position(|s| s == n).map(Some).ok_or_else(|| format!("corner {n} not found")),
        }
    }

    /// `setLayerRC(layer, corner, res, cap)` — through `set_layer_rc_cmd`'s `float`s.
    fn set_layer_rc_table(&mut self, number: i32, corner: usize, res: f64, cap: f64) {
        let n = self.scenes.len();
        let r = self.layer_res.entry(number).or_default();
        r.resize(n.max(r.len()), 0.0);
        r[corner] = narrow(res);
        let c = self.layer_cap.entry(number).or_default();
        c.resize(n.max(c.len()), 0.0);
        c[corner] = narrow(cap);
    }

    /// `layerRC(layer, corner)` — 0 when the layer or corner was never set.
    pub fn layer_rc(&self, number: i32, corner: usize) -> (f64, f64) {
        let get = |m: &BTreeMap<i32, Vec<f64>>| m.get(&number).and_then(|v| v.get(corner)).copied().unwrap_or(0.0);
        (get(&self.layer_res), get(&self.layer_cap))
    }

    /// `set_layer_rc [-layer l | -via v] [-capacitance c] [-resistance r] [-corner c]`.
    pub fn set_layer_rc(&mut self, db: &mut Db, units: Units, args: &[String]) -> Res<()> {
        let (keys, _) = parse_key_args(args, &["-layer", "-via", "-capacitance", "-resistance", "-corner"], &[])?;
        if keys.contains_key("-layer") && keys.contains_key("-via") {
            return Err("[ERROR EST-0201] Use -layer or -via but not both.".into());
        }
        let corner = self.scene(&keys)?;
        let d = f64::from(units.distance);
        if let Some(name) = keys.get("-layer") {
            if !db.tech_get_layers().contains(name) {
                return Err(format!("[ERROR EST-0202] layer {name} not found."));
            }
            if db.layer_get_routing_level(name) == 0 {
                return Err(format!("[ERROR EST-0203] {name} is not a routing layer."));
            }
            if !keys.contains_key("-capacitance") && !keys.contains_key("-resistance") {
                return Err("[ERROR EST-0204] missing -capacitance or -resistance argument.".into());
            }
            // F/m and ohm/m: `[unit_ui_sta $v] / [distance_ui_sta 1.0]`.
            let cap = keys.get("-capacitance").map(|v| num(v)).transpose()?.map_or(0.0, |v| (v * f64::from(units.capacitance)) / (1.0 * d));
            let res = keys.get("-resistance").map(|v| num(v)).transpose()?.map_or(0.0, |v| (v * f64::from(units.resistance)) / (1.0 * d));
            let corners: Vec<usize> = match corner {
                Some(c) => vec![c],
                None => {
                    // Only without -corner: the technology layer is rewritten too.
                    set_dblayer_wire_rc(db, name, res, cap)?;
                    (0..self.scenes.len()).collect()
                }
            };
            let number = db.layer_get_number(name);
            for c in corners {
                self.set_layer_rc_table(number, c, res, cap);
            }
            Ok(())
        } else if let Some(name) = keys.get("-via") {
            if !db.tech_get_layers().contains(name) {
                return Err(format!("[ERROR EST-0205] via {name} not found."));
            }
            if keys.contains_key("-capacitance") {
                self.log.push("[WARNING EST-0206] -capacitance not supported for vias.".into());
            }
            let Some(r) = keys.get("-resistance") else {
                return Err("[ERROR EST-0208] no -resistance specified for via.".into());
            };
            let res = num(r)? * f64::from(units.resistance);
            let corners: Vec<usize> = match corner {
                Some(c) => vec![c],
                None => {
                    db.layer_set_resistance(name, res).map_err(|e| e.to_string())?;
                    (0..self.scenes.len()).collect()
                }
            };
            let number = db.layer_get_number(name);
            for c in corners {
                self.set_layer_rc_table(number, c, res, 0.0);
            }
            Ok(())
        } else {
            Err("[ERROR EST-0209] missing -layer or -via argument.".into())
        }
    }

    /// `set_wire_rc` — every form the corpus uses; `-redistribution_layer` is refused.
    pub fn set_wire_rc(&mut self, db: &Db, units: Units, args: &[String]) -> Res<()> {
        let (keys, flags) = parse_key_args(
            args,
            &["-layer", "-layers", "-resistance", "-capacitance", "-corner", "-h_resistance", "-h_capacitance", "-v_resistance", "-v_capacitance", "-tech"],
            &["-clock", "-signal", "-data", "-redistribution_layer"],
        )?;
        let corner = self.scene(&keys)?;
        if flags.iter().any(|f| f == "-redistribution_layer") {
            return Err("set_wire_rc -redistribution_layer: RDL chips are not modelled".into());
        }
        // parse_wire_rc_techs: -tech names one technology; none means the shared entry.
        // ⚠️ UNWITNESSED: with one technology the per-technology and shared entries resolve alike,
        // so storing -tech values in the shared entry changes no corpus case; the cases that tell
        // them apart (set_wire_rc_selectors, _mixed_tech) script odb directly and are refused.
        let techs: Vec<Option<String>> = match keys.get("-tech") {
            Some(t) if *t == db.tech_get_name() => vec![Some(t.clone())],
            Some(t) => return Err(format!("[ERROR EST-0030] technology {t} not found.")),
            None => vec![None],
        };
        let d = f64::from(units.distance);
        let (clk, signal) = (flags.iter().any(|f| f == "-clock"), flags.iter().any(|f| f == "-signal"));
        let (mut hr, mut hc, mut vr, mut vc) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let table_or_db = |me: &Rc, layer: &str| -> (f64, f64) {
            match corner {
                None => dblayer_wire_rc(db, layer),
                Some(c) => me.layer_rc(db.layer_get_number(layer), c),
            }
        };
        let mut added = Vec::new();
        if let Some(list) = keys.get("-layers") {
            if ["-h_resistance", "-h_capacitance", "-v_resistance", "-v_capacitance"].iter().any(|k| keys.contains_key(*k)) {
                return Err("[ERROR EST-0001] Use -layers or -resistance/-capacitance but not both.".into());
            }
            if keys.contains_key("-layer") {
                return Err("[ERROR EST-0006] Use -layers or -layer but not both.".into());
            }
            let dirs: BTreeMap<String, String> = db.layers_with_direction().map_err(|e| e.to_string())?.into_iter().collect();
            let (mut th_r, mut th_c, mut tv_r, mut tv_c, mut nh, mut nv) = (0.0, 0.0, 0.0, 0.0, 0, 0);
            for name in list.split_whitespace() {
                if !db.tech_get_layers().iter().any(|l| l == name) {
                    return Err(format!("[ERROR EST-0002] layer {name} not found."));
                }
                let (r, c) = table_or_db(self, name);
                match dirs.get(name).map(String::as_str) {
                    Some("HORIZONTAL") => {
                        (th_r, th_c, nh) = (th_r + r, th_c + c, nh + 1);
                    }
                    Some("VERTICAL") => {
                        (tv_r, tv_c, nv) = (tv_r + r, tv_c + c, nv + 1);
                    }
                    _ => {
                        (th_r, th_c, nh) = (th_r + r, th_c + c, nh + 1);
                        (tv_r, tv_c, nv) = (tv_r + r, tv_c + c, nv + 1);
                    }
                }
                added.push(db.layer_get_number(name));
            }
            if nh == 0 {
                return Err("[ERROR EST-0016] No horizontal layer specified.".into());
            }
            if nv == 0 {
                return Err("[ERROR EST-0017] No vertical layer specified.".into());
            }
            (hr, hc, vr, vc) = (th_r / f64::from(nh), th_c / f64::from(nh), tv_r / f64::from(nv), tv_c / f64::from(nv));
        } else if let Some(name) = keys.get("-layer") {
            if !db.tech_get_layers().iter().any(|l| l == name) {
                return Err(format!("[ERROR EST-0015] layer {name} not found."));
            }
            let (r, c) = table_or_db(self, name);
            (hr, hc, vr, vc) = (r, c, r, c);
            added.push(db.layer_get_number(name));
        } else {
            let per_m = |k: &str, scale: f32| -> Res<Option<f64>> { keys.get(k).map(|v| num(v).map(|v| (v * f64::from(scale)) / (1.0 * d))).transpose() };
            if let Some(r) = per_m("-resistance", units.resistance)? {
                (hr, vr) = (r, r);
            }
            if let Some(c) = per_m("-capacitance", units.capacitance)? {
                (hc, vc) = (c, c);
            }
            if let Some(r) = per_m("-h_resistance", units.resistance)? {
                hr = r;
            }
            if let Some(c) = per_m("-h_capacitance", units.capacitance)? {
                hc = c;
            }
            if let Some(r) = per_m("-v_resistance", units.resistance)? {
                vr = r;
            }
            if let Some(c) = per_m("-v_capacitance", units.capacitance)? {
                vc = c;
            }
        }
        // add_wire_rc_layers, per layer in the order given.
        for &number in &added {
            for t in &techs {
                let w = self.wire_rc.entry(t.clone()).or_default();
                if clk || !signal {
                    w.clk_layers.push(number);
                }
                if signal || !clk {
                    w.signal_layers.push(number);
                }
            }
        }
        let (set_signal, set_clk) = if !signal && !clk { (true, true) } else { (signal, clk) };
        let what = match (set_signal, set_clk) {
            (true, true) => "Signal/clock",
            (true, false) => "Signal",
            _ => "Clock",
        };
        for (v, msg) in [(hr, "10] {} horizontal wire resistance is 0."), (vr, "11] {} vertical wire resistance is 0."), (hc, "12] {} horizontal wire capacitance is 0."), (vc, "13] {} vertical wire capacitance is 0.")] {
            if v == 0.0 {
                self.log.push(format!("[WARNING EST-00{}", msg.replace("{}", what)));
            }
        }
        let corners: Vec<usize> = corner.map_or_else(|| (0..self.scenes.len()).collect(), |c| vec![c]);
        let n = self.scenes.len();
        for c in corners {
            for t in &techs {
                let w = self.wire_rc.entry(t.clone()).or_default();
                let set = |res: &mut Vec<HV>, cap: &mut Vec<HV>| {
                    res.resize(n.max(res.len()), HV::default());
                    cap.resize(n.max(cap.len()), HV::default());
                    (res[c].h, cap[c].h, res[c].v, cap[c].v) = (narrow(hr), narrow(hc), narrow(vr), narrow(vc));
                };
                if set_signal {
                    set(&mut w.signal_res, &mut w.signal_cap);
                }
                if set_clk {
                    set(&mut w.clk_res, &mut w.clk_cap);
                }
            }
        }
        Ok(())
    }

    /// `sortClkAndSignalLayers` — by layer number, every technology's lists.
    pub fn sort_clk_and_signal_layers(&mut self) {
        for w in self.wire_rc.values_mut() {
            w.clk_layers.sort();
            w.signal_layers.sort();
        }
    }

    /// `resolveWireRC(category)`: the current technology's entry when that category is set there,
    /// else the shared entry's — per CATEGORY, so a technology can take its resistance from one
    /// entry and its layers from the other.
    pub fn resolve<'a, T>(&'a self, tech: &str, category: fn(&WireRc) -> &Vec<T>) -> &'a [T] {
        for key in [Some(tech.to_string()), None] {
            if let Some(w) = self.wire_rc.get(&key) {
                if !category(w).is_empty() {
                    return category(w);
                }
            }
        }
        &[]
    }

    /// The per-corner resolved values, as `wire{Signal,Clk}{H,V}{Resistance,Capacitance}` return them.
    pub fn resolved(&self, tech: &str, corner: usize) -> [f64; 8] {
        let at = |v: &[HV], f: fn(&HV) -> f64| v.get(corner).map_or(0.0, f);
        let (sr, sc) = (self.resolve(tech, |w| &w.signal_res), self.resolve(tech, |w| &w.signal_cap));
        let (cr, cc) = (self.resolve(tech, |w| &w.clk_res), self.resolve(tech, |w| &w.clk_cap));
        [at(sr, |x| x.h), at(sr, |x| x.v), at(sc, |x| x.h), at(sc, |x| x.v), at(cr, |x| x.h), at(cr, |x| x.v), at(cc, |x| x.h), at(cc, |x| x.v)]
    }

    /// The RC state as the instrumented reference prints it at `estimateWireParasitics` (`VYGE|rc`,
    /// `layer_rc`, `layers`).
    pub fn trace(&self, tech: &str) -> String {
        let mut t = String::new();
        for (k, name) in self.scenes.iter().enumerate() {
            let v = self.resolved(tech, k);
            t.push_str(&format!(
                "VYGE|rc|{k}|{name}|sig_h_r={}|sig_v_r={}|sig_h_c={}|sig_v_c={}|clk_h_r={}|clk_v_r={}|clk_h_c={}|clk_v_c={}\n",
                fmt_g17(v[0]), fmt_g17(v[1]), fmt_g17(v[2]), fmt_g17(v[3]), fmt_g17(v[4]), fmt_g17(v[5]), fmt_g17(v[6]), fmt_g17(v[7])
            ));
            for (&l, r) in &self.layer_res {
                if k < r.len() {
                    let (res, cap) = self.layer_rc(l, k);
                    t.push_str(&format!("VYGE|layer_rc|{k}|{l}|r={}|c={}\n", fmt_g17(res), fmt_g17(cap)));
                }
            }
        }
        let list = |v: &[i32]| v.iter().map(|n| format!("{n};")).collect::<String>();
        t.push_str(&format!("VYGE|layers|signal={}|clk={}\n", list(self.resolve(tech, |w| &w.signal_layers)), list(self.resolve(tech, |w| &w.clk_layers))));
        t
    }
}

/// `set_dblayer_wire_rc layer res cap` (ohm/m, F/m): edge capacitance zeroed, capacitance per
/// square `cap * 1e6 / width_um`, resistance per square `width_um * 1e-6 * res`.
fn set_dblayer_wire_rc(db: &mut Db, layer: &str, res: f64, cap: f64) -> Res<()> {
    db.layer_set_edge_capacitance(layer, 0.0).map_err(|e| e.to_string())?;
    let width = f64::from(db.layer_get_width(layer)) / f64::from(db.tech_get_db_units_per_micron());
    db.layer_set_capacitance(layer, cap * 1e6 / width).map_err(|e| e.to_string())?;
    db.layer_set_resistance(layer, width * 1e-6 * res).map_err(|e| e.to_string())?;
    Ok(())
}

/// `dblayer_wire_rc layer` → (ohm/m, F/m) from the technology layer: resistance per square over
/// the width, and `1 * width * area_cap + edge_cap * 2` pF/µm.
pub fn dblayer_wire_rc(db: &Db, layer: &str) -> (f64, f64) {
    let width = f64::from(db.layer_get_width(layer)) / f64::from(db.tech_get_db_units_per_micron());
    let res_per_micron = db.layer_get_resistance(layer) / width;
    let cap_pf_per_micron = 1.0 * width * db.layer_get_capacitance(layer) + db.layer_get_edge_capacitance(layer) * 2.0;
    (res_per_micron * 1e6, cap_pf_per_micron * 1e-12 * 1e6)
}

/// C's `%.17g` (the reference trace's `{:.17g}`): 17 significant digits, fixed notation when the
/// exponent is in `-4..17`, scientific otherwise, trailing zeros removed, a two-digit exponent.
pub fn fmt_g17(v: f64) -> String {
    fmt_g(v, 17)
}

/// C's `%.<p>g`.
pub fn fmt_g(v: f64, p: usize) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".into() } else if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    let sci = format!("{:.*e}", p - 1, v);
    let (mant, exp) = sci.split_once('e').expect("exponent");
    let x: i32 = exp.parse().expect("exponent value");
    let strip = |s: String| if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if x >= -4 && x < p as i32 {
        strip(format!("{:.*}", (p as i32 - 1 - x).max(0) as usize, v))
    } else {
        let m = strip(mant.to_string());
        format!("{m}e{}{:02}", if x < 0 { '-' } else { '+' }, x.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `%.17g` as the reference trace printed these values.
    #[test]
    fn g17_matches_c() {
        assert_eq!(fmt_g17(166666.671875), "166666.671875");
        assert_eq!(fmt_g17(6516000.0), "6516000");
        assert_eq!(fmt_g17(1.3272000165542863e-10), "1.3272000165542863e-10");
        assert_eq!(fmt_g17(0.0), "0");
    }

    /// A value through a `float` parameter: 166666.6667 is stored as 166666.671875.
    #[test]
    fn a_stored_value_is_narrowed_to_float() {
        assert_eq!(narrow(0.0001666666666666667 * 1000.0 / 1e-6), 166666.671875);
    }
}
