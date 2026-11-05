// SPDX-License-Identifier: Apache-2.0
//! Timing modes and the scenes bound to them (`set_mode`, `define_scene`), as far as `est` reads
//! them: each mode holds its own clocks, and each scene names one mode.
//!
//! 🔑 Reference rules (OpenSTA at the 4155ac3 pin):
//! - `set_mode N` (`Sta::setCmdMode`): a mode that exists becomes the command mode. A new name
//!   makes a mode, appended; when the only mode was `default`, that mode is DELETED first, with
//!   its constraints (a clock made before the first `set_mode` is gone), and when the only scene
//!   was `default`, it is bound to the new mode.
//! - `define_scene N [-mode M]` (`Sta::makeScene`): the scene is bound to `-mode`, else to the
//!   command mode, which must exist (STA-1572). When the only scene was `default`, it is DELETED
//!   first; the new scene is appended.
//! - `create_clock`, `set_propagated_clock`, `set_clock_latency` write the command mode.
//! - `estimateWireParasitics` asks about clocks per mode: `isSkipPin` over every mode, and the
//!   per-scene skip in each scene's own mode. Modes holding the SAME clock definitions answer
//!   alike, which is what is modelled; modes whose clocks differ are refused by name.
#![cfg(feature = "odb")]

use crate::placement::Clocks;
use crate::rc::Rc;

/// The modes in creation order, the command mode, and each scene's mode (by index, aligned with
/// `Rc::scenes`).
#[derive(Debug, Clone)]
pub struct Modes {
    modes: Vec<(String, Clocks)>,
    cmd: usize,
    scene_modes: Vec<usize>,
}

impl Default for Modes {
    fn default() -> Self {
        Modes { modes: vec![("default".to_string(), Clocks::default())], cmd: 0, scene_modes: vec![0] }
    }
}

impl Modes {
    /// The command mode's clocks: what a clock command writes.
    pub fn cmd_clocks(&mut self) -> &mut Clocks {
        &mut self.modes[self.cmd].1
    }

    /// The mode names, in creation order.
    pub fn names(&self) -> Vec<&str> {
        self.modes.iter().map(|(n, _)| n.as_str()).collect()
    }

    /// The mode each scene of `rc` is bound to.
    pub fn scene_mode_names(&self) -> Vec<&str> {
        self.scene_modes.iter().map(|&m| self.modes[m].0.as_str()).collect()
    }

    /// `Sta::setCmdMode`.
    pub fn set_cmd_mode(&mut self, rc: &Rc, name: &str) -> Result<(), String> {
        if let Some(i) = self.modes.iter().position(|(n, _)| n == name) {
            self.cmd = i;
            return Ok(());
        }
        if self.modes.len() == 1 && self.modes[0].0 == "default" {
            let lone_default_scene = rc.scenes.len() == 1 && rc.scenes[0] == "default";
            if !lone_default_scene {
                // The reference deletes a mode its scenes still point to.
                return Err(format!("set_mode {name}: deletes the default mode while scenes {:?} are bound to it (not modelled)", rc.scenes));
            }
            self.modes.clear();
        }
        self.modes.push((name.to_string(), Clocks::default()));
        self.cmd = self.modes.len() - 1;
        if rc.scenes.len() == 1 && rc.scenes[0] == "default" {
            self.scene_modes = vec![self.cmd];
        }
        Ok(())
    }

    /// `define_scene name [-mode m] ...` (`Sta::makeScene`): only the name and the mode reach est.
    pub fn define_scene(&mut self, rc: &mut Rc, args: &[String]) -> Result<(), String> {
        let mut name = None;
        let mut mode = None;
        let mut i = 0;
        while i < args.len() {
            match args[i].as_str() {
                "-mode" => {
                    mode = Some(args.get(i + 1).ok_or("define_scene -mode needs a mode")?.clone());
                    i += 1;
                }
                "-liberty" | "-liberty_min" | "-liberty_max" | "-spef" | "-spef_min" | "-spef_max" => i += 1,
                a if a.starts_with('-') => return Err(format!("define_scene {a}: not modelled")),
                a => name = Some(a.to_string()),
            }
            i += 1;
        }
        let name = name.ok_or("define_scene needs a name")?;
        let m = match mode {
            Some(m) => self.modes.iter().position(|(n, _)| *n == m).ok_or_else(|| format!("[ERROR STA-1572] mode {m} not found."))?,
            None => self.cmd,
        };
        if rc.scenes.len() == 1 && rc.scenes[0] == "default" {
            rc.scenes.clear();
            self.scene_modes.clear();
        }
        rc.scenes.push(name);
        self.scene_modes.push(m);
        Ok(())
    }

    /// `define_corners`: the scenes it makes are bound to the command mode.
    pub fn define_corners(&mut self, rc: &mut Rc, names: &[String]) {
        rc.define_corners(names);
        self.scene_modes = vec![self.cmd; names.len()];
    }

    /// The clocks every mode agrees on, for the estimate; a refusal when the modes' clocks differ
    /// (the per-mode skip is then not one answer).
    pub fn estimate_clocks(&mut self) -> Result<&mut Clocks, String> {
        let first = &self.modes[0].1;
        if let Some((n, _)) = self.modes.iter().skip(1).find(|(_, c)| !c.same_definitions(first)) {
            return Err(format!("modes {} and {n} hold different clocks: the per-mode clock skip is not modelled", self.modes[0].0));
        }
        Ok(&mut self.modes[0].1)
    }

    /// `ensureClkNetwork` for every mode.
    pub fn network_read(&mut self) {
        for (_, c) in &mut self.modes {
            c.network_read();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    /// `Sta::setCmdMode`: the first new mode replaces a lone `default` mode, its clocks with it,
    /// and takes over the lone `default` scene.
    #[test]
    fn a_first_mode_replaces_the_default_mode_and_its_clocks() {
        let rc = Rc::new();
        let mut m = Modes::default();
        m.cmd_clocks().create_clock("early", &s(&["clk"]));
        m.set_cmd_mode(&rc, "func").unwrap();
        assert_eq!(m.names(), vec!["func"]);
        assert!(m.cmd_clocks().clocks().is_empty(), "the default mode's clock went with it");
        assert_eq!(m.scene_mode_names(), vec!["func"]);
        m.set_cmd_mode(&rc, "test").unwrap();
        assert_eq!(m.names(), vec!["func", "test"], "a later mode is appended");
        assert_eq!(m.scene_mode_names(), vec!["test"], "a lone default scene follows each new mode");
        m.set_cmd_mode(&rc, "func").unwrap();
        assert_eq!(m.scene_mode_names(), vec!["test"], "selecting an existing mode rebinds nothing");
    }

    /// `Sta::makeScene`: the first scene replaces a lone `default` scene; later ones append; a
    /// scene is bound to `-mode`, else the command mode, and an unknown mode is STA-1572.
    #[test]
    fn a_first_scene_replaces_the_default_scene_and_binds_its_mode() {
        let mut rc = Rc::new();
        let mut m = Modes::default();
        m.set_cmd_mode(&rc, "func").unwrap();
        m.set_cmd_mode(&rc, "test").unwrap();
        m.define_scene(&mut rc, &s(&["scene_func", "-mode", "func", "-liberty", "lib"])).unwrap();
        assert_eq!(rc.scenes, s(&["scene_func"]));
        m.define_scene(&mut rc, &s(&["scene_test", "-liberty", "lib"])).unwrap();
        assert_eq!(rc.scenes, s(&["scene_func", "scene_test"]));
        assert_eq!(m.scene_mode_names(), vec!["func", "test"], "no -mode: the command mode");
        let e = m.define_scene(&mut rc, &s(&["x", "-mode", "nope", "-liberty", "lib"])).unwrap_err();
        assert!(e.contains("STA-1572"), "{e}");
    }

    /// The estimate reads one clock answer only when every mode holds the same clocks.
    #[test]
    fn modes_with_different_clocks_are_refused() {
        let rc = Rc::new();
        let mut m = Modes::default();
        for mode in ["func", "test"] {
            m.set_cmd_mode(&rc, mode).unwrap();
            m.cmd_clocks().create_clock("core_clock", &s(&["clk"]));
        }
        assert!(m.estimate_clocks().is_ok(), "same clock in both modes");
        m.set_cmd_mode(&rc, "scan").unwrap();
        m.cmd_clocks().create_clock("scan_clock", &s(&["sclk"]));
        assert!(m.estimate_clocks().unwrap_err().contains("different clocks"));
    }

    /// Same clocks, but a pin propagated in one mode only: the modes answer `isIdealClock`
    /// differently for that pin, so they are not one answer either.
    #[test]
    fn a_pin_propagated_in_one_mode_only_is_a_difference() {
        let rc = Rc::new();
        let mut m = Modes::default();
        for mode in ["func", "test"] {
            m.set_cmd_mode(&rc, mode).unwrap();
            m.cmd_clocks().create_clock("core_clock", &s(&["clk"]));
        }
        m.cmd_clocks().set_propagated_clock(&[crate::placement::ClockObject::Pin("r1/CK".into())]).unwrap();
        assert!(m.estimate_clocks().unwrap_err().contains("different clocks"));
    }
}
