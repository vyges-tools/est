// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation before detailed routing: the RC network of every net, built
//! from its placement (Steiner trees) or from its global route, as the timer then reads it.
//!
//! Modules follow the reference's split:
//! - [`wire`] — `MakeWireParasitics`: a globally routed net's network (one node per routing
//!   point, a resistor per segment, a pin attachment per pin). The global router's own
//!   partial-slack calls read it.
//! - [`liberty`] — what a timing library says that estimation and routing read: register clocks,
//!   pads, arc roles (`isRegClk`, `isNonLeafClock`).
//! - [`clk_network`] (feature `odb`) — the timer's clock network (`findClkNets`).
//! - [`placement`] (feature `odb`) — `estimate_parasitics -placement`: which nets get a network,
//!   and each one's Steiner tree. ⬜ The network itself is not built yet.

pub mod wire;
pub mod liberty;
pub mod clk_network;
pub mod placement;
