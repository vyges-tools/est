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

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_est::…` paths resolve here.
extern crate self as vyges_est;

pub mod wire;
pub mod liberty;
pub mod clk_network;
pub mod sim;
pub mod placement;
pub mod rc;
pub mod network;
pub mod spef;

/// The `vyges-est` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
