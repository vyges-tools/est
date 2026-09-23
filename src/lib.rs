// SPDX-License-Identifier: Apache-2.0
//! `vyges-est` — parasitic estimation before detailed routing: the RC network of every net, built
//! from its placement (Steiner trees) or from its global route, as the timer then reads it.
//!
//! Modules follow the reference's split:
//! - [`wire`] — `MakeWireParasitics`: a globally routed net's network (one node per routing
//!   point, a resistor per segment, a pin attachment per pin). The global router's own
//!   partial-slack calls read it.
//! - ⬜ the placement path (`estimateWireParasiticSteiner`) — not built yet.

pub mod wire;
