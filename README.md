# vyges-est

Parasitic estimation before detailed routing: the RC network of every net, built from its
placement or from its global route, in the form a static timer reads.

- **Placement** (`vyges-est estimate`): a Steiner tree per net from its pin locations, a pi model
  per branch with each corner's wire RC, via chains from the pins' layers, written as SPEF.
  Matches OpenROAD's `estimate_parasitics -placement` SPEF byte for byte on 8 of 9 of its own
  tests and 180 of 186 of the other regression cases that call it (21,289 nets), at the pin
  `--describe` reports.
- **Global route** (the `wire` module): one node per routing point, a resistor per route segment
  with its capacitance split to both ends, and a resistor from each pin to the grid point it
  attaches to. A library for the router; not a command.

```sh
vyges install loom
vyges loom est estimate job.json
vyges loom est --describe
```

Documentation: [`docs/src/est.md`](docs/src/est.md) (an mdBook; `mdbook build docs`), with the
[CLI reference](docs/src/reference/vyges-est.md).

## Build

The command-line engine reads the design database through `vyges-opendb`, which builds OpenROAD's
database library from source. Check out `vyges-tools/opendb`, `opendb-lib` and `loom` beside this
repository (see `.github/workflows/ci.yml`), then:

```sh
cargo build --release --features cli
cargo test  --release --features cli
```

Without `--features cli` the library builds with no C++ dependency, and the pure rules (liberty
clocks, the global-route network) are tested on their own.

Licensed under Apache-2.0.
