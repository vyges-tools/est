# vyges-est — parasitic estimation from placement

> **Part of the Vyges Loom suite.** Install once with `vyges install loom`, then run
> `vyges loom est`. It's also a standalone `vyges-est` binary on your PATH (the
> integration contract for flow authors).

`vyges-est` answers: **before a net is routed, what resistance and capacitance will its wire
have?** A timer needs an RC network for every net to compute delay, and after placement there is
no routed wire yet. So the network is estimated from where the pins are: a Steiner tree joins
them, each branch of the tree becomes a wire segment of a known length on a known layer, and each
segment becomes a resistor with its capacitance split to both ends. Resizing, buffering and
timing-driven placement all read this estimate. A different estimate leads them to different
decisions.

What the estimate depends on, in the order it is decided:

| stage | decides |
| --- | --- |
| **which nets** | power, ground and special nets are skipped. A net straight from a port to a pad gets a 1 mΩ link. An ideal clock (no `set_propagated_clock`) gets nothing. |
| **the tree** | the net's connected pins, driver first, joined by `vyges-stt` (Prim-Dijkstra at `alpha`) |
| **the wire RC** | per corner: `set_layer_rc` / `set_wire_rc`, horizontal and vertical weighted, clock RC for clock nets that are not leaves |
| **the network** | a pi model per branch, a 1 mΩ link for a zero-length one, width scaled by a non-default rule, and a via chain from each pin's layer up to the wire |
| **the SPEF** | one file per corner, the timer's units, the network in its storage order |

## Run it

```sh
vyges install loom                         # one-time
vyges loom est estimate job.json           # -> a JSON report, and the SPEF files the job names
vyges loom est estimate job.json -o r.json # the report to a file
```

A job is the list of commands a flow script would run, in order, with the arguments as the script
passed them:

```json
{
  "steps": [
    { "cmd": "read_lef",     "args": ["tech.lef"] },
    { "cmd": "read_lef",     "args": ["cells.lef"] },
    { "cmd": "read_liberty", "args": ["cells.lib"] },
    { "cmd": "read_def",     "args": ["placed.def"] },
    { "cmd": "create_clock", "args": ["clk"] },
    { "cmd": "set_wire_rc",  "args": ["-signal", "-layer", "metal3"] },
    { "cmd": "set_wire_rc",  "args": ["-clock",  "-layer", "metal5"] },
    { "cmd": "estimate_parasitics", "args": ["-placement", "-spef_file", "out.spef"] }
  ]
}
```

```json
{ "tool": "vyges-est", "status": "estimated", "estimates": 1, "nets": 412,
  "spef_files": ["out.spef"], "log": [] }
```

With more than one corner (`define_corners`), one SPEF is written per corner, with `_<corner>`
inserted before `.spef`, as the reference names them. `read_db` reads a prepared database in
place of `read_lef` and `read_def`. `trace` writes the resolved RC per corner and one decision line
per net, for comparison against a reference run instrumented to print the same fields.

See the full [CLI reference](./reference/vyges-est.md) (generated from `--help`).

## Exit status

| code | status | meaning |
| --- | --- | --- |
| 0 | `estimated` | at least one net was estimated and every requested SPEF was written |
| 2 | `vacuous` | no net was estimated: no estimate step ran, or every corner lacked wire capacitance (EST-0018). **Not a pass.** |
| 2 | `error` | usage, unreadable input, or a step that failed |
| 3 | `refused` | a step or an option this engine does not model; `reason` names it |

⛔ **`vacuous` is not success.** The declared assertion passes only on `estimated`. A run that
estimated nothing fails it, even when nothing was the right answer.

## Correlation

Scored against OpenROAD at pin `da9f29f1`, comparing **every SPEF byte for byte** against a fresh
reference run:

- **8 of 9** of the reference's own `estimate_parasitics -placement` tests. The ninth edits the
  design through the database's Tcl API, which a job cannot express.
- **180 of 186** of the other regression cases that call it, across the reference's modules, with
  **21,289 nets and 0 differing lines**. Each case's design is prepared by the reference and read
  as a database, so the estimate alone is scored.

⚠️ **A number here means nothing without the build.** The reference's own answer moves between
OpenROAD commits, so the pin is part of the claim. `--describe` publishes the pin this binary was
built against.

Every value is single precision wherever the reference's is. Tcl doubles are narrowed to `float`
when a command stores them, sums are accumulated in `float`, and SPEF values are printed as `%g`
with six significant digits. Computing in double precision instead changes the SPEF's last digit
on real designs.

## What it refuses

A refusal is named in `reason`. Nothing is approximated:

- `estimate_parasitics -global_routing`. Only `-placement` is modelled.
- Liberty `bus`, `bundle`, `ff_bank` and `latch_bank` groups.
- A net driven by a tie cell's constant output.
- `set_case_analysis`, `set_logic_*`, `set_disable_timing`, `create_generated_clock`, `set_sense`
  and `set_clock_sense`. They change which pins are clocks without touching the database, so the
  job cannot carry them.
- RC commands before any liberty library is read. The timer's default units are not modelled.

## Where it sits

`est` reads the design database and the timer's libraries, and writes SPEF. Its consumers are the
stages that time a placed design before it is routed: resizing, buffering and timing-driven
placement. The Steiner tree comes from `vyges-stt`. The global-route RC network (the `wire` module
of the same crate) is used by the router.

## Licence

Apache-2.0.
