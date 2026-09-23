# vyges loom est — CLI reference

_Generated from `vyges loom est --help` — this page is the tool's own output, verbatim._

```text
vyges loom est — parasitic estimation: the RC network of every placed net, as SPEF

USAGE:
  vyges loom est estimate <job.json> [-o FILE]
  vyges loom est --describe
  vyges loom est --help
  vyges loom est --version

JOB FIELDS:
  steps     required — the commands in order, each {"cmd": ..., "args": [...]}:
              read_lef, read_def, read_db, read_liberty [-corner c], define_corners,
              create_clock <sources>, set_propagated_clock, set_layer_rc, set_wire_rc,
              estimate_parasitics -placement [-spef_file F]
  alpha     the Steiner builder's Prim-Dijkstra alpha (default 0.3)
  trace     write the RC state and one decision line per net to this path

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

EXIT STATUS:
  0  estimated    at least one net was estimated and every requested SPEF was written
  2  vacuous      no net was estimated — no estimate step, or every corner lacked wire
                  capacitance (EST-0018). NOT a pass.
  2  error        usage, unreadable input, or a step that failed
  3  refused      a step or an option this engine does not model — see `reason`
```
