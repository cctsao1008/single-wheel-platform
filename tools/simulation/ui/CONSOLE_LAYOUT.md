# Console Information Architecture

The browser console is a presentation surface over already-produced simulator-neutral evidence.

```text
+-------------------------------------------------------------------+
| Header: project identity | trace status | contract | load trace   |
+----------+--------------------------------------+-----------------+
|          |                                      |                 |
| Sidebar  | Model viewport                       | Inspector       |
|          |                                      |                 |
| setup    | side: pitch / forward motion        | evidence        |
| sim      | front: roll / reaction phase        | state           |
| state    |                                      | applied input   |
| inputs   | split / side / front                | boundary        |
| evidence |                                      |                 |
| contract |                                      |                 |
+----------+--------------------------------------+-----------------+
| Transport: restart | play/pause | speed | scrub | time | log     |
+-------------------------------------------------------------------+
```

## Responsibility boundary

| Surface | Responsibility | Explicit non-responsibility |
| --- | --- | --- |
| Header | identify loaded evidence and viewer contract | infer backend truth or authority |
| Sidebar | navigate presentation surfaces and select model view | configure controller/firmware |
| Model viewport | project common state into a human-readable drawing | integrate physics |
| Inspector | display values actually carried by the trace | synthesize missing state |
| Transport | replay and scrub recorded evidence | alter simulation time or rerun a backend |
| Event log | report local viewer events and validation failures | act as a production/runtime log |

The common v2 trace currently carries plant state and applied torque, but it does not carry estimator state, control demand, runtime authority, or saturation provenance. Those must remain visibly unavailable until a future evidence contract introduces them explicitly.

This distinction is deliberate: a polished console is not permission to invent telemetry.
