# Narrator: reports in words, by map section

The dashboard's **Robot reports** card turns patrol events into one sentence each,
by map section instead of coordinates:

| Event | Report (template wording; the LLM rephrases) |
|---|---|
| Level 1, new object | In section C, a suspicious object (#42) has been detected. |
| Level 2, still there | In section C, object #42 is still there and has not moved. |
| Level 3, picked up | In section B, someone just picked up object #42 and I am following it. |
| Lost while following | In section B, I lost sight of object #42 and am going back to patrol. |
| Removed | In section C, object #42 is no longer at its place. |
| Acknowledged | In section C, object #42 was checked and cleared by the operator. |
| Going home / docked | In section A, I am heading back to my dock. / I am on my dock. |
| Battery low / critical | In section B, my battery is low. |

## How it works

```
task_manager --/fleet/events--> narrator --/fleet/narration--> dashboard "Robot reports"
                                   |  section from the event's map position (zones)
                                   |  LLM provider writes one sentence (worker thread)
                                   '- no provider / credentials / network, error, timeout -> template
```

- `security_task_manager/narrator_node.py`: the ROS node. Narration runs in a worker
  thread, so the patrol never waits for the LLM. `/fleet/narration` keeps the last
  10 reports for a dashboard that connects later. `/fleet/zones` sends the sections
  to the dashboard map.
- `narrator.py`: event -> facts -> sentence, the provider interface and the Claude
  provider. The LLM gets only the facts (event, object number, warning level,
  section) and is told not to invent anything.
- `zones.py`: sections as polygons in the map frame. A point just outside every
  section (within 1 m) gets the nearest one.
- Object events use the object's position; robot events (going home, battery) the
  robot's. A follow starts right after "moving", so only "following" is reported.

## Sections

`security_bringup/config/narrator.yaml`: three bands along the `police_map2` arena,
**A** (dock end), **B** (middle), **C** (far end). Rename them, add descriptions, or
change the polygons there (`zone_corners` points per section, all in one flat list
of map x, y). The dashboard draws them on the live map.

## LLM providers

The narrator talks to the LLM through its own small interface
(`narrator.LLMProvider`: `complete(system, user)` -> sentence or `None`), so the
model behind it can change without touching the narrator, the node or the
dashboard.

| `llm_provider` | Uses | Needs |
|---|---|---|
| `claude` (default) | Anthropic's official SDK, `claude-opus-5-5` at effort `low`, 8 s timeout, one retry, server-side refusal fallback (`fallbacks: "default"`) | `pip install anthropic` + a key |
| `none` | Templates only | nothing |

Adding a provider: subclass `LLMProvider` in `narrator.py` (return `None` when it
cannot answer; set `off_reason` when it can never work), and register it in
`PROVIDERS`. Then `llm_provider: <name>` in `narrator.yaml`.

### Claude setup (laptop, once)

```bash
pip install --user --break-system-packages anthropic   # ROS nodes run on the system Python (done 2026-11-11, 2.8.0)
echo 'export ANTHROPIC_API_KEY=sk-ant-...' >> ~/.bashrc  # your key; never put it in this repository
```

Open a new terminal (so the key is set) before `ros2 launch security_bringup laptop.launch.xml`.
The narrator's startup line says which mode is active:

```
Narrator ready: 3 sections (A, B, C), LLM claude (claude-opus-5-5, effort low)
Narrator ready: 3 sections (A, B, C), templates: no Claude credentials (set ANTHROPIC_API_KEY, or run `ant auth login`)
```

The tag on the dashboard card shows **claude** or **template** for the latest report.
Rejected key or unknown model: templates for the rest of the run (logged once).
Network down, timeout or rate limit: the template for that report, Claude again
after 10-30 s.

Options (`narrator.yaml`): `llm_provider`, `llm_model` ('' = provider default),
`llm_effort`, `llm_timeout`, `language`.

Cost: one short request per reported event (a demo produces a few per minute), a
few hundred tokens each.

## Tests

`security_task_manager/test/test_narrator.py`: sections of the dock, waypoints and
object; template wording; robot vs object position; the Claude path with a fake
client (request shape, one-line output, refusal and errors fall back to the
template); any provider through the interface; the real SDK with and without a key.
