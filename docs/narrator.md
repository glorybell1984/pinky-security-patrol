# Narrator: reports in words, by map section

The dashboard's **Robot reports** card turns patrol events into one sentence each,
by map section instead of coordinates:

| Event | Report (template wording; Claude rephrases) |
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
                                   |  Claude writes one sentence (worker thread)
                                   '- no credentials / no network / error / timeout -> template
```

- `security_task_manager/narrator_node.py`: the ROS node. Narration runs in a worker
  thread, so the patrol never waits for the LLM. `/fleet/narration` keeps the last
  10 reports for a dashboard that connects later. `/fleet/zones` sends the sections
  to the dashboard map.
- `narrator.py`: event -> facts -> sentence. Claude gets only the facts (event, object
  number, warning level, section) and is told not to invent anything.
- `zones.py`: sections as polygons in the map frame. A point just outside every
  section (within 1 m) gets the nearest one.
- Object events use the object's position; robot events (going home, battery) the
  robot's. A follow starts right after "moving", so only "following" is reported.

## Sections

`security_bringup/config/narrator.yaml`: three bands along the `police_map2` arena,
**A** (dock end), **B** (middle), **C** (far end). Rename them, add descriptions, or
change the polygons there (`zone_corners` points per section, all in one flat list
of map x, y). The dashboard draws them on the live map.

## Claude

Model `claude-opus-5-5` at effort `low` (one sentence), 8 s timeout and one retry,
then the template. The request turns on the API's server-side refusal fallback
(`fallbacks: "default"`), so a declined request is answered by another model.

Setup on the laptop (once):

```bash
pip install --user --break-system-packages anthropic   # ROS nodes run on the system Python
export ANTHROPIC_API_KEY=...                           # or: ant auth login
```

The startup log says which mode is active:
`Narrator ready: 3 sections (A, B, C), Claude (claude-opus-5-5, effort low)` or
`... templates: the anthropic package is not installed`. The tag on the dashboard
card shows **Claude** or **template** for the latest report.

Options (`narrator.yaml`): `narrator_mode: template` (never call Claude),
`language: Korean`, `model`, `effort`, `timeout`.

Cost: one short request per reported event (a demo produces a few per minute), a
few hundred tokens each.

## Tests

`security_task_manager/test/test_narrator.py`: sections of the dock, waypoints and
object; template wording; robot vs object position; the Claude path with a fake
client (request shape, one-line output, refusal and errors fall back to the
template).
