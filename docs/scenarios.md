# Demo scenarios

The step-by-step demo is in [demo_guide.md](demo_guide.md).

1. Robot patrols its loop.
2. A box with marker 7 is placed on the route -> level 1, position logged, frame visible in RViz.
3. Next loop: box still there -> level 2, robot holds facing it, lamp blinks, UI alert.
4. Someone picks up the box -> level 3, robot follows at about 0.7 m.
5. Marker hidden -> after 5 s robot goes to the last position, reports LOST_TRACK, resumes patrol.
6. Show the CSV log (`~/patrol_logs`) and the Event log tab for marker 7.

## Test checklist

| Check | Fake Pinky | Real Pinky |
|---|---|---|
| Level 1 on first sighting | ✅ tested | ⬜ |
| Level 2 on the next pass, robot holds and faces the object | ✅ tested | ⬜ |
| Level 3 when picked up, follow at 0.7 m with lamp ON | ✅ tested | ⬜ |
| Lost track -> last position -> resume patrol | ✅ tested | ⬜ |
| REMOVED when the place is in view and empty | ✅ tested | ⬜ |
| Pause / resume / emergency stop / acknowledge | ✅ tested | ⬜ |
| Dashboard: tabs, map, registry, log, simulation panel | ✅ tested | ⬜ |
| Perception with a synthetic image (bearing, distance, TF to map) | ✅ tested | ⬜ |
| LiDAR distance with Pinky's 180°-turned `rplidar_link` | ✅ tested | ⬜ |
| Lamp `/set_lamp` and LCD `/set_emotion` calls on patrol / hold | ✅ tested (fake servers) | ⬜ |
| Low battery -> charger -> plug -> charged -> Start patrol resumes | ✅ tested | ⬜ |
| Low during follow: follow finishes, then charger. Critical during follow: stops at once | ✅ tested | ⬜ |
| Camera publisher on the robot (pinkylib / picamera2) | — | ⬜ |
| ArUco detection distance with printed markers (target >= 1.5 m) | — | ⬜ |
| Map position within 0.2 m of the real position | — | ⬜ |
| No false MOVING while patrolling past a static box | — | ⬜ |
| Emergency stop stops the real robot immediately | — | ⬜ |

Unit tests: `cd security_task_manager && python3 -m pytest test` (21 tests).
