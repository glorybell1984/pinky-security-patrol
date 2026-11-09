# Demo scenarios

1. Robot patrols its loop.
2. A box with marker 7 is placed on the route -> level 1, position logged, frame visible in RViz.
3. Next loop: box still there -> level 2, robot holds, lamp blinks, UI alert.
4. Someone picks up the box -> level 3, robot follows at about 0.7 m.
5. Marker hidden -> after 5 s robot goes to last position, reports LOST_TRACK, resumes patrol.
6. Show the CSV log for marker 7.

## Test checklist

- [ ] ArUco detection distance measured (target >= 1.5 m)
- [ ] Map position within 0.2 m of the real position
- [ ] Same object not reported twice on one pass
- [ ] Level 2 only after another waypoint has been visited
- [ ] Follow keeps >= 0.5 m distance
- [ ] Stop button cancels follow immediately
