# Maps

| Map | Area | Made with |
|---|---|---|
| `police_map.yaml` + `.pgm` | Demo arena, about 1.7 m x 4 m (x -0.2 .. 1.5, y -0.1 .. 4.0 in the map frame), 5 cm cells | SLAM on the real Pinky |

Keep the `.yaml` and `.pgm` together. After `colcon build` they are installed to
`share/security_bringup/maps/`, so Nav2 can load them with:

```bash
ros2 launch pinky_navigation bringup_launch.xml \
    map:=$(ros2 pkg prefix security_bringup)/share/security_bringup/maps/police_map.yaml
```

Waypoints, `charger_pose` and the dock in `config/patrol.yaml` are map-frame
coordinates, so they must be picked again whenever the map is rebuilt
(RViz *Publish Point*, then `ros2 topic echo /clicked_point --once`).
