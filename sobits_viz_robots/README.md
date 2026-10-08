# sobits_viz_robots

A stop-gap home for robot descriptors whose description package does not ship
one yet. Every viewer reads a robot through
[sobits_robot_descriptor](https://github.com/TeamSOBITS/sobits_robot_descriptor),
which looks for `<robot>_description/config/<robot>.robot.yaml` first and falls
back to this package's `config/` only when nothing else resolves.

| Robot | Descriptor |
| --- | --- |
| SOBIT LIGHT | [config/sobit_light/sobit_light.robot.yaml](config/sobit_light/sobit_light.robot.yaml) |

SOBIT LIGHT's file is schema v2 and moves to `sobit_light_description` once
that package ships its own; this package goes away with it. SOBIT HOME's lives
in `sobit_home_description/config/sobit_home.robot.yaml`.

## Adding a robot

Put the descriptor in the robot's own description package:

```sh
ros2 run sobits_robot_descriptor new_robot --robot-id <robot> -o <robot>_description/config
ros2 run sobits_robot_descriptor validate <robot>
```

Then give each viewer the robot's own settings file, for example
`sobits_viz_rerun/config/<robot>/<robot>.yaml`.
