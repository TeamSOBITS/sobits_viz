# sobits_viz_robots

The robot descriptors every viewer in this repository shares. A descriptor is a
[sobits_vla_tools](https://github.com/TeamSOBITS/sobits_vla_tools) `.robot.yaml`
naming the robot's joint groups, mobile base, cameras and topics; the copies
here add a `sensors.lidars` list, which the original schema has no place for
yet. Each copy carries a header saying when it was taken and how it diverges —
they do not track the originals, so check the two agree before trusting one.

| Robot | Descriptor |
| --- | --- |
| SOBIT HOME | [config/sobit_home/sobit_home.robot.yaml](config/sobit_home/sobit_home.robot.yaml) |
| SOBIT LIGHT | [config/sobit_light/sobit_light.robot.yaml](config/sobit_light/sobit_light.robot.yaml) |

A viewer resolves the path through the ament index, so nothing is duplicated:

```python
from ament_index_python.packages import get_package_share_directory
descriptor = f"{get_package_share_directory('sobits_viz_robots')}/config/{robot}/{robot}.robot.yaml"
```

## Adding a robot

Copy [config/template/template.robot.yaml](config/template/template.robot.yaml)
to `config/<robot>/<robot>.robot.yaml` and fill in every `<...>`, or drop in the
robot's own descriptor from `sobits_vla_common/robots/`. Then give each viewer
the robot's own settings file, for example `sobits_viz_rerun/config/<robot>/<robot>.yaml`.
