# phasespace_ros2

ROS 2 (Jazzy; Humble works too) package for the PhaseSpace system. `tracker` connects to the PhaseSpace server and publishes, in frame `phasespace`:

| Topic | Content |
|---|---|
| `/phasespace_leds` | one sphere + ID per visible LED, coloured per LED group |
| `/phasespace_world` (latched) | floor plane, camera view cones and IDs |
| `/phasespace/groups` (latched) | JSON `{"frame", "groups": {name: {"leds": [ids], "oriented": bool}}}` |
| `/tf_static` | `phasespace → ps_cam_<id>` for every calibrated camera (optical axis = local −Z) |
| `/tf` | `phasespace → <group>` group pose; `phasespace → <group>/led_<id>` every visible LED of a group; `phasespace → led_<id>` LEDs in no group; `phasespace → phasespace_<id>` rigid trackers |

LED groups come from the active session profile (server default, or `profile:=`): each microdriver in it is one group, named after the device. Current `Dhyan Hat` profile: `dhyan-hat` (LEDs 0–7), `dhyan-hat-2` (8–15), `dhyan-hat-3` (16–23).

SMPL-X humans and the G1 head live in `g1_phasespace` (thesis `g1_perception_ws`), which uses this repo as the submodule `src/Phasespace`.

## Run

```bash
colcon build --symlink-install --packages-up-to phasespace_ros2
source install/setup.bash
ros2 launch phasespace_ros2 phasespace.launch.py            # tracker + RViz
ros2 launch phasespace_ros2 phasespace.launch.py rviz:=false profile:="Dhyan Hat"
```

`Ros_1/` has a `COLCON_IGNORE` (catkin packages). `phasespace_ros2/owl.py` is a symlink to the OWL client `Ros_2/phasespace_node_ros2.py`.

## Orientation

Each LED group gets a full pose once it has a reference layout. Put the object in the orientation that should count as "facing +x, upright", hold it still, and capture into the source tree, then rebuild:

```bash
ros2 run phasespace_ros2 tracker --capture dhyan-hat --ref-dir <path to>/phasespace_ros2/trackers
colcon build --packages-select phasespace_ros2
```

This writes `trackers/dhyan-hat.json` (LED positions relative to their centroid). At runtime each frame is fitted to it (Kabsch on matched LED IDs, needs >= 3 visible), which rotates the group TF. With no reference or fewer than 3 LEDs the group TF is the plain centroid, unrotated.

`phasespace` frame: origin is the alignment origin on the floor tape, Z up, metres; the floor is z = 0. PhaseSpace mm / Y-up is converted as (x, y, z) → (x, −z, y) / 1000.

## Docker

```bash
xhost +local:docker
docker compose -f docker/docker-compose.yml up --build
```

Builds the package on `osrf/ros:jazzy-desktop`; `phasespace_ros2/trackers` is mounted so captures persist.
