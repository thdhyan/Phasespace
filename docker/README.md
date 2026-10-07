# PhaseSpace → RViz / G1

`phasespace_rviz.py` connects to the PhaseSpace server and publishes, in frame `phasespace`:

| Topic | Content |
|---|---|
| `/phasespace_leds` | one sphere + ID per visible LED, coloured per microdriver |
| `/phasespace_world` (latched) | floor plane, camera view cones and IDs, SMPL-X body per LED group |
| `/tf_static` | `phasespace → ps_cam_<id>` for every calibrated camera (optical axis = local −Z) |
| `/tf` | `phasespace → <microdriver name>` at the LED-group centroid (head centre); `phasespace → phasespace_<id>` for rigid trackers |
| `/phasespace/humans` (latched) | JSON in G1_sim's `/sim/humans` schema: `{"frame", "humans": [{"name", "x", "y", "z", "heading_deg", "n_leds"}]}` |

LED groups come from the active session profile (server default, or `--profile`): each microdriver in it is one person/object, named after the device (`dhyan-hat` → LEDs 0–7). Add a microdriver to the profile in the Configuration Manager and it shows up as a new group.

`phasespace` frame: origin is the alignment origin on the floor tape, Z up, metres; the floor is z = 0. PhaseSpace mm / Y-up is converted as (x, y, z) → (x, −z, y) / 1000.

## Run

```bash
xhost +local:docker
docker compose -f docker/docker-compose.yml up --build
```

Without Docker (host ROS 2): `python3 docker/phasespace_rviz.py` and `rviz2 -d docker/phasespace.rviz`.

The SMPL-X model is licensed: it is mounted read-only from `$SMPLX_DIR` (default `~/Projects/HOVER/body_models/smplx`), never copied into the image or the repo.

## G1 ground truth

The G1 planner reads humans from `/sim/humans`; on the real robot point it at PhaseSpace instead:

```bash
ros2 run ... --ros-args -r /sim/humans:=/phasespace/humans
```

Positions are in `phasespace`, not the robot's `World`/`map`: a static transform between the two (or a microdriver on the robot) is still needed. `heading_deg` is `null` until the group has a rigid-body tracker.
