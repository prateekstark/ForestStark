# Deploying on a machine

The autonomy stack that runs in the simulator is the one that runs on the robot. Gazebo is reached only
through the topics in `fs_bringup/sim.py` (`bridge_entries`), and on a machine the drivers serve the
same topics. [hardware_interface.md](hardware_interface.md) lists them for each robot, with the
joint ranges and limits from the URDF. It is generated, so it cannot drift from what the simulator tests:

```bash
ros2 run fs_bringup interface_contract --out docs/hardware_interface.md
```

## What runs where

| | Simulation (`<robot>.launch.py`) | Machine (`robot.launch.py`) |
|---|---|---|
| Plant, sensors, soil | Gazebo + fs_gazebo systems | the machine and its drivers |
| GNSS | `gnss_degrader` models RTK, float and dropouts | `gnss_degrader mode:=receiver` passes the receiver's fix through and reads its status |
| Safety monitor, localization, planting head, traction monitor | same nodes, `use_sim_time` true | same nodes, wall clock |
| Katar driver | simulated human (`katar_driver`) | the human driver, cued by the cab display |
| Jaal fleet manager | in the launch | `fleet_manager:=true` on the base-station PC |
| People, livestock | `pedestrians`, `livestock` test the safety layer | real ones: the safety layer is what was tested |
| Visual models (saplings, basins, mulch) | spawned in Gazebo | `spawn_models` false |

The machine-specific numbers (plug capacity, tank volume, pump rate, running mass) live in
`fs_bringup/config/robots.yaml`. The simulation reads the same file. The compose service mounts the
robot's own copy over it.

## Build and install

```bash
docker build -f docker/Dockerfile --target test .                       # build + every test + physics audit
docker build -f docker/Dockerfile --target robot -t foreststark:robot .
```

On the robot's computer (Ubuntu with Docker), copy this repository to `/opt/foreststark`. Then:

1. Copy `docker/.env.example` to `docker/.env` and set `ROBOT`, `NS` and `SITE`.
2. Put the surveyed site (site YAML and soil map, the same format the simulator generates) in `/data/site`.
3. Start the stack at boot:

```bash
sudo cp deploy/foreststark.service /etc/systemd/system/ && sudo systemctl enable --now foreststark
```

Planting records (geotagged, one per spot, with the method, deviations and species) are written to
`/data/fs_logs`. An operator laptop on the same network and `ROS_DOMAIN_ID` can run RViz with
`rviz:=true`, or the operator tablet node.

## Commissioning checklist

Do these on every new machine, in order, before it plants unattended.

1. **Interface.** Every topic in hardware_interface.md is present at its rate, with
   `ros2 topic hz /<ns>/...`. `joint_states` names match the URDF.
2. **Command timeout.** Stop publishing `cmd_vel`: the drive must stop within 0.5 s. Kill the safety
   monitor: the drive must stop, and the monitor must respawn and hold a protective stop until
   localization is back.
3. **E-stop and tool hold.** The hardware E-stop cuts drive and tool power independently of software.
   `hold` on `auger/cmd` stops spin and feed where they are; `resume` carries on.
4. **Joint limits.** Command each joint to the ends of its range in hardware_interface.md at
   reduced speed. Soft limits in the drivers sit inside the URDF limits.
5. **Physics against the audit** ([physics_audit.md](physics_audit.md)):
   - Weigh the machine at each wheel and compare with the audit's contact loads.
   - On a tilt table or a measured slope, do not go past the working slope.
   - Measure drawbar pull on the site soil and compare with the gradeability row.
   - If any of these is more than 15 % off, correct `cad/build.py`'s mass table, rebuild the inertials, and rerun the audit.
6. **Auger.** Bore in the site soil and compare feed rate and energy (`auger/status`) with the
   simulator's for the same soil and moisture. Check refusal at rock, and the wall check in dry sand.
7. **Safety scenarios.** Walk the same scenarios the simulator runs (`people:=all`): a person
   crossing, a person at the tool, one standing in the path, and an obstacle. The behaviour must match.
8. **Site.** Check the site survey (utilities, no-dig zones, RTK-degraded zones) on the ground
   before the first run.

## Releasing

CI (`.github/workflows/ci.yml`) builds the workspace and runs every test. It checks that the
physics audit has no FAIL and that the audit and hardware contract in `docs/` match the models. It
then builds the robot image. A model or URDF change that moves the physics needs both documents
regenerated in the same commit.
