# Calibrating against the real machine and site

The simulator's numbers come from CAD, published soil constants, IMD normals and the literature.
Each field measurement below replaces one of those with a measured value.
`ros2 run fs_bringup calibrate` fits the measurement and writes the result to the calibration
directory: `$FS_CALIBRATION_DIR`, `~/.ros/fs_calibration` by default, or `/data/fs_calibration` on
the robot. The next launch of Gazebo, the autonomy stack or the physics audit picks it up. Each
fit has a test in `fs_bringup/test/test_calibration.py` that recovers known parameters from noisy
synthetic data.

After each step, rerun `ros2 run fs_bringup physics_audit`. If a check changes status, act on it.

## 1. Weigh the machine (mass and centre of mass)

**What:** each wheel, or each track segment, on its own scale pad. Use running order: tanks full,
plugs loaded, operator seated for Katar. The machine stands level with the tool stowed.

**CSV (`link,kg`):** one row per contact link named in the URDF. For example `wheel_fl_link,512`.
Pahadi needs `fl1_link` … `rr3_link`. For a track, put the pad under each road wheel, or split the
track's load evenly between its three road-wheel links.

```bash
ros2 run fs_bringup calibrate weights --robot jaal --csv jaal_scales.csv
```

**Result:** `jaal_mass.yaml` holds the mass scale and the horizontal CoM shift. `sim.apply_inertials`
scales every link's mass and inertia, and moves the base link's CoM so the machine's CoM matches the
scales. A second weighing composes with the first. Recalibrate after any change to the build or payload.

## 2. Soil constants (Bekker plate, shear ring)

**Plate-sinkage test:** bevameter plates of at least two widths, for example 10, 20 and 30 cm,
loaded in steps. Record pressure and sinkage, at least six points per plate, on the site's
undisturbed ground.
`b_m,z_m,p_kpa` → `n`, `kc`, `kphi`.

**Shear test:** a shear ring or annular bevameter at three or more normal pressures that span the
machine's ground pressures (10–60 kPa). Record shear stress against shear displacement.
`sigma_kpa,j_m,tau_kpa` → `c`, `phi`, `K`.

**Shear vane against moisture:** a hand vane and a TDR moisture probe, read at the same spot, across
the season or on irrigated and dry plots.
`moisture,tau_kpa` → reference moisture and wet softening.

```bash
ros2 run fs_bringup calibrate plate --soil sandy_loam --csv plate.csv
ros2 run fs_bringup calibrate shear --soil sandy_loam --csv ring.csv
ros2 run fs_bringup calibrate vane  --soil sandy_loam --csv vane.csv
```

**Result:** `soils.yaml`, soil by soil, keeps only what was measured. Every soil map written after
that carries the calibrated constants: the Gazebo systems, the autonomy stack and the audit all read
the map. Name the soil class the site belongs to, or calibrate each class you find.

## 3. Check traction (drawbar pull)

**What:** pull the machine against a load cell on a winch or tractor. Hold slips from about 5 % to
50 %, measured from wheel speed against ground speed (GNSS or a fifth wheel), on the site soil at a
known moisture. `slip,pull_n`.

```bash
ros2 run fs_bringup calibrate pull --robot jaal --soil sandy_loam --csv pull.csv
```

**Result:** a comparison, not a fit: the RMS error and measured/model pull. If the model is more than
15 % off, redo step 2 on that ground before trusting the gradeability row of the audit.

## 4. Survival (a year on)

**What:** a year after planting, walk the plant records (`plants.csv` from the run, or its geojson on
a phone) and mark each plant alive or dead. `robot,id,alive`.

```bash
ros2 run fs_bringup calibrate survival --plants ~/.ros/fs_logs/<run>/plants.csv --counts survey.csv
```

**Result:** `survival.yaml` stores a hazard scale per region. It is the factor on the model's drought,
waterlogging, sodicity and shock hazards that reproduces the counted survival, and later predictions
use it. You need about 100 plants in a region before the scale means much.

## On the robot

Set `FS_CALIBRATION_DIR` to a persistent directory on the robot, for example `/data/fs_calibration`.
Mount it into the robot container when that is set up, then copy in the files fitted on a laptop,
or run the calibrate commands on the robot itself.
