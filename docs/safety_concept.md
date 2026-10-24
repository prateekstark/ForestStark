# Safety concept

This document sets out what has to be certified before a ForestStark robot works near people
without a person holding a stop button, and how the software was built to sit above that layer
rather than replace it. It is the starting point for the risk assessment that a notified body, or a
test house such as TÜV or CFMTTI Budni, will review. It is not a certificate.

## Standards

| Standard | Used for |
|---|---|
| ISO 12100 | Risk assessment and risk reduction (the method below) |
| ISO 18497 | Safety of highly automated agricultural machines: the autonomy-specific requirements, the test object, and obstacle detection |
| ISO 25119 / ISO 13849-1, -2 | Functional safety of the control system: required Performance Level (AgPL / PL), architecture, validation |
| ISO 13855 | Positioning protective devices: protective field length from stopping performance |
| IEC 61496-1, -3 | Electro-sensitive protective equipment: the safety laser scanners (type 3) |
| IEC 61800-5-2 | Drive safety functions: STO, SS1, SLS |
| ISO 13850 | Emergency stop |
| ISO 4254-1 | Agricultural machinery general safety (guards, hydraulics, PTO), which applies to all three machines |

## Layers

1. **Certified layer.** Safety laser scanners, a safety PLC, emergency-stop circuits, an
   inclinometer, and the safety functions in the drives (STO, SS1, SLS). It reaches PL d through a
   Category 3 architecture. It acts on its own safe inputs, and nothing in software can switch it off.
2. **Comfort layer.** The software safety monitor (`fs_autonomy/safety.py`, `safety_monitor`). It
   tracks people and objects, slows from 6 m, stops for a person in the path at 2.5–3 m, and holds
   the tool when someone comes near. It is non-safety, at most PL a. Its job is to keep the machine
   working smoothly so that layer 1 almost never has to act.
3. **Planning.** Navigators, geofence and survey zones. Also non-safety.

The comfort layer's stop distances are floored above the certified layer's protective field at every
speed (`SafetyConfig.chain`), so in normal operation the software always stops first. The simulator
runs the PLC logic on the same scans (`plc_model`, `fs_autonomy.safety_fields.PlcModel`) to show
this. On a machine, the PLC's state comes to ROS on `safety/hw` through `fs_hardware safety_bridge`.

## Hazards and safety functions

The required level comes from the ISO 13849-1 risk graph:
- **S2:** serious, irreversible injury.
- **F2:** frequent exposure for roadside and field crews; F1 for Pahadi on remote slopes.
- **P1:** possible to avoid at walking speed; P2 for the auger.

| # | Hazard | Machine | Safety function | Implementation | PLr |
|---|---|---|---|---|---|
| SF1 | Any hazard, on demand | all | Emergency stop, stop category 1 then 0 | E-stop buttons on every side, and a wireless e-stop whose loss of link also stops the machine. Dual-channel to the safety PLC, then STO on all drives, hydraulic dump valve, and engine stop on Katar's tool PTO | d |
| SF2 | Person run over or struck by the machine | Jaal, Pahadi | Protective field, then SS1 | Outdoor safety scanners (IEC 61496-3 type 3), front and rear on Jaal, front on Pahadi. Field sets switch on speed measured by the safety PLC from the drive encoders, sized per ISO 13855 ([safety_fields.md](safety_fields.md)) | d |
| SF3 | Entanglement or impact by the auger or boom | all | Tool zone, then STO of the tool drives | A scanner field 2 m around the tool while it turns or feeds. Katar's boom-side scanner. Spin and feed get STO; the boom holds on its load-holding valves | d |
| SF4 | Excess speed carries the machine past its fields | Jaal, Pahadi | Safely-limited speed | SLS in the drives at the top field band: 1.5 m/s for Jaal, 0.7 m/s for Pahadi | d |
| SF5 | Tip-over on a slope | Pahadi | Tilt limit, then stop and stow | A dual-channel inclinometer on the safety PLC: stop travel and the arm above 30°. The software stability monitor warns much earlier, from the support polygon | c |
| SF6 | Unexpected start after a stop | all | Restart interlock | Automatic restart only after the protective field has been clear for 1 s and only for travel. The tool restarts only on an operator acknowledgement after an SF1 or SF3 trip | c |
| SF7 | Leaving the work area | Jaal, Pahadi | Geofence | GNSS is not safety-rated, so this stays in the comfort layer (PL a). The physical boundary is the site plan and the crew's barrier tape | – |

**Katar.** A person drives the tractor, so travel safety is the driver's job. The certified layer
guards the tool (SF1, SF3), and an operator-presence switch in the cab enables the tool.

## Architecture (PL d, Category 3)

- **Safety PLC:** a dual-channel controller such as Pilz PNOZmulti 2 or Siemens S7-1200F. It has
  CANopen for diagnostics and talks to the drives over a safe fieldbus (FSoE or CANopen Safety,
  EN 50325-5), or over hard-wired STO.
- **Scanners:** outdoor-certified, with dust and rain filtering, for example SICK outdoorScan3. The
  field sets come from `safety_fields.yaml`, with polygons in each scanner's own frame.
- **Drives:** servo and track drives with integrated STO, SS1 and SLS (SIL 2 / PL d). Hydraulic
  axes use a safety-rated dump valve with position monitoring, and load-holding valves on the boom
  and arm.
- **Diagnostics to software:** non-safety, through the PLC's diagnostic PDO
  (`fs_hardware/safety_bridge.py`). Software sends back only a heartbeat and the requested mode
  (travel or planting), which choose field sets but cannot disable any function.

## Validation (ISO 13849-2, ISO 18497)

1. **Calculation:** PFHd, MTTFd, DC and CCF for each safety function, in SISTEMA or equivalent,
   using the chosen components' data.
2. **Stopping performance:** measure the stopping distance at each band's top speed, on the site's
   worst soil, wet and downhill. Then recompute the fields with the measured values in `StopChain`.
3. **Detection:** the ISO 18497 test object, plus a 200 mm child-size cylinder, lying and crouching
   persons, and animals, approached from every direction at every band. Repeat in dust, rain, low sun
   and with the crop or saplings at field edges.
4. **Fault injection:** open each E-stop channel, unplug each scanner, kill the PLC heartbeat,
   and power-cycle the drives. The machine must stop every time.
5. **Field trial with crews:** log every comfort-layer stop and every certified-layer trip
   (`safety/hw`). A certified trip in normal work is a finding: the comfort layer should have
   stopped first.

## Status

- **Done in this repository:**
  - the comfort layer;
  - the field sizing, field sets and PLC logic model;
  - the floor that keeps the comfort layer above the certified layer;
  - the simulation backstop;
  - the PLC diagnostic bridge and its message (`fs_interfaces/HwSafety`).
- **To do with hardware:**
  - component selection and wiring;
  - the PLC program;
  - the drives' safety parameters;
  - the validation above;
  - the technical file and conformity assessment.
