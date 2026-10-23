# Audit record for one planting attempt. Published on /plants/records.
std_msgs/Header header     # frame map, stamp = time the plug was placed
uint32 id                  # per-robot sequence number
string robot
uint32 spot_id
string tool                # auger | pick | seedball | none
string outcome             # planted | seeded | skipped
string reason              # why a spot was skipped, or notes
geometry_msgs/Point intended
geometry_msgs/Point actual # tool tip when placing (ground truth in simulation)
float64 error_m            # horizontal distance actual - intended
float64 latitude
float64 longitude
string soil
float64 rock_depth_m
float64 hole_depth_m
float64 bore_energy_kj
float64 cycle_time_s
float64 water_l
string species
# Regional prescription (fs_autonomy.silviculture)
string region              # e.g. thar_arid
string microsite           # e.g. saline patch, waterlogged, rocky slope
string method              # what was done: pit, backfill, depth, catchment, mulch, water
string deviations          # where the machine departed from the prescription, and why
