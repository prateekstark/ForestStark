# Wheel/track-soil interaction from the Bekker-Wong terramechanics model.
std_msgs/Header header
string soil
string[] wheels
float64[] normal_load_n
float64[] sinkage_m
float64[] slip                 # longitudinal slip ratio, -1..1
float64[] traction_n           # longitudinal soil force on each wheel
float64[] lateral_n
float64 mean_slip
float64 lateral_utilisation    # lateral force / lateral shear capacity, 1 = sliding
float64 motion_resistance_n
bool immobilised
bool side_slipping
