# Planting head state, published at 10 Hz.
std_msgs/Header header
string robot
string phase               # idle | locate | lower | bore | pick | retract | place | close | water | seed | halfmoon | log | stow
float64 phase_elapsed_s
float64 auger_depth_m
float64 auger_torque_nm
float64 auger_power_w
float64 auger_thrust_n
uint32 plugs_left
uint32 seedballs_left
float64 water_left_l
uint32 planted
