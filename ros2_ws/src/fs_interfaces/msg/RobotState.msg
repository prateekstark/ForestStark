# Fleet-level state of one robot, published at 2 Hz.
std_msgs/Header header
string robot
string state              # idle | to_start | drive | plant | turn | to_tender | queue | service | return | safety_stop | wait_fix | done | fault
float64 x
float64 y
float64 yaw
uint32 plugs_left
float64 water_left_l
float64 soc
uint32 planted
uint32 skipped
string gnss               # rtk_fix | float | dead_reckoning
