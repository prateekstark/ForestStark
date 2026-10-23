# Output of the per-robot safety monitor (fs_autonomy safety_monitor), published at 10 Hz.
# The monitor also gates the robot's velocity commands: navigators publish cmd_vel_nav and only
# the monitor publishes cmd_vel.
uint8 CLEAR=0                    # full speed
uint8 SLOW=1                     # something ahead: speed scaled down
uint8 STOP=2                     # protective stop; resumes by itself once the zone has been clear for a while
uint8 ESTOP=3                    # latched: needs an operator release
std_msgs/Header header
string robot
uint8 level
string reason                    # person_near | person_in_path | person_crossing | obstacle | estop | geofence | watchdog | localization
float32 speed_scale              # 0..1 applied to the navigator's command
bool tool_hold                   # a person is inside the tool zone: auger and mechanisms hold
float32 nearest_person_m         # from the robot's footprint; -1 if none tracked
# What is blocking travel (base_link), for the navigator's yield-or-reroute decision
bool blocked
float32 blocker_x
float32 blocker_y
float32 blocker_r
bool blocker_moving              # it has moved since first seen: a person or an animal, not a stump
float32 blocked_s                # how long the same object has been blocking
bool free_left                   # a detour corridor on that side of travel is clear of tracked objects
bool free_right
geometry_msgs/Point[] people     # tracked moving objects, map frame (z = speed, m/s)
