# State of the certified safety layer: safety PLC, safety scanners, the drives' STO. Published on
# safety/hw by fs_hardware safety_bridge (from the PLC's diagnostic PDO) on a machine, or by the
# safety monitor's PLC model in simulation. Software reads it; it can never change what the PLC does.
std_msgs/Header header
bool estop                 # an emergency-stop circuit is open (machine buttons, radio e-stop or its link)
bool protective_stop       # a protective field is violated, or was within the reset time
bool warning               # a warning field is violated
bool tool_zone             # a person in the tool zone
bool tilt                  # inclinometer beyond its limit
bool sto_drive             # safe torque off on the travel drives
bool sto_tool              # safe torque off on the tool drives (auger spin and feed, boom)
uint8 field_set            # active scanner field set (0 = standstill)
string field_name          # e.g. +x_0.8
bool heartbeat_ok          # the PLC sees the software heartbeat (a non-safety input)
bool simulated             # a software model of the PLC (simulation)
