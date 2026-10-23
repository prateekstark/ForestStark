# Static and quasi-static tip-over stability (force-angle measure, Papadopoulos & Rey 1996).
uint8 OK=0
uint8 WARN=1
uint8 CRITICAL=2
std_msgs/Header header
uint8 level
float64 margin_deg                     # smallest extra tilt before the net force passes over a support edge
string critical_edge                   # front | rear | left | right | front-left ...
float64 roll_deg
float64 pitch_deg
float64 mass_kg
geometry_msgs/Point cg                 # map frame
geometry_msgs/Point[] support_polygon  # convex hull of ground contacts, map frame
float64[] edge_margins_deg             # one per polygon edge, same order
