# Ordered work list for one robot.
std_msgs/Header header
string robot
string pattern             # roadside_row | square_grid | triangular_grid | contour
float64 spacing_m
float64 row_spacing_m
Spot[] spots               # in planting order
