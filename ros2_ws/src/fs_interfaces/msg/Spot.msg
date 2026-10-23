# One planned planting spot, in the map frame (ENU, metres from the site origin).
uint32 id
geometry_msgs/Point position
uint32 row                 # row (Jaal) or contour line (Pahadi); 0 for a single roadside row
float64 station            # distance along the row or planting line, m
string tool                # auger | pick | seedball | auto
string note                # e.g. "shifted 0.4 m off a stone", "no-dig: culvert"
