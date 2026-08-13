# What it would cost this robot to reach one arbitrary point, scored the way
# Rrg::evaluateLocalNavigationPath scores the goal it is handed on
# move_base_simple/goal. Only the position of that goal is ever read, so a
# bare Point is the whole input.
geometry_msgs/Point target
bool reachable

# A graph vertex sits within local_navigation_reaching_radius of the target:
# the robot routes over the global graph and covers the rest directly.
uint8 MODE_DIRECT = 0
# Nothing that close, and the target is outside the local bounds or on an
# unknown voxel: the robot heads for an intermediate frontier first.
uint8 MODE_VIA_FRONTIER = 1
# Near enough that the local planner just drives at it, no global routing.
uint8 MODE_LOCAL = 2
uint8 mode

# Frontier the robot would route through; -1 unless mode is MODE_VIA_FRONTIER.
int32 via_frontier_id

# Straight hop onto the global graph plus the Dijkstra distance over it. This
# is the term validated against flown distance -- median 1.9% error.
float64 path_length
# Straight line from where the graph ends to the target. Unvalidated, and
# optimistic by construction: that stretch lies in unknown space, which the
# robot crosses by exploring rather than by flying straight.
float64 residual_distance
# The value the planner minimises in this mode. For MODE_VIA_FRONTIER the
# residual is weighted by (1 + residual/total), between 1x and 2x, exactly as
# the local navigation does; the other modes add it once.
float64 total_cost
