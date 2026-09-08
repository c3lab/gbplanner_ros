# A frontier vertex of the global graph, scored the way the global planner
# scores its candidates in Rrg::calculateGlobalPath.
int32 id
geometry_msgs/Point position
# Heading the gain was evaluated at (irrelevant for 360 deg sensors).
float64 yaw

# --- Raw volumetric gain at this vertex ---
# num_unknown_voxels is the number of currently unknown voxels the sensor
# would observe from here.
int32 num_unknown_voxels
int32 num_free_voxels
int32 num_occupied_voxels
# Weighted gain: num_unknown * unknown_voxel_gain
#              + num_free * free_voxel_gain
#              + num_occupied * occupied_voxel_gain
float64 gain

# --- Ranking ---
# Shortest-path distance from the robot to this frontier over the global
# graph, plus the straight hop from the robot to the graph.
float64 path_length
# Shortest-path distance from this frontier back to the graph root (home).
float64 path_length_to_home
# Score the frontiers are ranked by, descending:
#   gain * exp(-0.01 * path_length) * time_spare
# With select_closest_frontier the gain term drops out and only distance
# counts. time_spare is the slack left in the mission time budget, or 1 when
# the frontier does not fit in the budget at all.
float64 exp_gain
# False when the remaining mission time does not cover reaching this frontier
# (and returning home, if auto_homing_enable). The global planner discards
# these outright.
bool feasible
