# The Go2 wrapper, and why it exists

`go2_with_odometry.xacro` includes `unitree_go2_description`'s robot unchanged
and adds one block back: gz's odometry publisher.

That plugin is theirs, not ours. It sits in `unitree_go2_gazebo.xacro` commented
out, with the author's own note beside it:

    <!--leave it uncomment and use the ekf files for localisation-->

Left off, nothing publishes `/model/go2/odometry_with_covariance`, so the bridge
entry that feeds `/odom/raw` carries nothing, robot_localization's EKF has no
translation to integrate, and the whole tree reports a robot standing still.

Measured with it off: the robot walked 3.6 m across `cave_box` while `/odom`
stayed at exactly zero and `tf odom -> base_link` read a constant 0.426 m of
height and nothing else. gbplanner then plans for a robot at the origin, and
every graph is the root vertex alone.

## Two deliberate differences from their commented block

**`publish_tf` is false.** The EKF chain already owns `odom -> base_link`, and a
second publisher of it would put a cycle in the tree: `odom -> base_link` from
gz against `base_link -> base_footprint -> odom` from the two EKFs.

**The odom topic is renamed** to `/model/go2/odometry_ground_truth`. `/odom`
belongs to the EKF's filtered output. What this plugin is here for is the
covariance topic it publishes alongside, whatever the odom topic is called,
because that is the one their bridge is already wired to.

## Why a wrapper rather than a patch

`unitree_go2_description` is baked into the image as a Tier A source. Their
launch takes the description path as an argument, so pointing it at this file is
the whole integration - nothing in their package is edited.

## Keep the comments here, not in the xacro

Their launch passes the xacro's output straight into a `robot_description`
parameter without wrapping it as a string, so launch tries to parse the XML as
YAML. A comment containing something YAML-shaped - a list, a colon, a quoted
phrase - fails the whole launch with "Unable to parse the value of parameter
robot_description as yaml". Measured, with the first draft of this text inside
the xacro.
