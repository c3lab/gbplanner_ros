#!/usr/bin/env python3
"""Keep a prefixed /<ns>/tf_static alive for the ros1_bridge.

tf_remapper_cpp publishes the prefixed static set once, latched. ROS 1
latching serves that to any late subscriber, so on this master everything
works -- but it does NOT survive the bridge: parameter_bridge forwards the
message to ROS 2 with volatile durability and no history. Measured, on a late
ROS 2 subscriber:

    ros2 topic echo --qos-durability transient_local --once /<ns>/tf_static
    -> nothing

So the ROS 2 side has exactly one chance to catch it, at the instant the
bridge forwards it. Lose that DDS discovery race and the robot's static frames
are missing from the fleet graph for the rest of the session, with nothing
logged on either side; every lookup crossing a static link then fails with a
bare LookupException. Observed failing roughly one run in three, on a
different robot each time.

Republishing on a timer removes the race entirely: the bridge keeps forwarding,
so whenever the ROS 2 side is listening it converges within one period. Cheap,
since a static set is a handful of transforms.

Subscribing and publishing on the same topic is deliberate: this node picks up
the remapper's latched message the same way any other subscriber does, and
merging its own output back in is idempotent.
"""
import rospy
from tf2_msgs.msg import TFMessage


class TFStaticPump:
    def __init__(self):
        topic = rospy.get_param('~topic', '/tf_static')
        period = rospy.get_param('~period', 1.0)

        self._transforms = {}
        # latch so a subscriber arriving between ticks still gets the set at
        # once, matching how static tf behaves everywhere else on this master.
        self._pub = rospy.Publisher(topic, TFMessage, queue_size=10, latch=True)
        rospy.Subscriber(topic, TFMessage, self._on_tf_static, queue_size=100)
        rospy.Timer(rospy.Duration(period), self._republish)
        rospy.loginfo('tf_static_pump: republishing %s every %.1fs', topic, period)

    def _on_tf_static(self, msg):
        for t in msg.transforms:
            self._transforms[t.child_frame_id] = t

    def _republish(self, _event):
        if self._transforms:
            self._pub.publish(TFMessage(transforms=list(self._transforms.values())))


if __name__ == '__main__':
    rospy.init_node('tf_static_pump')
    TFStaticPump()
    rospy.spin()
