// The ROS 1 twin of quat_client.cpp. Same question, same output shape: is the
// subnormal quaternion x/y in the response real, or an artefact of the client
// that printed it? Reading the same service from C++ and from Python on both
// middlewares is the only way to tell those two apart.
//
// Built with plain g++ against /opt/ros/noetic plus the workspace devel space -
// no catkin package, because nothing may be written into the ROS 1 repository.
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <planner_msgs/planner_srv.h>
#include <ros/ros.h>

namespace {

uint64_t bits(double value) {
  uint64_t out;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

char fp_class(double value) {
  switch (std::fpclassify(value)) {
    case FP_NAN: return 'A';
    case FP_INFINITE: return 'I';
    case FP_ZERO: return 'Z';
    case FP_SUBNORMAL: return 'S';
    default: return 'N';
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "suite_a_quat_client_ros1");
  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<planner_msgs::planner_srv>("/gbplanner");
  if (!client.waitForExistence(ros::Duration(60.0))) {
    std::fprintf(stderr, "quat_client_ros1: /gbplanner never appeared\n");
    return 2;
  }

  planner_msgs::planner_srv srv;
  srv.request.header.frame_id = "world";
  srv.request.header.stamp = ros::Time::now();
  srv.request.bound_mode = 0;
  if (!client.call(srv)) {
    std::fprintf(stderr, "quat_client_ros1: call failed\n");
    return 3;
  }

  std::printf("status = %d\n", srv.response.status);
  std::printf("planning_bound_mode = %d\n", srv.response.planning_bound_mode);
  std::printf("path_size = %zu\n", srv.response.path.size());
  for (size_t i = 0; i < srv.response.path.size(); ++i) {
    const geometry_msgs::Quaternion& q = srv.response.path[i].orientation;
    std::printf("path[%zu].position = %.17g %.17g %.17g\n", i,
                srv.response.path[i].position.x, srv.response.path[i].position.y,
                srv.response.path[i].position.z);
    std::printf("path[%zu].orientation = %.17g %.17g %.17g %.17g\n", i,
                q.x, q.y, q.z, q.w);
    std::printf("path[%zu].orientation.bits = %016" PRIx64 " %016" PRIx64
                " %016" PRIx64 " %016" PRIx64 "\n", i,
                bits(q.x), bits(q.y), bits(q.z), bits(q.w));
    std::printf("path[%zu].orientation.class = %c%c%c%c\n", i,
                fp_class(q.x), fp_class(q.y), fp_class(q.z), fp_class(q.w));
  }
  return 0;
}
