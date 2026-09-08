// A C++ client for /gbplanner, written for one question only: does the
// PlannerSrv response really contain subnormal quaternion x/y, or does only the
// Python client print them that way?
//
// It calls the service exactly as pci_general does and dumps every quaternion
// component as its IEEE-754 bit pattern, so a subnormal cannot hide behind a
// formatted decimal. Same output shape as probe_ros{1,2}.py's path_quat_bits.
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>

#include <planner_msgs/srv/planner_srv.hpp>
#include <rclcpp/rclcpp.hpp>

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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("suite_a_quat_client");
  auto client = node->create_client<planner_msgs::srv::PlannerSrv>("/gbplanner");
  if (!client->wait_for_service(std::chrono::seconds(60))) {
    std::fprintf(stderr, "quat_client: /gbplanner never appeared\n");
    return 2;
  }

  auto request = std::make_shared<planner_msgs::srv::PlannerSrv::Request>();
  request->header.frame_id = "world";
  request->header.stamp = node->now();
  request->bound_mode = 0;

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(300)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    std::fprintf(stderr, "quat_client: call failed\n");
    return 3;
  }
  auto response = future.get();

  std::printf("status = %d\n", response->status);
  std::printf("planning_bound_mode = %d\n", response->planning_bound_mode);
  std::printf("path_size = %zu\n", response->path.size());
  for (size_t i = 0; i < response->path.size(); ++i) {
    const auto& q = response->path[i].orientation;
    std::printf("path[%zu].position = %.17g %.17g %.17g\n", i,
                response->path[i].position.x, response->path[i].position.y,
                response->path[i].position.z);
    std::printf("path[%zu].orientation = %.17g %.17g %.17g %.17g\n", i,
                q.x, q.y, q.z, q.w);
    std::printf("path[%zu].orientation.bits = %016" PRIx64 " %016" PRIx64
                " %016" PRIx64 " %016" PRIx64 "\n", i,
                bits(q.x), bits(q.y), bits(q.z), bits(q.w));
    std::printf("path[%zu].orientation.class = %c%c%c%c\n", i,
                fp_class(q.x), fp_class(q.y), fp_class(q.z), fp_class(q.w));
  }
  rclcpp::shutdown();
  return 0;
}
