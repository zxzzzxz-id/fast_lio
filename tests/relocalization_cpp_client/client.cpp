#include <rclcpp/rclcpp.hpp>
#include <fast_lio/srv/relocalize.hpp>
#include <iostream>
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("external_cpp_service_check");
  using Service = fast_lio::srv::Relocalize;
  auto client = node->create_client<Service>("/reloc/relocalize");
  if (!client->wait_for_service(std::chrono::seconds(5))) return 2;
  auto request = std::make_shared<Service::Request>();
  request->mode = argc > 1 ? std::stoi(argv[1]) : Service::Request::ODOMETRY;
  request->x = 0.4; request->y = -0.2; request->z = 0.1; request->yaw_deg = 8.0;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(12)) !=
      rclcpp::FutureReturnCode::SUCCESS) return 3;
  auto result = future.get();
  std::cout << "success=" << result->success << " frames=" << result->accumulated_frames
            << " correspondences=" << result->correspondences << " rmse=" << result->rmse
            << " elapsed=" << result->elapsed_sec << std::endl;
  rclcpp::shutdown();
  return result->success ? 0 : 1;
}
