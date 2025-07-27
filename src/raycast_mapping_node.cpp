#include "raycast_mapping.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<LocalMapCreator>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
