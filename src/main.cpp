#include "px4_adrc_ctrl/adrc_controller_node.hpp"

#include "rclcpp/rclcpp.hpp"

/**
 * @brief Entry point for the px4_adrc_controller node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Process exit code.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_adrc_ctrl::AdrcControllerNode>());
  rclcpp::shutdown();
  return 0;
}

