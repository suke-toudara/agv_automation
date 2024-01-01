#include "rclcpp/rclcpp.hpp"
#include "behavior_planner/kinetics.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
