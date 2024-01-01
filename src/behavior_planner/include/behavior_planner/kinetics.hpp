#ifndef TWO_WHEEL_ROBOT_CONTROLLER_HPP_
#define TWO_WHEEL_ROBOT_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/point.hpp"
//#include "geometry_msgs/msg/orientation.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class RobotController : public rclcpp::Node 
{
public:
    RobotController();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist &  msg);
    void calculateNewPose(const geometry_msgs::msg::Twist &  msg, double dt);
    void publishOdometry();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot_pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_odometry_publisher_;
    rclcpp::Clock clock_;
    rclcpp::Time last_time_;
    geometry_msgs::msg::Pose current_pose_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    double wheel_separation_ = 0.5;  // Replace with the actual wheel separation of your robot
    double wheel_radius  = 0 ;
    
};

#endif  // TWO_WHEEL_ROBOT_CONTROLLER_HPP_
