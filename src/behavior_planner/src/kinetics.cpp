#include "behavior_planner/kinetics.hpp"

RobotController::RobotController() : Node("robot_controller"),tf_broadcaster_(this)
{
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](const geometry_msgs::msg::Twist & twist){cmdVelCallback(twist);});
    robot_pose_publisher_ = create_publisher<geometry_msgs::msg::Pose>("robot_pose", 10);
    robot_odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("robot_odometry", 10);
}   

void RobotController::cmdVelCallback(const geometry_msgs::msg::Twist &  msg) {
    auto current_time = this->now();
    auto elapsed_time = current_time - last_time_;
    double dt = elapsed_time.seconds();
    calculateNewPose(msg, dt);
    robot_pose_publisher_->publish(current_pose_);
    publishOdometry();
    last_time_ = current_time;
}

void RobotController::calculateNewPose(const geometry_msgs::msg::Twist & msg, double dt) {
    current_pose_.position.x += msg.linear.x * cos(current_pose_.orientation.z) * dt;
    current_pose_.position.y += msg.linear.x * sin(current_pose_.orientation.z) * dt;
    current_pose_.orientation.z += msg.angular.z * dt;
    while (current_pose_.orientation.z > M_PI) {
        current_pose_.orientation.z -= 2 * M_PI;
    }
    while (current_pose_.orientation.z <= -M_PI) {
        current_pose_.orientation.z += 2 * M_PI;
    }
}


void RobotController::publishOdometry() {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = this->now();
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose = current_pose_;
    odometry.pose.covariance.fill(0.0); // Fill covariance matrix with zeros for simplicity
    // Publish odometry message
    robot_odometry_publisher_->publish(odometry);
    // Publish tf transform
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odometry.header;
    odom_tf.child_frame_id = odometry.child_frame_id;
    odom_tf.transform.translation.x = odometry.pose.pose.position.x;
    odom_tf.transform.translation.y = odometry.pose.pose.position.y;
    odom_tf.transform.translation.z = odometry.pose.pose.position.z;
    odom_tf.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf);
}

// RobotController::kinetics(float velocity){

// }
