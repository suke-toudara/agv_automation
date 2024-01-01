// // costmap_creator.hpp
// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>

// //nav2
// #include "nav2_costmap_2d/layer.hpp"
// #include "nav2_costmap_2d/layered_costmap.hpp"

// #include <grid_map_core/GridMap.hpp>
// #include <grid_map_core/iterators/GridMapIterator.hpp>
// #include <grid_map_msgs/msg/grid_map.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>

// #ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #else
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #endif


// class CostmapCreator : public rclcpp::Node {
// public:
//   CostmapCreator();

// private:
//   rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  
//   void scanCallback(const sensor_msgs::msg::LaserScan & scan_msg);
// };
