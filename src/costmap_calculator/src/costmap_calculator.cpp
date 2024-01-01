// #include "costmap_calculator/costmap_calculator.hpp"

// CostmapCreator::CostmapCreator() : Node("costmap_creator") {
//   auto grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
//   auto costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
//   auto scan_sub = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, [this](const sensor_msgs::msg::LaserScan & scan){scanCallback(scan);});
// }

// void CostmapCreator::scanCallback(const sensor_msgs::msg::LaserScan & scan_msg) {
//   nav_msgs::msg::OccupancyGrid costmap_msg;
//   costmap_msg.header = scan_msg.header;
//   costmap_msg.info.width = 100; // Set the width of the costmap (adjust as needed)
//   costmap_msg.info.height = 100; // Set the height of the costmap (adjust as needed)
//   costmap_msg.info.resolution = 0.1; // Set the resolution of the costmap (adjust as needed)


//   grid_map_msgs::msg::GridMap grid_map;
  
//   // Assume a simple mapping for demonstration purposes
//   for (int i = 0; i < costmap_msg.info.width * costmap_msg.info.height; ++i) {
//     costmap_msg.data.push_back(0); // Set all cells to free
//   }

//   // For each laser beam, mark the corresponding cell as occupied
//   for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
//     double range = scan_msg.ranges[i];
//     double angle = scan_msg.angle_min + i * scan_msg.angle_increment;

//     if (range < scan_msg.range_max) {
//       int x = static_cast<int>((range * std::cos(angle)) / costmap_msg.info.resolution);
//       int y = static_cast<int>((range * std::sin(angle)) / costmap_msg.info.resolution);

//       // Mark the cell as occupied
//       if (x >= 0 && x < costmap_msg.info.width && y >= 0 && y < costmap_msg.info.height) {
//         costmap_msg.data[y * costmap_msg.info.width + x] = 100; // Occupied cell
//       }
//     }
//   }

//   // Publish
//   costmap_pub_->publish(costmap_msg);
//   grid_map_pub_->publish(grid_map);
// }


// /*cost_map関係*/

// void CostmapCalculatorComponent::initGridMap()
// {
//   grid_map_.setFrameId("base_link");
//   grid_map_.setGeometry(
//     grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_), resolution_,
//     grid_map::Position(0.0, 0.0));
// }

// void CostmapCalculatorComponent::publish()
// {
//   auto msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
//   msg->header.stamp = get_clock()->now();
//   grid_map_pub_->publish(std::move(msg));
// }



main(int argc, char **argv){
  return 0;
}