
#ifndef CostmapFilter_HPP_
#define CostmapFilter_HPP_

#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class CostmapFilter : public rclcpp::Node
{
public:
  CostmapFilter();
  virtual ~CostmapFilter();


private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  void gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr filter_map_pub_;
  filters::FilterChain<grid_map::GridMap> filterChain_;
  std::string filterChainParametersName_;
};

#endif  