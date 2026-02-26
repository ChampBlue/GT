#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameter_result.hpp"
#include "rclcpp/node_interfaces/on_set_parameters_callback_handle.hpp"

#include "sensor_msgs/msg/lidar_scan.hpp"

class LidarFilter : public rclcpp::Node
{
  public:
    LidarFilter();
  
  private:
    void scan_cb(const sensor_msgs::msg::LidarScan::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter> & params
    );

    double min_distance_threshold_{1.0};
    std::string frame_id_{"laser"};

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};