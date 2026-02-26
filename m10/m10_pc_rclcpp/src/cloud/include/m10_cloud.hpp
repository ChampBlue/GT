#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class M10Cloud : public rclcpp::Node {
  public:
    explict M10Cloud();

  private:
    std::string ns_{""};
    std::string global_frame_{"map"};
    std::string target_base_frame_{"base_footprint"};
    std::string output_topic_{"virtual_obstacles"};
    double publish_hz_{10.0};
    double radius_{0.7};
    int num_points_{16};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf();
    std::vector<geometry_msgs::msg::Point32> circle_points_local() const;
    
    std::vector<geometry_msgs::msg::Point32> transform_points(
        const geometry_msgs::msg::TransformStamped & t,
        const std::vector<geometry_msgs::msg::Point32> & pts_local
    ) const;

    sensor_msgs::msg::PointCloud2 to_cloud(
        const std::vector<geometry_msgs::msg::Point32> & world_pts
    )const;

    void on_timer(void);
};
