#include "lidar_filter.hpp"

#include <functional>   //std::bind , std::placeholders
#include <utility>      //std::move
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/sensor_data_qos.hpp"

LidarFilter::LidarFilter() : Node("lidar_filter") {
    min_distance_threshold_ = this->declare_parameter<double>("min_distance_threshold", 1.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "laser");
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LidarFilter::on_param_change, this, std::placeholders::_1)
    );

    auto qos = rclcpp::SensorDataQoS();

    filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", qos);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->scan_cb(std::move(msg));
        }
    );
}

rcl_interfaces::msg::SetParametersResult LidarFilter::on_param_change(const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
        if (p.get_name() == "min_distance_threshold") {
            if (p.get_name() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = "min distance threshold must be double";
                break;
            }
            min_distance_threshold_ = p.as_double();
            RCLCPP_INFO(this->get_logger(),
                        "min_distance_threshold -> %.3f", min_distance_threshold_);
        }
    }
    return result;
}

void LidarFilter::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<float> filtered;
    
    //reserve(n) : 용량을 n 만큼 미리 확보 -> push_back 반복할 때 vector는 공간이 부족하면 더 큰 메모리를 새로 잡고 데이터 복사 이동
    //              -> 라이다처럼 원소 개수가 많은 경우 성능에 영향을 주기 떄문에 미리 그 공간을 잡아두는 최적화가 reserve()
    filtered.reserve(msg->ranges.size());

    for (const float r: msg->ranges) {
        filtered.push_back((r < min_distance_threshold_) ? 0.0f : r);
    }

    //move() : filtered의 내부 메모리를 복사하지 말고 통째로 msg->ranges로 넘겨라(소유권 이동) -> 즉 큰 라이다 배열을 복사하지 않고 바꿔치기하는거라 성능에 좋음.
    msg->ranges = std::move(filtered);
    msg->header.frame_id = frame_id_;

    filtered_scan_pub_->publish(*msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<LidarFilter> node = std::make_shared<LidarFilter();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
