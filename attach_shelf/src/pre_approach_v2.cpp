#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2()
      : Node("pre_approach_node"), stopped_(false), rotated_(false),
        service_called_(false) {
    // Declare and get parameters
    this->declare_parameter<double>("obstacle", -0.3);
    this->declare_parameter<double>("degrees", 90.0);
    this->declare_parameter<bool>("final_approach", true);

    obstacle_distance_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_double();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproachV2::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproachV2::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&PreApproachV2::controlLoop, this));

    service_client_ =
        this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float front_distance = msg->ranges[msg->ranges.size() / 2];

    // RCLCPP_INFO(this->get_logger(), "front_distance: %.2f", front_distance);
    // RCLCPP_INFO(this->get_logger(), "obstacle_distance_: %.2f",
    //             obstacle_distance_);

    if (front_distance < obstacle_distance_) {
      stopped_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert quaternion to Euler angle (yaw)
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    current_angle_ = yaw * 180.0 / M_PI; // Convert to degrees
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd_msg;

    // RCLCPP_INFO(this->get_logger(), "stoppd_: %d, rotated_: %d", stopped_,
    //             rotated_);

    if (!stopped_) {
      // Move forward
      cmd_msg.linear.x = 0.2;
      cmd_msg.angular.z = 0.0;
      cmd_pub_->publish(cmd_msg);
      // RCLCPP_INFO(this->get_logger(), "Forward only.");
    } else if (!rotated_) {
      double target_angle = degrees_;
      double normalized_target_angle =
          fmod(target_angle + 360.0, 360.0); // Normalize

      // Determine current angle in the range [0, 360)
      double normalized_current_angle = fmod(current_angle_ + 360.0, 360.0);
      double angle_difference =
          normalized_target_angle - normalized_current_angle;

      // Normalize angle difference to [-180, 180]
      if (angle_difference > 180) {
        angle_difference -= 360;
      } else if (angle_difference < -180) {
        angle_difference += 360;
      }

      // RCLCPP_INFO(this->get_logger(), "angle_difference : %.2f",
      //             angle_difference);
      cmd_msg.linear.x = 0.0;

      // Adjust angular velocity based on angle difference
      if (std::abs(angle_difference) > 0.75) {
        // Check if we need to keep rotating
        // Set a maximum speed and scale based on angle difference
        double max_speed = 9.0; // Maximum speed (radians per second)
        double speed_scale = std::min(std::abs(angle_difference) / 180.0,
                                      1.0); // Scale to [0, 1]
        cmd_msg.angular.z = speed_scale * max_speed *
                            (angle_difference > 0 ? 1 : -1); // Adaptive speed
      } else {
        cmd_msg.angular.z = 0.0; // Stop rotation
        rotated_ = true;         // Mark as rotated
        RCLCPP_INFO(this->get_logger(), "Rotation done.");
        service_called_ = false;
      }
      cmd_pub_->publish(cmd_msg);

    } else if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Call /approach_shelf service...");
      callApproachService();
    }
  }

  void callApproachService() {
    if (!service_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Service /approach_shelf not available.");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_;

    RCLCPP_INFO(this->get_logger(), "Calling /approach_shelf service...");

    auto future = service_client_->async_send_request(
        request,
        [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture
                   result) {
          if (result.get()->complete) {
            RCLCPP_INFO(this->get_logger(),
                        "Final approach completed successfully.");
          } else {
            RCLCPP_WARN(this->get_logger(), "Final approach failed.");
          }
        });

    service_called_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr service_client_;

  bool stopped_, rotated_, service_called_;

  double obstacle_distance_, degrees_;
  bool final_approach_;

  double current_angle_; // Current angle in degrees

  rclcpp::Time rotation_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration rotation_duration_{0, 0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachV2>());
  rclcpp::shutdown();
  return 0;
}
