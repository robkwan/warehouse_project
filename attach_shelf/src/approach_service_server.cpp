#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For quaternion operations
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals; // Make sure this line is present

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer()
      : Node("approach_service_server"), tf_broadcaster_(this),
        tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
        active_tf_(false) {

    // this->declare_parameter<bool>("use_sim_time", true);

    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(this->get_logger(), "use_sim_time_: %d", use_sim_time_);

    // Create callback groups
    scan_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscription with callback group
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = scan_group_;
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, _1),
        scan_opts);

    // Publisher
    if (use_sim_time_) {
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 10);
    } else {
      cmd_pub_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    // Timer for TF publishing
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ApproachServiceServer::broadcast_transform, this),
        scan_group_);

    // Service with callback group
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handle_service, this, _1, _2),
        rmw_qos_profile_services_default, service_group_);

    elevator_pub_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    RCLCPP_INFO(this->get_logger(), "approach_service_server node is ready");
  }

private:
  // ROS Interfaces
  rclcpp::CallbackGroup::SharedPtr scan_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr push_timer_;
  // Internal state
  geometry_msgs::msg::TransformStamped cart_tf_;
  bool active_tf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
  bool use_sim_time_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!active_tf_)
      return;

    std::vector<int> leg_indices;
    for (size_t i = 0; i < msg->intensities.size(); ++i) {
      if (msg->intensities[i] > 1500.0) // Increase this intensity for real
                                        // robot from 1000 to 1200 or 1500?!
        leg_indices.push_back(i);
    }

    float x = 1.0, y = 0.0;

    if (leg_indices.size() >= 2) {
      // 1. Get data for the front point
      int index1 = leg_indices.front();
      // Optional but recommended: Check if indices are valid
      if (static_cast<size_t>(index1) >= msg->ranges.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Front index %d out of bounds (size %zu)", index1,
                     msg->ranges.size());
        // Handle error appropriately - maybe return or continue to next cycle
        return;
      }
      float range1 = msg->ranges[index1];
      float angle1 = msg->angle_min + index1 * msg->angle_increment;
      float x1 = range1 * std::cos(angle1);
      float y1 = range1 * std::sin(angle1);

      // 2. Get data for the back point
      int index2 = leg_indices.back();
      // Optional but recommended: Check if indices are valid
      if (static_cast<size_t>(index2) >= msg->ranges.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Back index %d out of bounds (size %zu)", index2,
                     msg->ranges.size());
        // Handle error appropriately - maybe return or continue to next cycle
        return;
      }
      float range2 = msg->ranges[index2];
      float angle2 = msg->angle_min + index2 * msg->angle_increment;
      float x2 = range2 * std::cos(angle2);
      float y2 = range2 * std::sin(angle2);

      // 3. Calculate the midpoint in Cartesian coordinates
      float mid_x = (x1 + x2) / 2.0f; // Use 2.0f for floating point division
      float mid_y = (y1 + y2) / 2.0f;

      // Log relevant information (optional)
      RCLCPP_INFO(this->get_logger(),
                  "Front Point (Idx %d): R=%.2f, A=%.2f, (x=%.2f, y=%.2f)",
                  index1, range1, angle1, x1, y1);
      RCLCPP_INFO(this->get_logger(),
                  "Back Point  (Idx %d): R=%.2f, A=%.2f, (x=%.2f, y=%.2f)",
                  index2, range2, angle2, x2, y2);
      RCLCPP_INFO(this->get_logger(), "Midpoint: (x=%.2f, y=%.2f)", mid_x,
                  mid_y);

      // Assign the calculated midpoint to your target variables
      x = mid_x;
      y = mid_y;
    }

    cart_tf_.header.stamp = this->get_clock()->now();
    cart_tf_.header.frame_id = "robot_front_laser_base_link";
    cart_tf_.child_frame_id = "cart_frame";
    cart_tf_.transform.translation.x = x;
    cart_tf_.transform.translation.y = y;
    cart_tf_.transform.translation.z = 0.0;
    cart_tf_.transform.rotation.x = 0.0;
    cart_tf_.transform.rotation.y = 0.0;
    cart_tf_.transform.rotation.z = 0.0;
    cart_tf_.transform.rotation.w = 1.0;
  }

  void broadcast_transform() {
    if (active_tf_) {
      // RCLCPP_INFO(this->get_logger(), "TF publish: parent='%s', child='%s'",
      //             cart_tf_.header.frame_id.c_str(),
      //             cart_tf_.child_frame_id.c_str());
      if (cart_tf_.header.frame_id == cart_tf_.child_frame_id) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid TF: parent and child are the same ('%s')",
                     cart_tf_.header.frame_id.c_str());
        return;
      }
      tf_broadcaster_.sendTransform(cart_tf_);

      cart_tf_.header.stamp = this->get_clock()->now();
      tf_broadcaster_.sendTransform(cart_tf_);
    }
  }

  void handle_service(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service triggered: attach_to_shelf = %s",
                request->attach_to_shelf ? "true" : "false");

    active_tf_ = true;
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    if (!request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(),
                  "attach_to_shelf is false — only broadcasting TF.");
      response->complete = true;
      active_tf_ = false; // optionally disable TF loop
      return;
    }

    // Wait for laser scan to fill ranges
    // rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Check if cart_frame was valid
    if (cart_tf_.transform.translation.x == 0.0 &&
        cart_tf_.transform.translation.y == 0.0) {
      RCLCPP_INFO(this->get_logger(),
                  "Not enough shelf legs detected. Failing service.");
      response->complete = false;
      active_tf_ = false;
      return;
    }
    // Wait for TF to be available
    /*while (rclcpp::ok()) {
      try {
        tf_buffer_.lookupTransform("robot_front_laser_base_link", "cart_frame",
                                   tf2::TimePointZero);
        break;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Waiting for TF: %s", ex.what());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }*/

    // Follow the TF
    rclcpp::Rate rate(10);
    geometry_msgs::msg::Twist move;

    while (rclcpp::ok()) {
      try {
        auto tf = tf_buffer_.lookupTransform("robot_front_laser_base_link",
                                             "cart_frame", tf2::TimePointZero);

        float dx = tf.transform.translation.x;
        float dy = tf.transform.translation.y;
        float dist = std::hypot(dx, dy);

        RCLCPP_INFO(this->get_logger(), "dx = %.2f, dy=%.2f, dist=%.2f", dx, dy,
                    dist);

        if ((dist < 0.2)) { // loosen from 0.1 to 0.2 for real robot?!
          RCLCPP_INFO(this->get_logger(), "Reached cart_frame.");
          // move.linear.x = 0;
          // move.angular.z = 0;
          // cmd_pub_->publish(move);

          break;
        }

        // move.linear.x = std::clamp(0.5f * dx, 0.02f, 0.08f);
        // move.angular.z = std::clamp(-0.5f * dy, -0.5f, 0.5f);
        move.linear.x = 0.15;
        move.angular.z = 0.0;
        cmd_pub_->publish(move);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
      }

      rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Aligning robot to face cart_frame...");
    /*
        while (rclcpp::ok()) {
          auto tf = tf_buffer_.lookupTransform("robot_front_laser_base_link",
                                               "cart_frame",
       tf2::TimePointZero); float dx = tf.transform.translation.x; float dy =
       tf.transform.translation.y;

          float angle_error = std::atan2(dy, dx); // radians
          // RCLCPP_INFO(this->get_logger(), "Angle error: %.2f rad",
          // angle_error);

          if (std::abs(angle_error) < 0.02) // ~1.2 degrees
            break;

          geometry_msgs::msg::Twist turn;
          turn.linear.x = 0.0;
          turn.angular.z = std::clamp(-2.0f * angle_error, -0.4f,
                                      0.4f); // negative sign to rotate
       correctly cmd_pub_->publish(turn);
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    */
    /*
        auto tf = tf_buffer_.lookupTransform("robot_front_laser_base_link",
                                             "cart_frame", tf2::TimePointZero);
        // Extract the current orientation of the base_link in the target_frame
        geometry_msgs::msg::Quaternion current_orientation_in_target =
            tf.transform.rotation;

        // Convert quaternion to Euler angles to get the current yaw
        tf2::Quaternion q_current;
        tf2::fromMsg(current_orientation_in_target, q_current);
        tf2::Matrix3x3 m(q_current);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        // Our desired yaw is 0.0, meaning aligned with the target frame's
        // orientation
        double desired_yaw = 0.0;
        double yaw_error = desired_yaw - current_yaw;

        auto start_time_r = this->now();
        double rotation_speed = 0.2; // Example rotation speed
        rclcpp::Duration desired_duration_r =
            rclcpp::Duration::from_seconds(std::abs(yaw_error) /
       rotation_speed);

        move.linear.x = 0.0;
        move.angular.z = (yaw_error > 0)
                             ? rotation_speed
                             : -rotation_speed; // Rotate in the correct
       direction

        while (this->now() - start_time_r < desired_duration_r) {
          cmd_pub_->publish(move);
          rclcpp::sleep_for(
              std::chrono::milliseconds(10)); // Small sleep to avoid tight
       looping
        }

    move.linear.z = 0.0;
    cmd_pub_->publish(move);
*/
    // Stop and push forward
    move.linear.x = 0.0;
    move.angular.z = 0.0;
    cmd_pub_->publish(move);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(this->get_logger(), "Final 30cm forward push...");

    auto start_time = this->now();
    rclcpp::Duration desired_duration = 9s; // 8 seconds

    move.linear.x = 0.1;
    move.angular.z = 0.0;

    while (this->now() - start_time < desired_duration) {
      cmd_pub_->publish(move);
      rclcpp::sleep_for(
          std::chrono::milliseconds(10)); // Small sleep to avoid tight looping
    }

    move.linear.x = 0.0;
    cmd_pub_->publish(move);

    std_msgs::msg::String msg;
    msg.data = "up";
    elevator_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published 'up' to /elevator_up");

    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Final approach complete.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ApproachServiceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
