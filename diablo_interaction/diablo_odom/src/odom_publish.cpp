#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <time.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

class OdomPublisherNode : public rclcpp::Node
{
public:
  OdomPublisherNode()
    : Node("odom_publisher")
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , wheel_radius(0.09)
    , wheel_separation(0.488)
    , timestamp(this->get_clock()->now())
  {
    //订阅电机信息中的左右轮速度
    motor_subscriber_ = this->create_subscription<motion_msgs::msg::LegMotors>(
        "/diablo/sensor/Motors", 10, std::bind(&OdomPublisherNode::legmotors_callback, this, _1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/diablo_odom", 10);
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(100),timer_callback);
  }

private:
  void legmotors_callback(const motion_msgs::msg::LegMotors::SharedPtr msg)
  {
    rclcpp::Time time = this->get_clock()->now();
    const double dt = time.seconds() - timestamp.seconds();
    const double left_vel = msg->left_wheel_vel * wheel_radius;
    const double right_vel = msg->right_wheel_vel * wheel_radius;

    const double v = (left_vel + right_vel) * 0.5;
    const double w = (right_vel - left_vel) / wheel_separation;
    theta += w * dt;
    x += v * dt * cos(theta);
    y += v * dt * sin(theta);

    auto odom_msg = nav_msgs::msg::Odometry();

    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;
    odom_publisher_->publish(odom_msg);
    timestamp = time;
  }

  rclcpp::Subscription<motion_msgs::msg::LegMotors>::SharedPtr motor_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x, y, theta;
  double wheel_radius, wheel_separation;
  rclcpp::Time timestamp;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
