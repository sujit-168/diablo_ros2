#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MsgConvert : public rclcpp::Node
{
public:
  MsgConvert() : Node("msg_convert")
  {
    publisher_ =
        this->create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 10);  // 创建一个发布器来发布自定义消息
    // 创建一个订阅器来订阅cmd_vel消息
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MsgConvert::msgconvert_callback, this, std::placeholders::_1));
    int i = 0;

  private:
    void msgconvert_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      motion_msgs::msg::MotionCtrl custom_msg;
      if (!subscriber_.getSubscriber())
      {
        RCLCPP_INFO(this->get_logger(), "还未接收到cmd_vel,刑天保持站立模式并初始化");
        while (i < 10)
        {
          custom_msg.mode_mark = true;

          custom_msg.value.forward = 0.0;
          custom_msg.value.left = 0.0;
          custom_msg.value.up = 0.0;
          custom_msg.value.roll = 0.0;
          custom_msg.value.pitch = 0.0;
          custom_msg.value.leg_split = 0.0;

          custom_msg.mode.pitch_ctrl_mode = false;
          custom_msg.mode.roll_ctrl_mode = false;
          custom_msg.mode.height_ctrl_mode = false;
          custom_msg.mode.stand_mode = true;
          custom_msg.mode.jump_mode = false;
          custom_msg.mode.split_mode = false;
          i++;
          publisher_->publish(custom_msg);
        }

        custom_msg.mode_mark = false;

        custom_msg.value.forward = 0.0;
        custom_msg.value.left = 0.0;
        custom_msg.value.up = 1.0;
        custom_msg.value.roll = 0.0;
        custom_msg.value.pitch = 0.2;
        custom_msg.value.leg_split = 0.0;

        custom_msg.mode.pitch_ctrl_mode = false;
        custom_msg.mode.roll_ctrl_mode = false;
        custom_msg.mode.height_ctrl_mode = false;
        custom_msg.mode.stand_mode = true;
        custom_msg.mode.jump_mode = false;
        custom_msg.mode.split_mode = false;
        publisher_->publish(custom_msg);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "成功订阅到cmd_vel");
        custom_msg.mode_mark = false;

        custom_msg.value.forward = msg->linear.x;
        custom_msg.value.left = msg->angular.z;
        custom_msg.value.up = 1.0;
        custom_msg.value.roll = 0.0;
        custom_msg.value.pitch = 0.2;
        custom_msg.value.leg_split = 0.0;

        custom_msg.mode.pitch_ctrl_mode = false;
        custom_msg.mode.roll_ctrl_mode = false;
        custom_msg.mode.height_ctrl_mode = false;
        custom_msg.mode.stand_mode = true;
        custom_msg.mode.jump_mode = false;
        custom_msg.mode.split_mode = false;
        publisher_->publish(custom_msg);
      }
    }

    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  };
} int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgConvert>());
  rclcpp::shutdown();
  return 0;
}