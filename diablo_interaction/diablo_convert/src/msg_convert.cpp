#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MsgConvert : public rclcpp::Node
{
public:

  MsgConvert() : Node("msg_convert")
  {
    // 创建一个发布器来发布自定义消息
    motion_cmd_pub_ = this->create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 10);
    // 创建一个订阅器来订阅cmd_vel消息
    cmd_vel_sub_ = 
      this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MsgConvert::msgconvert_callback, this, std::placeholders::_1));
    teleop_cmd_sub = 
      this->create_subscription<motion_msgs::msg::MotionCtrl>("teleop_motion_cmd", 10, std::bind(&MsgConvert::teleop_motion_cmd_callback, this, std::placeholders::_1));
    //create a timer to publish the custom message every 30ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&MsgConvert::timer_callback, this));
    
    custom_msg.mode_mark = false;

    custom_msg.value.forward = 0.0;
    custom_msg.value.left = 0.0;
    custom_msg.value.up = 0.0;
    custom_msg.value.roll = 0.0;
    custom_msg.value.pitch = 0.0;
    custom_msg.value.leg_split = 0.0;

    custom_msg.mode.pitch_ctrl_mode = false;
    custom_msg.mode.roll_ctrl_mode = false;
    custom_msg.mode.height_ctrl_mode = false;
    custom_msg.mode.stand_mode = false;
    custom_msg.mode.jump_mode = false;
    custom_msg.mode.split_mode = false;

    last_custom_msg = custom_msg;
    i = 0;
  }

private:
  motion_msgs::msg::MotionCtrl custom_msg;
  motion_msgs::msg::MotionCtrl last_custom_msg;
  int i;

  void msgconvert_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // assign the value of the incoming msg to custom_msg
    custom_msg.value.forward = msg->linear.x;
    custom_msg.value.left = msg->angular.z;
  }

  void teleop_motion_cmd_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
  {
    last_custom_msg.mode_mark = custom_msg.mode_mark;
    custom_msg.mode_mark = msg->mode_mark;
    // if the custom_msg.mode_mark change from false to true, we want to keep the custom_msg.mode_mark true for 3 times
    if (custom_msg.mode_mark != last_custom_msg.mode_mark)
    {
      custom_msg.mode_mark = true;
      i++;
      if (i > 10)
      {
        custom_msg.mode_mark = false;
        i = 0;
      }
    }


    // // if the value of the custom_msg.mode is not equal to the last_custom_msg.mode
    // // we should assign the custom_msg.mode_mark to true and last for 5 times
    // if (custom_msg.mode.pitch_ctrl_mode != last_custom_msg.mode.pitch_ctrl_mode ||
    //     custom_msg.mode.roll_ctrl_mode != last_custom_msg.mode.roll_ctrl_mode ||
    //     custom_msg.mode.height_ctrl_mode != last_custom_msg.mode.height_ctrl_mode ||
    //     custom_msg.mode.stand_mode != last_custom_msg.mode.stand_mode ||
    //     custom_msg.mode.jump_mode != last_custom_msg.mode.jump_mode ||
    //     custom_msg.mode.split_mode != last_custom_msg.mode.split_mode)
    // {
    //   custom_msg.mode_mark = true;
    //   i = 0;
    // }
    // else
    // {
    //   i++;
    //   if (i > 5)
    //   {
    //     custom_msg.mode_mark = false;
    //   }
    // }
    
    // assign the incoming msg.mode to custom_msg.mode
    custom_msg.mode = msg->mode;
    custom_msg.value.up = msg->value.up;
    custom_msg.value.pitch = msg->value.pitch;
  }

  void timer_callback()
  {
    // publish the custom_msg
    motion_cmd_pub_->publish(custom_msg);
    
  }
  rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr motion_cmd_pub_;
  rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr teleop_cmd_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_; 
} ;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgConvert>());
  rclcpp::shutdown();
  return 0;
}