#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include "turtlebot4_node_interfaces/srv/rotate.hpp"

using namespace std::chrono_literals;

class TurtleBot4RotateNode : public rclcpp::Node
{
public:
  TurtleBot4RotateNode()
  : Node("turtlebot4_rotate_node")
  {
    // Subscribe to the /interface_buttons topic
    this->interface_buttons_subscriber_ = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "/interface_buttons", 
      rclcpp::SensorDataQoS(), 
      std::bind(&TurtleBot4RotateNode::interface_buttons_callback, this, std::placeholders::_1)
    );
    this->rotate_subscriber_ = this->create_subscription<irobot_create_msgs::action::RotateAngle_Goal>(
      "/robot_rotate", 
      rclcpp::SensorDataQoS(),
      std::bind(&TurtleBot4RotateNode::rotate_callback, this, std::placeholders::_1)
    );

    this->rotate_ptr_= rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(this, "/rotate_angle");
    this->rotate_srv_ = this->create_service<turtlebot4_node_interfaces::srv::Rotate>(
      "/robot_rotate_srv", 
      std::bind(&TurtleBot4RotateNode::rotate_srv_func, this, std::placeholders::_1, std::placeholders::_2));
  
  }

private:
  void interface_buttons_callback(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
  { 
    if(create3_buttons_msg->button_2.is_pressed){
      RCLCPP_INFO(this->get_logger(), "Button 2 pressed - cancelling all goals");
      this->rotate_ptr_->async_cancel_all_goals();
    }
  }

  void rotate_callback(const irobot_create_msgs::action::RotateAngle_Goal path_d)
  {
    // auto opts = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

    // opts.goal_response_callback = std::bind(&TurtleBot4RotateNode::goal_response_callback, this, std::placeholders::_1);
    // // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    // opts.result_callback = std::bind(&TurtleBot4RotateNode::result_callback, this, std::placeholders::_1);

    // RCLCPP_INFO(this->get_logger(), "Sending goal");

    // this->rotate_ptr_->async_send_goal(path_d, opts);

    RCLCPP_INFO(this->get_logger(), "Button 2 pressed - cancelling all goals");
    this->rotate_ptr_->async_cancel_all_goals();

  }

  void rotate_srv_func(
    const std::shared_ptr<turtlebot4_node_interfaces::srv::Rotate::Request> request,
    std::shared_ptr<turtlebot4_node_interfaces::srv::Rotate::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Recieved Rotate Goal"); 
    response->success = true;
    irobot_create_msgs::action::RotateAngle_Goal path_r;
    
    path_r.angle = request->angle;
    auto opts = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

    opts.goal_response_callback = std::bind(&TurtleBot4RotateNode::goal_response_callback, this, std::placeholders::_1);
    // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback = std::bind(&TurtleBot4RotateNode::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending rotate goal with value");

    auto result = this->rotate_ptr_->async_send_goal(path_r, opts);

    //  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
    //     != rclcpp::FutureReturnCode::SUCCESS)
    //   {
    //     RCLCPP_ERROR(this->get_logger(), "Failed");
    //   }

    RCLCPP_INFO(this->get_logger(), "sending back response: true");

  }
  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;
  rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotate_ptr_;
  rclcpp::Service<turtlebot4_node_interfaces::srv::Rotate>::SharedPtr rotate_srv_;
  rclcpp::Subscription<irobot_create_msgs::action::RotateAngle_Goal>::SharedPtr rotate_subscriber_;

  void result_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Rotate Goal was aborted");
        
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Rotate Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;
    ss << "Completed goal with results: \nResults" << "\n\tOrientation \n\t\tx: " << result.result->pose.pose.orientation.x << " y: " << result.result->pose.pose.orientation.y
    << " z: " << result.result->pose.pose.orientation.z << " w: " << result.result->pose.pose.orientation.w;
    ss << "\n\tPosition \n\t\tx: " << result.result->pose.pose.position.x << " y: " << result.result->pose.pose.position.y << " z: " << result.result->pose.pose.position.z;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Rotate Goal was rejected by server");
    } else {
      std::stringstream ss;
      ss << "Rotate Goal accepted by server";
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr, const std::shared_ptr<irobot_create_msgs::action::RotateAngle_Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Remaning Angle Travel: " << feedback->remaining_angle_travel;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4RotateNode>());
  rclcpp::shutdown();
  return 0;
}
