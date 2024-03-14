#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include "turtlebot4_node_interfaces/srv/drive.hpp"

using namespace std::chrono_literals;

class TurtleBot4DriveNode : public rclcpp::Node
{
public:
  TurtleBot4DriveNode()
  : Node("turtlebot4_drive_node")
  {
    // Subscribe to the /interface_buttons topic
    this->interface_buttons_subscriber_ = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "/interface_buttons", 
      rclcpp::SensorDataQoS(), 
      std::bind(&TurtleBot4DriveNode::interface_buttons_callback, this, std::placeholders::_1)
    );
    this->drive_subscriber_ = this->create_subscription<irobot_create_msgs::action::DriveDistance_Goal>(
      "/robot_drive", 
      rclcpp::SensorDataQoS(),
      std::bind(&TurtleBot4DriveNode::drive_callback, this, std::placeholders::_1)
    );

    this->drive_ptr_= rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(this, "/drive_distance");
    this->drive_srv_ = this->create_service<turtlebot4_node_interfaces::srv::Drive>(
      "/robot_drive_srv", 
      std::bind(&TurtleBot4DriveNode::drive_srv_func, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void interface_buttons_callback(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
  { 
    if(create3_buttons_msg->button_2.is_pressed){
      RCLCPP_INFO(this->get_logger(), "Button 2 pressed - cancelling all goals");
      this->drive_ptr_->async_cancel_all_goals();
    }
  }

  void drive_callback(const irobot_create_msgs::action::DriveDistance_Goal path_d)
  {
    // auto opts = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();

    // opts.goal_response_callback = std::bind(&TurtleBot4DriveNode::goal_response_callback, this, std::placeholders::_1);
    // // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    // opts.result_callback = std::bind(&TurtleBot4DriveNode::result_callback, this, std::placeholders::_1);

    // RCLCPP_INFO(this->get_logger(), "Sending goal");

    // this->drive_ptr_->async_send_goal(path_d, opts);
    
    RCLCPP_INFO(this->get_logger(), "Button 2 pressed - cancelling all goals");
    this->drive_ptr_->async_cancel_all_goals();

  }

  void drive_srv_func(
    const std::shared_ptr<turtlebot4_node_interfaces::srv::Drive::Request> request,
    std::shared_ptr<turtlebot4_node_interfaces::srv::Drive::Response> response)
  {
    response->success = true;
    irobot_create_msgs::action::DriveDistance_Goal path_d;
    
    path_d.distance = request->distance;
    auto opts = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();

    opts.goal_response_callback = std::bind(&TurtleBot4DriveNode::goal_response_callback, this, std::placeholders::_1);
    // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback = std::bind(&TurtleBot4DriveNode::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending drive goal with value");

    auto result = this->drive_ptr_->async_send_goal(path_d, opts);

    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
    //     != rclcpp::FutureReturnCode::SUCCESS)
    //   {
    //     RCLCPP_ERROR(this->get_logger(), "Failed");
    //   }

    RCLCPP_INFO(this->get_logger(), "sending back response: true");

  }
  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;
  rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr drive_ptr_;
  rclcpp::Service<turtlebot4_node_interfaces::srv::Drive>::SharedPtr drive_srv_;
  rclcpp::Subscription<irobot_create_msgs::action::DriveDistance_Goal>::SharedPtr drive_subscriber_;

  void result_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Drive Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Drive Goal was canceled");
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

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Drive Goal was rejected by server");
    } else {
      std::stringstream ss;
      ss << "Drive Goal accepted by server";
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr, const std::shared_ptr<irobot_create_msgs::action::DriveDistance_Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Remaning Distance Travel: " << feedback->remaining_travel_distance;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4DriveNode>());
  rclcpp::shutdown();
  return 0;
}
