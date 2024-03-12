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
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std::chrono_literals;

class TurtleBot4Node : public rclcpp::Node
{
public:
// using GoalHandleDriveDistance = rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::D>;
// using GoalHandleRotateAngle = rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::RotateAngle>;
  TurtleBot4Node()
  : Node("turtlebot4_cpp_node")
  {
    // Subscribe to the /interface_buttons topic
    interface_buttons_subscriber_ = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>("/interface_buttons", rclcpp::SensorDataQoS(), std::bind(&TurtleBot4Node::interface_buttons_callback, this, std::placeholders::_1));
    //battery_state_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>("/battery_state", rclcpp::SensorDataQoS(), std::bind(&TurtleBot4Node::battery_state_callback, this, std::placeholders::_1));
    this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", rclcpp::SensorDataQoS());
    this->path_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS());
    this->drive_ptr_= rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(this, "/drive_distance");
    this->rotate_ptr_= rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(this, "/rotate_angle");
    this->nav_ptr_ = rclcpp_action::create_client<irobot_create_msgs::action::NavigateToPosition>(this, "/navigate_to_position");
  }

private:
  // Interface buttons subscription callback
  #define NUMPOINTS 7
  #define OFFSET 0.23 // meters
  bool wait_for_callback_rotate = false;
  float points[21][2] = {{0.9,0.27},{1,0.58},{0.855,0.70},{0.745,0.38},{0.36,0.293},{0.5,0.171},{0.905,0.265}};
  float pointst[21][2] = {{0,0},{0,1},{0.5,0.5},{0.5,0},{0,0}};

  void battery_state_callback(const sensor_msgs::msg::BatteryState bstate){
    std::stringstream ss;
    ss << "Battery Precentage: " ;//<< bstate.percentage * 100 << "%";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void interface_buttons_callback(
    const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
    { 
      if(create3_buttons_msg->button_1.is_pressed) {
        RCLCPP_INFO(this->get_logger(), "Button 1 pressed - starting path");
        path();
      }
      if(create3_buttons_msg->button_2.is_pressed){
        RCLCPP_INFO(this->get_logger(), "Button 2 pressed - cancelling all goals");
        this->drive_ptr_->async_cancel_all_goals();
        this->rotate_ptr_->async_cancel_all_goals();
        this->nav_ptr_->async_cancel_all_goals();
      }
    }
  void nav(){
    auto path_n = irobot_create_msgs::action::NavigateToPosition_Goal();
    // auto nav_opts = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();
    // drive_opts.result_callback = std::bind(&TurtleBot4Node::result_drive_callback, this, std::placeholders::_1);

    path_n.goal_pose.pose.orientation.x = 0;
    path_n.goal_pose.pose.orientation.y = 0;
    path_n.goal_pose.pose.orientation.z = 0;
    path_n.goal_pose.pose.orientation.w = 3.1415/2;
    path_n.goal_pose.pose.position.x = 1;
    path_n.goal_pose.pose.position.y = 0;
    path_n.goal_pose.pose.position.z = 0;

    RCLCPP_INFO(this->get_logger(), "bye goals");
    this->nav_ptr_->async_send_goal(path_n);

  }

  void path_action(){
    auto path_d = irobot_create_msgs::action::DriveDistance_Goal();
    auto drive_opts = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();
    drive_opts.result_callback = std::bind(&TurtleBot4Node::result_drive_callback, this, std::placeholders::_1);
    auto path_r = irobot_create_msgs::action::RotateAngle_Goal();
    auto rotate_opts = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();
    rotate_opts.result_callback = std::bind(&TurtleBot4Node::result_angle_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goals");

    path_r.angle = 3.1415/2;
    path_r.max_rotation_speed = 1.5;
    if(this->rotate_ptr_->action_server_is_ready()){  
      auto p = this->rotate_ptr_->async_send_goal(path_r, rotate_opts);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Rotate action server not ready");
      while(this->rotate_ptr_->wait_for_action_server(10s)){
        RCLCPP_INFO(this->get_logger(), "Waiting for action server");
        sleep(0.5);
      }
    }
    
    path_d.distance = 1;
    this->drive_ptr_->async_send_goal(path_d, drive_opts);
  }
  
  void path(){

    // now the path function is going to generate a path based on the points, 

    //take the first point, and assume that's the current position
    float current_pos[2];
    current_pos[0] = pointst[0][0];
    current_pos[1] = pointst[0][1];

    float current_dir[2] = {0,1};

    auto path_d = irobot_create_msgs::action::DriveDistance_Goal();
    auto drive_opts = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();
    auto path_r = irobot_create_msgs::action::RotateAngle_Goal();
    auto rotate_opts = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

    rotate_opts.goal_response_callback = std::bind(&TurtleBot4Node::goal_angle_response_callback, this, std::placeholders::_1);
    // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    rotate_opts.result_callback = std::bind(&TurtleBot4Node::result_angle_callback, this, std::placeholders::_1);

    drive_opts.goal_response_callback = std::bind(&TurtleBot4Node::goal_drive_response_callback, this, std::placeholders::_1);
    // rotate_opts.feedback_callback = std::bind(&TurtleBot4Node::feedback_angle_callback, this, std::placeholders::_1, std::placeholders::_2);
    drive_opts.result_callback = std::bind(&TurtleBot4Node::result_drive_callback, this, std::placeholders::_1);

    auto path_n = irobot_create_msgs::action::NavigateToPosition_Goal();
    RCLCPP_INFO(this->get_logger(), "Sending goals");
 

    //then for each subsequent point, calculate the offset from the previous point to get relative vector direction

    // then set it to be the new current position.

    for(int i = 1; i < NUMPOINTS; i++){
     
      float next_pos[2];
      next_pos[0] = pointst[i][0];
      next_pos[1] = pointst[i][1];

      std::stringstream log1;
      log1 << "From: " << current_pos[0] << ", " << current_pos[1] << " to " << next_pos[0] << ", " << next_pos[1];
      RCLCPP_INFO(this->get_logger(), log1.str().c_str());

      // the displacement between the current position and the next position
      float delta_x = next_pos[0] - current_pos[0];
      float delta_y = next_pos[1] - current_pos[1];

      // the magnitude of the distance between the current position and the next position
      float mag = std::sqrt(delta_x * delta_x + delta_y * delta_y);
      float delta_xn = delta_x/mag;
      float delta_yn = delta_y/mag;

      float ang = std::acos(current_dir[0]*delta_xn + current_dir[1]*delta_yn);

      float x = current_dir[0] * std::cos(ang) - current_dir[1] * std::sin(ang);//  cos θ − y sin θ
      float y = current_dir[0] * std::sin(ang) + current_dir[1] * std::cos(ang);//  cos θ + y sin θ

      float error = 0.001;

      if(x > delta_xn+error || x<delta_xn-error || y>delta_yn+error || y < delta_yn-error){
        x = current_dir[0] * std::cos(-ang) - current_dir[1] * std::sin(-ang);
        y = current_dir[0] * std::sin(-ang) + current_dir[1] * std::cos(-ang);
        RCLCPP_INFO(this->get_logger(), "Flipped Angle");
        ang = -ang;
      }

      //construct the Twist based on the new information
      //do we need to set the empty values? 
      std::stringstream log;
      log << "published path:" << "\n\tangle: " << (ang*180/3.1415) << " mag: " << mag << " new x: " << x << " new y:" << y 
      << " dx:" << delta_x << " dxn:" << delta_xn << " dy:" << delta_y << " dyn:"<< delta_yn;

      path_r.angle = ang;
      path_d.distance = mag;

      // path_n.goal_pose.pose.orientation.x = 0;

      // this->nav_ptr_->async_send_goal(path_n);
      if(this->rotate_ptr_->action_server_is_ready()){
        this->rotate_ptr_->async_send_goal(path_r, rotate_opts);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Rotate action server not ready");
      }

      rclcpp::sleep_for(6s);

      if(this->rotate_ptr_->action_server_is_ready()){
        this->drive_ptr_->async_send_goal(path_d, drive_opts);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Drive action server not ready");
      }

      //rotate -> drive -> rotate -> drive for offset
      // float something = next_pos 



      RCLCPP_INFO(this->get_logger(), log.str().c_str());
      
      current_pos[0] = next_pos[0];
      current_pos[1] = next_pos[1];
      current_dir[0] = delta_xn;
      current_dir[1] = delta_yn;

      rclcpp::sleep_for(2s);
    }

  }

  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr path_publisher_;
  rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr drive_ptr_;
  rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotate_ptr_; 
  rclcpp_action::Client<irobot_create_msgs::action::NavigateToPosition>::SharedPtr nav_ptr_; 
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  void result_angle_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & result ){//irobot_create_msgs::action::DriveDistance_Goal drive){
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
    result.result->pose.header.frame_id = "odom_angle";
    this->pose_publisher_->publish(result.result->pose);  
  }

  void goal_angle_response_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Rotate Goal was rejected by server");
    } else {
      std::stringstream ss;
      ss << "Rotate Goal accepted by server";
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
  }

  void feedback_angle_callback(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr, const std::shared_ptr<irobot_create_msgs::action::RotateAngle_Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Remaning Angle Travel: " << feedback->remaining_angle_travel;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_drive_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::WrappedResult & result){
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
    result.result->pose.header.frame_id = "odom_drive";
    this->pose_publisher_->publish(result.result->pose);
  }
  void goal_drive_response_callback(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "DriveGoal was rejected by server");
    } else {
      std::stringstream ss;
      ss << "Drive Goal accepted by server";
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
  }

  void feedback_drivecallback(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr, const std::shared_ptr<irobot_create_msgs::action::DriveDistance_Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Remaning Distance Travel: " << feedback->remaining_travel_distance;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4Node>());
  rclcpp::shutdown();
  return 0;
}
