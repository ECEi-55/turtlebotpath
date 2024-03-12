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
#include "turtlebot4_node_interfaces/srv/drive.hpp"
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

class TurtleBot4Node : public rclcpp::Node
{
public:
// using GoalHandleDriveDistance = rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::D>;
// using GoalHandleRotateAngle = rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::RotateAngle>;
  TurtleBot4Node()
  : Node("turtlebot4_goal_node")
  {
    // Subscribe to the /interface_buttons topic
    interface_buttons_subscriber_ = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>("/interface_buttons", rclcpp::SensorDataQoS(), std::bind(&TurtleBot4Node::interface_buttons_callback, this, std::placeholders::_1));
    this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", rclcpp::SensorDataQoS());
    this->drive_publisher_ = this->create_publisher<irobot_create_msgs::action::DriveDistance_Goal>("/robot_drive", rclcpp::SensorDataQoS());
    this->rotate_publisher_ = this->create_publisher<irobot_create_msgs::action::RotateAngle_Goal>("/robot_rotate", rclcpp::SensorDataQoS());
    this->drive_client_ = this->create_client<turtlebot4_node_interfaces::srv::Drive>("/robot_drive_srv");
  }
private:
  // Interface buttons subscription callback
  #define NUMPOINTS 7
  #define OFFSET 0.23 // meters

  float points[21][2] = {{0.9,0.27},{1,0.58},{0.855,0.70},{0.745,0.38},{0.36,0.293},{0.5,0.171},{0.905,0.265}};
  float pointst[21][2] = {{0,0},{0,1},{0.5,0.5},{0.5,0},{0,0}};

  void interface_buttons_callback(
    const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
    { 
      if(create3_buttons_msg->button_1.is_pressed) {
        RCLCPP_INFO(this->get_logger(), "Button 1 pressed - starting path");
        path_action();
      }
    }


  void path_action(){
    // auto path_d = irobot_create_msgs::action::DriveDistance_Goal();
    // auto path_r = irobot_create_msgs::action::RotateAngle_Goal();
 
    // path_d.distance = 1;
    // this->drive_publisher_->publish(path_d);

    // path_r.angle = 3.1415/2;;
    // this->rotate_publisher_->publish(path_r);

    auto dist = std::make_shared<turtlebot4_node_interfaces::srv::Drive::Request>();
    dist->distance = 1;


    while(!this->drive_client_->wait_for_service(1s)){
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return ;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "send rotate goal over");

  }
  
  void path(){

    // now the path function is going to generate a path based on the points, 

    //take the first point, and assume that's the current position
    float current_pos[2];
    current_pos[0] = pointst[0][0];
    current_pos[1] = pointst[0][1];

    float current_dir[2] = {0,1};

    auto path_d = irobot_create_msgs::action::DriveDistance_Goal();
    auto path_r = irobot_create_msgs::action::RotateAngle_Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goals");


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
  rclcpp::Publisher<irobot_create_msgs::action::DriveDistance_Goal>::SharedPtr drive_publisher_;
  rclcpp::Publisher<irobot_create_msgs::action::RotateAngle_Goal>::SharedPtr rotate_publisher_;
  rclcpp::Client<turtlebot4_node_interfaces::srv::Drive>::SharedPtr drive_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4Node>());
  rclcpp::shutdown();
  return 0;
}
