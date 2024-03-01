#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "Vector3.hpp"


class TurtleBot4Node : public rclcpp::Node
{
public:
  TurtleBot4Node()
  : Node("turtlebot4_cpp_node")
  {
    // Subscribe to the /interface_buttons topic
    interface_buttons_subscriber_ =
      this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "/interface_buttons",
      rclcpp::SensorDataQoS(),
      std::bind(&TurtleBot4Node::interface_buttons_callback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS());
  }

private:
  // Interface buttons subscription callback

  float points[21][2] = {{0.9,0.27},{1,0.58},{0.855,0.70},{0.745,0.38},{0.36,0.293},{0.5,0.171},{0.905,0.265}};


  void interface_buttons_callback(
    const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
    { 
      if(create3_buttons_msg->button_1.is_pressed) {
        RCLCPP_INFO(this->get_logger(), "Button 1 Pressed!");
        path();
      }
    }
  
  void path(){

    // now the path function is going to generate a path based on the points, 

    //take the first point, and assume that's the current position
    float current_pos[2];
    current_pos[0] = points[0][0];
    current_pos[1] = points[0][1];
    Vector3 current_dir(0,1,0);
    // current_dir.data[1] = 1;


    //then for each subsequent point, calculate the offset from the previous point to get relative vector direction

    // then set it to be the new current position.

    for(int i = 1; i < 7; i++){
      float next_pos[2];
      next_pos[0] = points[i][0];
      next_pos[1] = points[i][1];
      float delta_x = next_pos[0] - current_pos[0];
      float delta_y = next_pos[1] - current_pos[1];

      float mag = std::sqrt(delta_x * delta_x + delta_y * delta_y);
      Vector3 t_dir(delta_x,delta_y,0);
      t_dir.normalize();
      float ang = t_dir.angleBetweenVectors(current_dir);

      //construct the Twist based on the new information

      auto path_seg = geometry_msgs::msg::Twist();
      path_seg.linear.x = delta_x*10;
      path_seg.linear.z = delta_y*10;

      //do we need to set the empty values? 
      std::stringstream log;
      std::string logger;
      log << "published path:" << "\n\tangle: " << ang << " mag: " << mag;

      logger = log.str();

      path_publisher_->publish(path_seg);
      RCLCPP_INFO(this->get_logger(), logger.c_str());

      current_pos[0] = next_pos[0];
      current_pos[1] = next_pos[1];

      // current_rot = 0;
      sleep(2);
    }

    // auto path = geometry_msgs::msg::Twist();

    // path.linear.x = 0;
    // path.linear.y = 0;
    // path.linear.z = 0;

    // path.angular.x = 0;
    // path.angular.y = 0;
    // path.angular.z = 0;

    // path_publisher_->publish(path);

    // RCLCPP_INFO(this->get_logger(), "published path");
  }

  // Interface Button Subscriber
  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr path_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4Node>());
  rclcpp::shutdown();
  return 0;
}
