#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "common/ros2_sport_client.h"
#include "unitree_api/msg/request.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
// rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_suber;
rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_suber;

unitree_api::msg::Request req;
SportClient sport_req;

float vx = 0.0f;
float vyaw = 0.0f;
float vy = 0.0f;

rclcpp::Node::SharedPtr nh;

float joySpeed = 0.0f;
float joySpeedRaw = 0.0f;
float joySpeedYaw = 0.0f;
float joySpeedLateral = 0.0f;

float maxSpeedYaw = 1.4f;
float maxSpeedLateral = 0.5f;

bool manualMode = false;
bool estopLatched = false;
bool hasSentStandDown = false;
bool hasSentStopAfterManual = false;
double joyTime = 0.0;
double joyTimeoutSec = 0.35;

// void vel_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
// {
//     vx = msg->twist.linear.x;
//     vyaw = msg->twist.angular.z;
//     new_cmd = true;
// }

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->axes.size() < 6) {
    RCLCPP_WARN_THROTTLE(
      nh->get_logger(), *nh->get_clock(), 2000,
      "vel_cmd_repub: /joy has insufficient axes (%zu), ignoring.", joy->axes.size());
    return;
  }

  joyTime = nh->now().seconds();
  joySpeedRaw = joy->axes[4];
  joySpeed = joySpeedRaw;
  joySpeedLateral = joy->axes[3] * maxSpeedLateral;

  joySpeedYaw = joy->axes[0] * maxSpeedYaw;

  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joySpeed < -1.0) joySpeed = -1.0;
  if (joySpeedLateral > maxSpeedLateral) joySpeedLateral = maxSpeedLateral;
  if (joySpeedLateral < -maxSpeedLateral) joySpeedLateral = -maxSpeedLateral;
  
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }

  // Web GUI safety button: latch emergency until reboot is requested.
  if (joy->buttons.size() > 8 && joy->buttons[8] == 1) {
    estopLatched = true;
    hasSentStandDown = false;
    RCLCPP_ERROR(nh->get_logger(), "vel_cmd_repub: EMERGENCY SHUTDOWN latched");
  }

  // Web GUI reboot button: clear emergency latch and stand robot up.
  if (joy->buttons.size() > 9 && joy->buttons[9] == 1) {
    if (estopLatched) {
      estopLatched = false;
      hasSentStandDown = false;
      sport_req.StandUp(req);
      req_puber->publish(req);
      RCLCPP_INFO(nh->get_logger(), "vel_cmd_repub: Reboot received, StandUp sent");
    }
  }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals
    nh = rclcpp::Node::make_shared("vel_cmd_repub"); // Create a ROS2 node and make share with low_level_cmd_sender class

  nh->declare_parameter<double>("joy_timeout_sec", joyTimeoutSec);
  nh->declare_parameter<double>("max_speed_yaw", maxSpeedYaw);
  nh->declare_parameter<double>("max_speed_lateral", maxSpeedLateral);
  nh->get_parameter("joy_timeout_sec", joyTimeoutSec);
  nh->get_parameter("max_speed_yaw", maxSpeedYaw);
  nh->get_parameter("max_speed_lateral", maxSpeedLateral);

    // state_suber = nh->create_subscription<unitree_go::msg::SportModeState>("sportmodestate", 10);
    
    // vel_cmd_suber = nh->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10, vel_cmd_callback);

    joy_suber = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);
    req_puber = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    rclcpp::Rate rate(100); // Set the frequency of the timer callback
    bool status = rclcpp::ok();
    while (status){
        rclcpp::spin_some(nh);
        const double now = nh->now().seconds();
        const bool joyFresh = (joyTime > 0.0) && ((now - joyTime) <= joyTimeoutSec);
        const bool allowManualPublish = manualMode && joyFresh && !estopLatched;

        // Safety has the highest priority.
        if (estopLatched) {
          if (!hasSentStandDown) {
            sport_req.StopMove(req);
            req_puber->publish(req);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            sport_req.StandDown(req);
            req_puber->publish(req);
            hasSentStandDown = true;
          }
          hasSentStopAfterManual = false;
          rate.sleep();
          status = rclcpp::ok();
          continue;
        }

        if (allowManualPublish) {
          vx = joySpeed;
          vyaw = joySpeedYaw;
          vy = joySpeedLateral;
          sport_req.Move(req, vx, vy, vyaw);
          req_puber->publish(req);
          hasSentStopAfterManual = false;
        } else {
          // Publish one StopMove when manual control ends, then stay silent.
          // Staying silent avoids fighting with pathFollower when both nodes run.
          if (!hasSentStopAfterManual) {
            sport_req.StopMove(req);
            req_puber->publish(req);
            hasSentStopAfterManual = true;
          }
        }

        rate.sleep();
        status = rclcpp::ok();
    }

    rclcpp::shutdown();
    return 0;
}
