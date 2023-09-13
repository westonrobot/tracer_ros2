/*
 * tracer_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:38
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "tracer_base/tracer_base_ros.hpp"

#include "tracer_base/tracer_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
TracerBaseRos::TracerBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", "can0");

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("odom_topic_name", "odom");

  this->declare_parameter("is_tracer_mini", false);
  this->declare_parameter("simulated_robot", false);
  this->declare_parameter("control_rate", 50);

  LoadParameters();
}

void TracerBaseRos::LoadParameters() {
  port_name_ =
      this->get_parameter("port_name").get_parameter_value().get<std::string>();

  odom_frame_ = this->get_parameter("odom_frame")
                    .get_parameter_value()
                    .get<std::string>();
  base_frame_ = this->get_parameter("base_frame")
                    .get_parameter_value()
                    .get<std::string>();
  odom_topic_name_ = this->get_parameter("odom_topic_name")
                         .get_parameter_value()
                         .get<std::string>();

  is_tracer_mini_ =
      this->get_parameter("is_tracer_mini").get_parameter_value().get<bool>();
  simulated_robot_ =
      this->get_parameter("simulated_robot").get_parameter_value().get<bool>();
  sim_control_rate_ =
      this->get_parameter("control_rate").get_parameter_value().get<int>();

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- is tracer mini: " << std::boolalpha << is_tracer_mini_
            << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool TracerBaseRos::Initialize() {
  if (is_tracer_mini_) {
    std::cout << "Robot base: Tracer Mini" << std::endl;
  } else {
    std::cout << "Robot base: Tracer" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;

      robot_ = std::unique_ptr<TracerRobot>();
      std::cout << "Creating interface for Tracer with AGX_V2 Protocol"
                << std::endl;
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void TracerBaseRos::Stop() { keep_running_ = false; }

void TracerBaseRos::Run() {
  robot_ = std::make_shared<TracerRobot>();
  // instantiate a ROS messenger
  // TracerMessenger messenger(robot_.get(),this);
  std::unique_ptr<TracerMessenger<TracerRobot>> messenger =
      std::unique_ptr<TracerMessenger<TracerRobot>>(
          new TracerMessenger<TracerRobot>(robot_, this));

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

  // connect to robot and setup ROS subscription
  if (port_name_.find("can") != std::string::npos) {
    if (robot_->Connect(port_name_)) {
      robot_->EnableCommandedMode();
      std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return;
    }
  } else {
    std::cout << "Please check the specified port name is a CAN port"
              << std::endl;
    return;
  }

  // publish robot state at 50Hz while listening to twist commands
  messenger->SetupSubscription();
  keep_running_ = true;
  rclcpp::Rate rate(50);
  while (keep_running_) {
    messenger->PublishStateToROS();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}
}  // namespace westonrobot