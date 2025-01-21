/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include "../include/op3_tuner_client/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace op3_tuner_client
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv)
  : Node("op3_tuner_client"),
    init_argc_(argc),
    init_argv_(argv),
    is_refresh_(false)
{
    // Add your ros communications here
  joint_offset_data_pub_ = this->create_publisher<op3_tuning_module_msgs::msg::JointOffsetData>(
    "/robotis/tuning_module/joint_offset_data", 10);
  joint_gain_data_pub_ = this->create_publisher<op3_tuning_module_msgs::msg::JointOffsetData>(
    "/robotis/tuning_module/joint_gain_data", 10);
  torque_enable_pub_ = this->create_publisher<op3_tuning_module_msgs::msg::JointTorqueOnOffArray>(
    "/robotis/tuning_module/torque_enable", 10);
  command_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/tuning_module/command", 10);
  tuning_pose_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/tuning_module/tuning_pose", 10);

  get_present_joint_offset_data_client_ = this->create_client<op3_tuning_module_msgs::srv::GetPresentJointOffsetData>(
    "/robotis/tuning_module/get_present_joint_offset_data");

  std::string default_config_path = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME) + "/config/joint_data.yaml";
  parseOffsetGroup(default_config_path);

  //rclcpp::init(init_argc_, init_argv_);
}

QNode::~QNode()
{
  rclcpp::shutdown();
}

bool QNode::init()
{
  auto executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(this->get_node_base_interface());
  start();
  return true;
}

void QNode::run()
{
  rclcpp::Rate loop_rate(125);

  while (rclcpp::ok())
  {
  rclcpp::spin_some(this->get_node_base_interface());
  loop_rate.sleep();
  }

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::sendTorqueEnableMsg(op3_tuning_module_msgs::msg::JointTorqueOnOffArray msg)
{
  torque_enable_pub_->publish(msg);

  log(Info, "Joint Torque On/Off");
}

void QNode::sendJointOffsetDataMsg(op3_tuning_module_msgs::msg::JointOffsetData msg)
{
  joint_offset_data_pub_->publish(msg);

  log(Info, "Send Joint Offset Data");
}

void QNode::sendJointGainDataMsg(op3_tuning_module_msgs::msg::JointOffsetData msg)
{
  joint_gain_data_pub_->publish(msg);

  log(Info, "Send Joint Gain Data");
}

void QNode::sendCommandMsg(std_msgs::msg::String msg)
{
  command_pub_->publish(msg);

  std::stringstream log_msg;
  log_msg << "Send Command : " << msg.data;

  log(Info, log_msg.str());
}

void QNode::sendTuningPoseMsg(std_msgs::msg::String msg)
{
  tuning_pose_pub_->publish(msg);

  std::stringstream log_msg;
  log_msg << "Send Tuning Pose : " << msg.data;

  log(Info, log_msg.str());
}

void QNode::getPresentJointOffsetData(bool recalculate_offset)
{
  is_refresh_ = true;

  auto request = std::make_shared<op3_tuning_module_msgs::srv::GetPresentJointOffsetData::Request>();

  //request

  //response
  auto future = get_present_joint_offset_data_client_->async_send_request(request,
      [this, recalculate_offset](rclcpp::Client<op3_tuning_module_msgs::srv::GetPresentJointOffsetData>::SharedFuture result)
      {
        auto response = result.get();
        if (response)
        {
          for (auto &data : response->present_data_array)
          {
            op3_tuning_module_msgs::msg::JointOffsetPositionData _temp = data;

            // if(recalculate_offset == true)
            //   _temp.offset_value = _temp.present_value - _temp.goal_value;

            Q_EMIT updatePresentJointOffsetData(_temp);
          }
        }
        else
          log(Error, "Fail to get joint offset data");
      });

  is_refresh_ = false;
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  auto timestamp = rclcpp::Clock().now().seconds();
  switch (level)
  {
  case (Debug):
  {
    RCLCPP_DEBUG(this->get_logger(), msg.c_str());
    logging_model_msg << "[DEBUG] [" << timestamp << "]: " << msg;
    break;
  }
  case (Info):
  {
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    logging_model_msg << "[INFO] [" << timestamp << "]: " << msg;
    break;
  }
  case (Warn):
  {
    RCLCPP_WARN(this->get_logger(), msg.c_str());
    logging_model_msg << "[INFO] [" << timestamp << "]: " << msg;
    break;
  }
  case (Error):
  {
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    logging_model_msg << "[ERROR] [" << timestamp << "]: " << msg;
    break;
  }
  case (Fatal):
  {
    RCLCPP_FATAL(this->get_logger(), msg.c_str());
    logging_model_msg << "[FATAL] [" << timestamp << "]: " << msg;
    break;
  }
  }

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNode::clearLog()
{
  if (logging_model_.rowCount() == 0)
  return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

void QNode::parseOffsetGroup(const std::string &path)
{
  YAML::Node doc;
  try
  {
  // load yaml
  doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
  RCLCPP_ERROR(this->get_logger(), "Fail to load offset config yaml.");
  return;
  }

  // parse right_arm
  YAML::Node right_arm_node = doc["right_arm"];
  for (YAML::iterator yaml_it = right_arm_node.begin(); yaml_it != right_arm_node.end(); ++yaml_it)
  {
  int index;
  std::string joint_name;

  index = yaml_it->first.as<int>();
  joint_name = yaml_it->second.as<std::string>();

  right_arm_offset_group_[index] = joint_name;
  }

  YAML::Node left_arm_node = doc["left_arm"];
  for (YAML::iterator yaml_it = left_arm_node.begin(); yaml_it != left_arm_node.end(); ++yaml_it)
  {
  int index;
  std::string joint_name;

  index = yaml_it->first.as<int>();
  joint_name = yaml_it->second.as<std::string>();

  left_arm_offset_group_[index] = joint_name;
  }

  YAML::Node legs_node = doc["legs"];
  for (YAML::iterator yaml_it = legs_node.begin(); yaml_it != legs_node.end(); ++yaml_it)
  {
  int index;
  std::string joint_name;

  index = yaml_it->first.as<int>();
  joint_name = yaml_it->second.as<std::string>();

  legs_offset_group_[index] = joint_name;
  }

  YAML::Node body_node = doc["body"];
  for (YAML::iterator yaml_it = body_node.begin(); yaml_it != body_node.end(); ++yaml_it)
  {
  int index;
  std::string joint_name;

  index = yaml_it->first.as<int>();
  joint_name = yaml_it->second.as<std::string>();

  body_offset_group_[index] = joint_name;
  }
}

}  // namespace op3_tuner_client
