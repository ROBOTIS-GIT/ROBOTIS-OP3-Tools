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

/* Author: Jay Song */

#ifndef OP3_OFFSET_TUNER_SERVER_H_
#define OP3_OFFSET_TUNER_SERVER_H_

#include <fstream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "op3_base_module/base_module.h"
#include "op3_offset_tuner_msgs/msg/joint_offset_data.hpp"
#include "op3_offset_tuner_msgs/msg/joint_torque_on_off_array.hpp"
#include "op3_offset_tuner_msgs/srv/get_present_joint_offset_data.hpp"
#include "robotis_controller/robotis_controller.h"

namespace robotis_op
{

class JointOffsetData
{
public:
  double joint_offset_rad_;
  double joint_init_pos_rad_;
  int p_gain_;
  int i_gain_;
  int d_gain_;

  JointOffsetData()
    : joint_offset_rad_(0), joint_init_pos_rad_(0), p_gain_(800), i_gain_(0), d_gain_(0) { }

  JointOffsetData(double joint_offset_rad, double joint_init_pose_rad)
    : joint_offset_rad_(joint_offset_rad), joint_init_pos_rad_(joint_init_pose_rad), p_gain_(800), i_gain_(0), d_gain_(0) { }
};

class OffsetTunerServer : public rclcpp::Node
{
public:
  OffsetTunerServer();
  ~OffsetTunerServer();

  bool initialize();
  void moveToInitPose();
  void stringMsgsCallBack(const std_msgs::msg::String::SharedPtr msg);
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointOffsetDataCallback(const op3_offset_tuner_msgs::msg::JointOffsetData::SharedPtr msg);
  void jointTorqueOnOffCallback(const op3_offset_tuner_msgs::msg::JointTorqueOnOffArray::SharedPtr msg);
  bool getPresentJointOffsetDataServiceCallback(const std::shared_ptr<op3_offset_tuner_msgs::srv::GetPresentJointOffsetData::Request> req,
                                                std::shared_ptr<op3_offset_tuner_msgs::srv::GetPresentJointOffsetData::Response> res);

 private:
  const int BAUD_RATE = 2000000;
  const double PROTOCOL_VERSION = 2.0;
  const int SUB_CONTROLLER_ID = 200;
  const char * SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
  const int POWER_CTRL_TABLE = 24;

  void setCtrlModule(std::string module);
  void getInitPose(const std::string &path);

  robotis_framework::RobotisController* controller_;

  std::string offset_file_;
  std::string robot_file_;
  std::string init_file_;
  std::map<std::string, JointOffsetData*> robot_offset_data_;
  std::map<std::string, bool> robot_torque_enable_data_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_ctrl_module_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr send_tra_sub_;
  rclcpp::Subscription<op3_offset_tuner_msgs::msg::JointOffsetData>::SharedPtr joint_offset_data_sub_;
  rclcpp::Subscription<op3_offset_tuner_msgs::msg::JointTorqueOnOffArray>::SharedPtr joint_torque_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Service<op3_offset_tuner_msgs::srv::GetPresentJointOffsetData>::SharedPtr offset_data_server_;
};

}

#endif /* OP3_OFFSET_TUNER_SERVER_H_ */
