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

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "robotis_controller/robotis_controller.h"
#include "op3_base_module/base_module.h"
#include "op3_offset_tuner_msgs/JointOffsetData.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "op3_offset_tuner_msgs/GetPresentJointOffsetData.h"

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
  {
    joint_offset_rad_ = 0;
    joint_init_pos_rad_ = 0;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double joint_init_pose_rad)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->joint_init_pos_rad_ = joint_init_pose_rad;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  ~JointOffsetData()
  {
  }
};

class OffsetTunerServer : public robotis_framework::Singleton<OffsetTunerServer>
{

 public:
  OffsetTunerServer();
  ~OffsetTunerServer();

  bool initialize();
  void moveToInitPose();
  void stringMsgsCallBack(const std_msgs::String::ConstPtr& msg);
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void jointOffsetDataCallback(const op3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg);
  void jointTorqueOnOffCallback(const op3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg);
  bool getPresentJointOffsetDataServiceCallback(op3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                                                op3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res);

 private:
  const int BAUD_RATE = 2000000;
  const double PROTOCOL_VERSION = 2.0;
  const int SUB_CONTROLLER_ID = 200;
  const char *SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
  const int POWER_CTRL_TABLE = 24;

  void setCtrlModule(std::string module);
  void getInitPose(const std::string &path);

  robotis_framework::RobotisController* controller_;

  std::string offset_file_;
  std::string robot_file_;
  std::string init_file_;
  std::map<std::string, JointOffsetData*> robot_offset_data_;
  std::map<std::string, bool> robot_torque_enable_data_;

  ros::Subscriber send_tra_sub_;
  ros::Subscriber joint_offset_data_sub_;
  ros::Subscriber joint_torque_enable_sub_;
  ros::Subscriber command_sub_;
  ros::ServiceServer offset_data_server_;
};

}

#endif /* OP3_OFFSET_TUNER_SERVER_H_ */
