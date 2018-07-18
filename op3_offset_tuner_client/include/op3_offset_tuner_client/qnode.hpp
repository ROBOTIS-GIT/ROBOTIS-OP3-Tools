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
 ** Ifdefs
 *****************************************************************************/

#ifndef OP3_OFFSET_TUNER_CLIENT_QNODE_HPP_
#define OP3_OFFSET_TUNER_CLIENT_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include "op3_offset_tuner_msgs/JointOffsetData.h"
#include "op3_offset_tuner_msgs/JointOffsetPositionData.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOff.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "op3_offset_tuner_msgs/GetPresentJointOffsetData.h"

#endif
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace op3_offset_tuner_client
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread
{
Q_OBJECT
 public:
  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg);

  void sendTorqueEnableMsg(op3_offset_tuner_msgs::JointTorqueOnOffArray msg);
  void sendJointOffsetDataMsg(op3_offset_tuner_msgs::JointOffsetData msg);
  void sendCommandMsg(std_msgs::String msg);
  bool isRefresh()
  {
    return is_refresh_;
  }

  std::map<int, std::string> right_arm_offset_group_;
  std::map<int, std::string> left_arm_offset_group_;
  std::map<int, std::string> legs_offset_group_;
  std::map<int, std::string> body_offset_group_;

 public Q_SLOTS:
  void getPresentJointOffsetData(bool recalculate_offset = false);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void updatePresentJointOffsetData(op3_offset_tuner_msgs::JointOffsetPositionData msg);

 private:
  void parseOffsetGroup(const std::string &path);

  int init_argc_;
  char** init_argv_;
  bool is_refresh_;
  ros::Publisher chatter_publisher_;
  QStringListModel logging_model_;

  ros::Publisher joint_offset_data_pub_;
  ros::Publisher torque_enable_pub_;
  ros::Publisher command_pub_;

  ros::ServiceClient get_present_joint_offset_data_client_;
};

}  // namespace op3_offset_tuner_client

#endif /* OP3_OFFSET_TUNER_CLIENT_QNODE_HPP_ */
