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

#ifndef op3_tuner_client_QNODE_HPP_
#define op3_tuner_client_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <yaml-cpp/yaml.h>

#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "op3_tuning_module_msgs/msg/joint_offset_data.hpp"
#include "op3_tuning_module_msgs/msg/joint_offset_position_data.hpp"
#include "op3_tuning_module_msgs/msg/joint_torque_on_off.hpp"
#include "op3_tuning_module_msgs/msg/joint_torque_on_off_array.hpp"

#include "op3_tuning_module_msgs/srv/get_present_joint_offset_data.hpp"

#endif
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace op3_tuner_client
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread, public rclcpp::Node
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
  void clearLog();

  void sendTorqueEnableMsg(op3_tuning_module_msgs::msg::JointTorqueOnOffArray msg);
  void sendJointOffsetDataMsg(op3_tuning_module_msgs::msg::JointOffsetData msg);
  void sendJointGainDataMsg(op3_tuning_module_msgs::msg::JointOffsetData msg);
  void sendCommandMsg(std_msgs::msg::String msg);
  void sendTuningPoseMsg(std_msgs::msg::String msg);
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
  void updatePresentJointOffsetData(op3_tuning_module_msgs::msg::JointOffsetPositionData msg);

 private:
  void parseOffsetGroup(const std::string &path);

  int init_argc_;
  char** init_argv_;
  bool is_refresh_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_publisher_;
  QStringListModel logging_model_;

  rclcpp::Publisher<op3_tuning_module_msgs::msg::JointOffsetData>::SharedPtr joint_offset_data_pub_;
  rclcpp::Publisher<op3_tuning_module_msgs::msg::JointOffsetData>::SharedPtr joint_gain_data_pub_;
  rclcpp::Publisher<op3_tuning_module_msgs::msg::JointTorqueOnOffArray>::SharedPtr torque_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tuning_pose_pub_;

  rclcpp::Client<op3_tuning_module_msgs::srv::GetPresentJointOffsetData>::SharedPtr get_present_joint_offset_data_client_;
};

}  // namespace op3_tuner_client

#endif /* op3_tuner_client_QNODE_HPP_ */
