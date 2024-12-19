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

#ifndef OP3_DEMO_QNODE_HPP_
#define OP3_DEMO_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <interactive_markers/interactive_marker_server.hpp>
// #include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/get_joint_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"

// walking demo
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_walking_module_msgs/srv/get_walking_param.hpp"
#include "op3_walking_module_msgs/srv/set_walking_param.hpp"

// Preview walking
#include "op3_online_walking_module_msgs/msg/foot_step_command.hpp"
#include "op3_online_walking_module_msgs/msg/walking_param.hpp"
#include "op3_online_walking_module_msgs/msg/joint_pose.hpp"
#include "op3_online_walking_module_msgs/msg/step2_d_array.hpp"
// #include "humanoid_nav_msgs/srv/plan_footsteps.hpp"

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNodeOP3 : public QObject, public rclcpp::Node
{
  Q_OBJECT
public:

  enum LogLevel
  {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4
  };

  QNodeOP3(int argc, char** argv, QObject* parent = nullptr);
  virtual ~QNodeOP3();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "Demo");
  void clearLog();
  void assemble_lidar();
  void setJointControlMode(const robotis_controller_msgs::msg::JointCtrlModule &msg);
  void setControlMode(const std::string &mode);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  bool getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);
  std::string getModeName(const int &index);
  int getModeIndex(const std::string &mode_name);
  int getModeSize();
  int getJointSize();
  void clearUsingModule();
  bool isUsingModule(std::string module_name);
  void moveInitPose();

  void init_default_demo();
  // Head control
  void setHeadJoint(double pan, double tilt);

  // Walking
  void setWalkingCommand(const std::string &command);
  void refreshWalkingParam();
  void saveWalkingParam();
  void applyWalkingParam(const op3_walking_module_msgs::msg::WalkingParam &walking_param);
  void initGyro();

  // Preview Walking
  void init_preview_walking();
  void sendFootStepCommandMsg(op3_online_walking_module_msgs::msg::FootStepCommand msg);
  void sendWalkingParamMsg(op3_online_walking_module_msgs::msg::WalkingParam msg);
  void sendBodyOffsetMsg(geometry_msgs::msg::Pose msg);
  void sendFootDistanceMsg(std_msgs::msg::Float64 msg);
  void sendResetBodyMsg(std_msgs::msg::Bool msg );
  void sendWholebodyBalanceMsg(std_msgs::msg::String msg);
  void parseIniPoseData(const std::string &path);
  void sendJointPoseMsg(op3_online_walking_module_msgs::msg::JointPose msg);

  // Preview /w footstep
  void makeFootstepUsingPlanner();
  void makeFootstepUsingPlanner(const geometry_msgs::msg::Pose &target_foot_pose);
  void visualizePreviewFootsteps(bool clear);
  void clearFootsteps();
  void setWalkingFootsteps(const double &step_time);

  // Demo
  void setDemoCommand(const std::string &command);
  void setActionModuleBody();
  void setModuleToDemo();

  // Interactive marker
  void makeInteractiveMarker(const geometry_msgs::msg::Pose& marker_pose);
  bool updateInteractiveMarker(const geometry_msgs::msg::Pose& pose);
  void getInteractiveMarkerPose();
  void clearInteractiveMarker();

  std::map<int, std::string> module_table_;
  std::map<int, std::string> motion_table_;
  std::map<int, int> motion_shortcut_table_;

public Q_SLOTS:
  void getJointControlMode();
  void playMotion(int motion_index);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void updateCurrentJointControlMode(std::vector<int> mode);

  // Head
  void updateHeadAngles(double pan, double tilt);

  // Walking
  void updateWalkingParameters(op3_walking_module_msgs::msg::WalkingParam params);

  // Interactive marker
  void updateDemoPoint(const geometry_msgs::msg::Point point);
  void updateDemoPose(const geometry_msgs::msg::Pose pose);

private:
  void parseJointNameFromYaml(const std::string &path);
  void parseMotionMapFromYaml(const std::string &path);
  void refreshCurrentJointControlCallback(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg);
  void updateHeadJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void statusMsgCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg);

  // interactive marker
  void pointStampedCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void interactiveMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);

  // localization
  bool transformPose(const std::string &from_id, const std::string &to_id, const geometry_msgs::msg::Pose &from_pose,
                     geometry_msgs::msg::Pose &to_pose, bool inverse = false);

  int init_argc_;
  char** init_argv_;
  bool debug_;
  double body_height_;

  // interactive marker
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_clicked_point_sub_;
  std::string frame_id_;
  std::string marker_name_;
  geometry_msgs::msg::Pose pose_from_ui_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose curr_pose_msg_;
  // std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  op3_walking_module_msgs::msg::WalkingParam walking_param_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_pose_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr module_control_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_control_preset_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr init_gyro_pub_;
  rclcpp::Subscription<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr init_ft_foot_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr both_ft_foot_sub_;
  rclcpp::Subscription<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr current_module_control_sub_;
  rclcpp::Client<robotis_controller_msgs::srv::GetJointModule>::SharedPtr get_module_control_client_;

  // Head
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr set_head_joint_angle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_states_sub_;

  // Walking
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_walking_command_pub;
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr set_walking_param_pub;
  rclcpp::Client<op3_walking_module_msgs::srv::GetWalkingParam>::SharedPtr get_walking_param_client_;

  // preview walking
  // rclcpp::Client<humanoid_nav_msgs::srv::PlanFootsteps>::SharedPtr humanoid_footstep_client_;
  rclcpp::Publisher<op3_online_walking_module_msgs::msg::FootStepCommand>::SharedPtr foot_step_command_pub_;
  rclcpp::Publisher<op3_online_walking_module_msgs::msg::Step2DArray>::SharedPtr set_walking_footsteps_pub_;
  rclcpp::Publisher<op3_online_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr body_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr foot_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wholebody_balance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_body_msg_pub_;
  rclcpp::Publisher<op3_online_walking_module_msgs::msg::JointPose>::SharedPtr joint_pose_msg_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<geometry_msgs::msg::Pose2D> preview_foot_steps_;
  std::vector<int> preview_foot_types_;

  // Action
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;

  // Demo
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr demo_command_pub_;

  rclcpp::Time start_time_;

  QStringListModel logging_model_;
  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  std::map<int, std::string> index_mode_table_;
  std::map<std::string, int> mode_index_table_;
  std::map<std::string, bool> using_mode_table_;
};

}  // namespace robotis_op

template<typename T>
T deg2rad(T deg)
{
  return deg * M_PI / 180;
}

template<typename T>
T rad2deg(T rad)
{
  return rad * 180 / M_PI;
}
#endif /* OP3_DEMO_QNODE_HPP_ */
