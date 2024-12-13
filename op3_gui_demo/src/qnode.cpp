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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../include/op3_gui_demo/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNodeOP3::QNodeOP3(int argc, char** argv, QObject *parent)
  : Node("op3_gui_demo"),
    QObject(parent),
    init_argc_(argc),
    init_argv_(argv),
    body_height_(-1.0)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    if (arg_code == "debug")
      debug_ = true;
    else
      debug_ = false;
  }

  // Add your ros communications here.
  module_control_pub_ = this->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 10);
  module_control_preset_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
  init_pose_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 10);

  status_msg_sub_ = this->create_subscription<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 10, std::bind(&QNodeOP3::statusMsgCallback, this, std::placeholders::_1));
  current_module_control_sub_ = this->create_subscription<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/present_joint_ctrl_modules", 10, std::bind(&QNodeOP3::refreshCurrentJointControlCallback, this, std::placeholders::_1));

  get_module_control_client_ = this->create_client<robotis_controller_msgs::srv::GetJointModule>("/robotis/get_present_joint_ctrl_modules");

  // For default demo
  init_default_demo(shared_from_this());

  // Preview
  init_preview_walking(shared_from_this());

  // Config
  std::string default_config_path = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME) + "/config/gui_config.yaml";
  std::string config_path = this->declare_parameter<std::string>("gui_config", default_config_path);
  parseJointNameFromYaml(config_path);

  // start time
  start_time_ = rclcpp::Clock().now();

  // tf_listener_.reset(new tf2_ros::TransformListener(*this, tf_buffer_));
}

QNodeOP3::~QNodeOP3()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNodeOP3::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
  // load yaml
  doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
  RCLCPP_ERROR(this->get_logger(), "Fail to load id_joint table yaml.");
  return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
  int joint_id;
  std::string joint_name;

  joint_id = _it->first.as<int>();
  joint_name = _it->second.as<std::string>();

  id_joint_table_[joint_id] = joint_name;
  joint_id_table_[joint_name] = joint_id;

  if (debug_)
    std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }

  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string> >();

  int module_index = 0;
  for (std::vector<std::string>::iterator modules_it = modules.begin(); modules_it != modules.end(); ++modules_it)
  {
  std::string module_name = *modules_it;

  index_mode_table_[module_index] = module_name;
  mode_index_table_[module_name] = module_index++;

  using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
  int key_index;
  std::string module_name;

  key_index = yaml_it->first.as<int>();
  module_name = yaml_it->second.as<std::string>();

  module_table_[key_index] = module_name;
  if (debug_)
    std::cout << "Preset : " << module_name << std::endl;
  }
}

// joint id -> joint name
bool QNodeOP3::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;

  map_it = id_joint_table_.find(id);
  if (map_it == id_joint_table_.end())
  return false;

  joint_name = map_it->second;
  return true;
}

// joint name -> joint id
bool QNodeOP3::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator map_it;

  map_it = joint_id_table_.find(joint_name);
  if (map_it == joint_id_table_.end())
  return false;

  id = map_it->second;
  return true;
}

// map index -> joint id & joint name
bool QNodeOP3::getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;
  int count = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, count++)
  {
  if (index == count)
  {
    id = map_it->first;
    joint_name = map_it->second;
    return true;
  }
  }
  return false;
}

// mode(module) index -> mode(module) name
std::string QNodeOP3::getModeName(const int &index)
{
  std::string mode = "";
  std::map<int, std::string>::iterator map_it = index_mode_table_.find(index);

  if (map_it != index_mode_table_.end())
  mode = map_it->second;

  return mode;
}

// mode(module) name -> mode(module) index
int QNodeOP3::getModeIndex(const std::string &mode_name)
{
  int mode_index = -1;
  std::map<std::string, int>::iterator map_it = mode_index_table_.find(mode_name);

  if (map_it != mode_index_table_.end())
  mode_index = map_it->second;

  return mode_index;
}

// number of mode(module)s
int QNodeOP3::getModeSize()
{
  return index_mode_table_.size();
}

// number of joints
int QNodeOP3::getJointSize()
{
  return id_joint_table_.size();
}

void QNodeOP3::clearUsingModule()
{
  for (std::map<std::string, bool>::iterator map_it = using_mode_table_.begin(); map_it != using_mode_table_.end();
    ++map_it)
  map_it->second = false;
}

bool QNodeOP3::isUsingModule(std::string module_name)
{
  std::map<std::string, bool>::iterator map_it = using_mode_table_.find(module_name);

  if (map_it == using_mode_table_.end())
  return false;

  return map_it->second;
}

// move ini pose : wholedody module
void QNodeOP3::moveInitPose()
{
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_->publish(init_msg);

  log(Info, "Go to robot initial pose.");
}

// set mode(module) to each joint
void QNodeOP3::setJointControlMode(const robotis_controller_msgs::msg::JointCtrlModule &msg)
{
  module_control_pub_->publish(msg);
}

void QNodeOP3::setControlMode(const std::string &mode)
{
  std_msgs::msg::String set_module_msg;
  set_module_msg.data = mode;

  module_control_preset_pub_->publish(set_module_msg);

  std::stringstream _ss;
  _ss << "Set Mode : " << mode;
  log(Info, _ss.str());
}

// get current mode(module) of joints
void QNodeOP3::getJointControlMode()
{
  auto request = std::make_shared<robotis_controller_msgs::srv::GetJointModule::Request>();
  std::map<std::string, int> service_map;

  // _get_joint.request
  std::map<int, std::string>::iterator map_it;
  int index = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, index++)
  {
  request->joint_name.push_back(map_it->second);
  service_map[map_it->second] = index;
  }

  auto result = get_module_control_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
  auto response = result.get();
  // _get_joint.response
  std::vector<int> modules;
  modules.resize(getJointSize());

  // clear current using modules
  clearUsingModule();

  for (int ix = 0; ix < response->joint_name.size(); ix++)
  {
    std::string joint_name = response->joint_name[ix];
    std::string module_name = response->module_name[ix];

    std::map<std::string, int>::iterator service_it = service_map.find(joint_name);
    if (service_it == service_map.end())
    continue;

    index = service_it->second;

    service_it = mode_index_table_.find(module_name);
    if (service_it == mode_index_table_.end())
    continue;

    modules.at(index) = service_it->second;

    std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
    if (module_it != using_mode_table_.end())
    module_it->second = true;
  }

  // update ui
  Q_EMIT updateCurrentJointControlMode(modules);
  log(Info, "Get current Mode");
  }
  else
  log(Error, "Fail to get current joint control module.");
}

void QNodeOP3::refreshCurrentJointControlCallback(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "set current joint module");
  //int _index = 0;

  std::vector<int> modules;
  modules.resize(getJointSize());

  std::map<std::string, int> joint_module_map;

  // clear current using modules
  clearUsingModule();

  for (int ix = 0; ix < msg->joint_name.size(); ix++)
  {
  std::string joint_name = msg->joint_name[ix];
  std::string module_name = msg->module_name[ix];

  joint_module_map[joint_name] = getModeIndex(module_name);

  std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
  if (module_it != using_mode_table_.end())
    module_it->second = true;
  }

  for (int ix = 0; ix < getJointSize(); ix++)
  {
  int id = 0;
  std::string joint_name = "";

  if (getIDJointNameFromIndex(ix, id, joint_name) == false)
    continue;

  std::map<std::string, int>::iterator module_it = joint_module_map.find(joint_name);
  if (module_it == joint_module_map.end())
    continue;

  modules.at(ix) = module_it->second;
  }

  // update ui
  Q_EMIT updateCurrentJointControlMode(modules);

  log(Info, "Applied Mode", "Manager");
}


// LOG
void QNodeOP3::statusMsgCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNodeOP3::log(const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  rclcpp::Duration duration_time = rclcpp::Clock().now() - start_time_;
  int current_time = duration_time.seconds();
  int min_time = 0, sec_time = 0;
  min_time = (int) (current_time / 60);
  sec_time = (int) (current_time % 60);

  std::stringstream min_str, sec_str;
  if (min_time < 10)
  min_str << "0";
  if (sec_time < 10)
  sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream sender_ss;
  sender_ss << "[" << sender << "] ";

  switch (level)
  {
  case (Debug):
  {
    RCLCPP_DEBUG(this->get_logger(), msg.c_str());
    logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
    break;
  }
  case (Info):
  {
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
    break;
  }
  case (Warn):
  {
    RCLCPP_WARN(this->get_logger(), msg.c_str());
    logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
    break;
  }
  case (Error):
  {
    RCLCPP_ERROR(this->get_logger(), msg.c_str());
    logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
    break;
  }
  case (Fatal):
  {
    RCLCPP_FATAL(this->get_logger(), msg.c_str());
    logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNodeOP3::clearLog()
{
  if (logging_model_.rowCount() == 0)
  return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace robotis_op
