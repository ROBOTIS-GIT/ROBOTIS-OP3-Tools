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

#ifndef OP3_CAMERA_SETTING_TOOL_H_
#define OP3_CAMERA_SETTING_TOOL_H_

#include <thread>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_camera_setting_tool/camera_params_config.h"

#include "op3_camera_setting_tool_msgs/msg/v4l_parameter.hpp"
#include "op3_camera_setting_tool_msgs/msg/v4l_parameters.hpp"

#include "op3_camera_setting_tool_msgs/srv/get_parameters.hpp"
#include "op3_camera_setting_tool_msgs/srv/set_parameters.hpp"

std::string g_device_name;
std::string g_camera_node_name;
std::map<std::string, std::string> g_param_list;

robotis_op::CameraParamsConfig g_dyn_config;

// web setting
std::string g_default_setting_path;
rclcpp::Publisher<op3_camera_setting_tool_msgs::msg::CameraParams>::SharedPtr g_param_pub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_param_command_sub;
rclcpp::Service<op3_camera_setting_tool_msgs::srv::GetParameters>::SharedPtr g_get_param_service;
rclcpp::Service<op3_camera_setting_tool_msgs::srv::SetParameters>::SharedPtr g_set_param_service;

bool g_has_path;
std::string g_param_path;

void paramCallback(const rclcpp::Parameter& p);
void changeDynParam(const rclcpp::Parameter& p);
void changeDynParam(const std::string& param_name, const int& value);
//void updateDynParam(const std::string& param_name, const int& value);

void setCameraParameterCallback(const op3_camera_setting_tool_msgs::msg::V4lParameter::SharedPtr msg);
void setCameraParametersCallback(const op3_camera_setting_tool_msgs::msg::V4lParameters::SharedPtr msg);

void setV4lParameter(const std::string& param_name, const std::string& value);
void setV4lParameter(const std::string& param_name, const int& value);
void setV4lParameter(const std::string& cmd);

void getROSParam();
void setROSParam(const std::string& param_name, const int& value);

void paramCommandCallback(const std_msgs::msg::String::SharedPtr msg);
bool setParamCallback(const std::shared_ptr<op3_camera_setting_tool_msgs::srv::SetParameters::Request> req, std::shared_ptr<op3_camera_setting_tool_msgs::srv::SetParameters::Response> res);
bool getParamCallback(const std::shared_ptr<op3_camera_setting_tool_msgs::srv::GetParameters::Request> req, std::shared_ptr<op3_camera_setting_tool_msgs::srv::GetParameters::Response> res);
void resetParameter();
void saveParameter();
void publishParam();

#endif /* OP3_CAMERA_SETTING_TOOL_H_ */
