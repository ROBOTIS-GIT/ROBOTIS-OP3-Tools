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

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_camera_setting_tool/V4lParameter.h"
#include "op3_camera_setting_tool/V4lParameters.h"
#include "op3_camera_setting_tool/CameraParamsConfig.h"

#include "op3_camera_setting_tool/GetParameters.h"
#include "op3_camera_setting_tool/SetParameters.h"

std::string g_device_name;
std::string g_camera_node_name;
std::map<std::string, std::string> g_param_list;

boost::shared_ptr<dynamic_reconfigure::Server< op3_camera_setting_tool::CameraParamsConfig> > g_param_server;
op3_camera_setting_tool::CameraParamsConfig g_dyn_config;

// web setting
std::string g_default_setting_path;
ros::Publisher g_param_pub;
ros::Subscriber g_param_command_sub;
ros::ServiceServer g_get_param_client;
ros::ServiceServer g_set_param_client;

bool g_has_path;
std::string g_param_path;

void dynParamCallback(op3_camera_setting_tool::CameraParamsConfig &config, uint32_t level);
void changeDynParam(const std::string& param, const int& value);
void updateDynParam(op3_camera_setting_tool::CameraParamsConfig &config);

void setCameraParameterCallback(const op3_camera_setting_tool::V4lParameter::ConstPtr &msg);
void setCameraParametersCallback(const op3_camera_setting_tool::V4lParameters::ConstPtr &msg);

void setV4lParameter(const std::string& param, const std::string& value);
void setV4lParameter(const std::string& param, const int& value);
void setV4lParameter(const std::string& cmd);

void getROSParam();
void setROSParam(const std::string& param, const int& value);

void paramCommandCallback(const std_msgs::String::ConstPtr &msg);
bool setParamCallback(op3_camera_setting_tool::SetParameters::Request &req, op3_camera_setting_tool::SetParameters::Response &res);
bool getParamCallback(op3_camera_setting_tool::GetParameters::Request &req, op3_camera_setting_tool::GetParameters::Response &res);
void resetParameter();
void saveParameter();
void publishParam();

#endif /* OP3_CAMERA_SETTING_TOOL_H_ */
