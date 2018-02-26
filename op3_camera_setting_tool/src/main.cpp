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

/* Author: Kayman Jung*/

#include <ros/ros.h>

#include "op3_camera_setting_tool/op3_camera_setting_tool.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_camera_setting_tool");

  ros::NodeHandle nh;

  // get param to use
  nh.param("video_device", g_device_name, std::string("/dev/video0"));

  // v4l param name, ros param name
  g_param_list["brightness"] = "brightness";
  g_param_list["contrast"] = "contrast";
  g_param_list["saturation"] = "saturation";
  g_param_list["sharpness"] = "sharpness";

  g_param_list["focus_auto"] = "autofocus";
  g_param_list["focus_absolute"] = "focus";

  g_param_list["exposure_auto"] = "autoexposure";
  g_param_list["exposure_absolute"] = "exposure";
  g_param_list["gain"] = "gain";

  g_param_list["white_balance_temperature_auto"] = "auto_white_balance";
  g_param_list["white_balance_temperature"] = "white_balance";

  ros::Subscriber camera_param_sub = nh.subscribe("/op3_camera/set_param", 1, &setCameraParameterCallback);
  ros::Subscriber camera_params_sub = nh.subscribe("/op3_camera/set_params", 1, &setCameraParametersCallback);

  // web setting
  g_param_pub = nh.advertise<op3_camera_setting_tool::CameraParams>("/op3_camera/camera_params", 1);
  g_param_command_sub = nh.subscribe("/op3_camera/param_command", 1, &paramCommandCallback);
  g_set_param_client = nh.advertiseService("/op3_camera/set_camera_params", &setParamCallback);
  g_get_param_client = nh.advertiseService("/op3_camera/get_camera_params", &getParamCallback);
  g_default_setting_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/launch/camera_parameter_default.yaml";

  boost::recursive_mutex config_mutex;

  //dynamic_reconfigure::Server<op3_camera_setting_tool::cameraParamsConfig> param_server;
  dynamic_reconfigure::Server<op3_camera_setting_tool::cameraParamsConfig>::CallbackType callback_fnc;
  g_param_server.reset(new dynamic_reconfigure::Server<op3_camera_setting_tool::cameraParamsConfig>(config_mutex));

  g_param_server->getConfigDefault(g_dyn_config);

  // get param to set camera
  getROSParam();

  publishParam();

  updateDynParam(g_dyn_config);

  callback_fnc = boost::bind(&dynParamCallback, _1, _2);
  g_param_server->setCallback(callback_fnc);

  ROS_INFO("start camera setting tool!");

  ros::spin();

  return 0;
}

void changeDynParam(const std::string& param, const int& value)
{
  if (param == "brightness")
  {
    g_dyn_config.brightness = value;
  }
  else if (param == "contrast")
  {
    g_dyn_config.contrast = value;
  }
  else if (param == "saturation")
  {
    g_dyn_config.saturation = value;
  }
  else if (param == "sharpness")
  {
    g_dyn_config.sharpness = value;
  }
  else if (param == "gain")
  {
    g_dyn_config.gain = value;
  }
  else if (param == "focus_auto")
  {
    g_dyn_config.focus_auto = value;
  }
  else if (param == "focus_absolute")
  {
    g_dyn_config.focus_absolute = value;
  }
  else if (param == "exposure_auto")
  {
    g_dyn_config.exposure_auto = value;
  }
  else if (param == "exposure_absolute")
  {
    g_dyn_config.exposure_absolute = value;
  }
  else if (param == "white_balance_temperature_auto")
  {
    g_dyn_config.white_balance_temperature_auto = value;
  }
  else if (param == "white_balance_temperature")
  {
    g_dyn_config.white_balance_temperature = value;
  }
}

void dynParamCallback(op3_camera_setting_tool::cameraParamsConfig &config, uint32_t level)
{
  g_dyn_config = config;

  setV4lParameter("brightness", config.brightness);
  setV4lParameter("contrast", config.contrast);
  setV4lParameter("saturation", config.saturation);
  setV4lParameter("sharpness", config.sharpness);
  setV4lParameter("gain", config.gain);
  // focus
  setV4lParameter("focus_auto", config.focus_auto);
  if (config.focus_auto == false)
  {
    //0-255, -1 "leave alone"
    setV4lParameter("focus_absolute", config.focus_absolute);
  }

  // exposure
  // turn down exposure control (from max of 3)
  setV4lParameter("exposure_auto", config.exposure_auto);
  if (config.exposure_auto == 1)  // if it's manual
  {
    setV4lParameter("exposure_absolute", config.exposure_absolute);
  }

  // white balance
  setV4lParameter("white_balance_temperature_auto", config.white_balance_temperature_auto);
  if (config.white_balance_temperature_auto == false)
  {
    setV4lParameter("white_balance_temperature", config.white_balance_temperature);
  }
}

void updateDynParam(op3_camera_setting_tool::cameraParamsConfig &config)
{
  g_param_server->updateConfig(config);
}

void setCameraParameterCallback(const op3_camera_setting_tool::V4lParameter::ConstPtr &msg)
{
  setV4lParameter(msg->name, msg->value);

  updateDynParam(g_dyn_config);
}

void setCameraParametersCallback(const op3_camera_setting_tool::V4lParameters::ConstPtr &msg)
{
  for (int ix = 0; ix < msg->video_parameter.size(); ix++)
    setV4lParameter(msg->video_parameter[ix].name, msg->video_parameter[ix].value);

  updateDynParam(g_dyn_config);
}

void setV4lParameter(const std::string& param, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  setV4lParameter(cmd);

  setROSParam(param, boost::lexical_cast<int>(value));
  changeDynParam(param, boost::lexical_cast<int>(value));
}

void setV4lParameter(const std::string& param, const int& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  ROS_INFO("%s", cmd.c_str());
  setV4lParameter(cmd);

  setROSParam(param, value);
  changeDynParam(param, value);
}

void setV4lParameter(const std::string& cmd)
{
  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE *stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL)
        output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0)
      ROS_WARN("%s", output.c_str());
  }
  else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());

}

void getROSParam()
{
  ros::NodeHandle nh("~");

  int param_int_value;
  bool param_bool_value;
  bool exist_param;
  std::string prefix = (g_camera_node_name == "") ? g_camera_node_name : g_camera_node_name + "/";

  //exist_param = nh.getParam(prefix + "brightness", param_int_value);  //0-255
  exist_param = nh.getParam("brightness", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("brightness", param_int_value);
    g_dyn_config.brightness = param_int_value;
    ROS_ERROR("brightness is set");
  }

  exist_param = nh.getParam("contrast", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("contrast", param_int_value);
    g_dyn_config.contrast = param_int_value;
    ROS_ERROR("contrast is set");
  }

  exist_param = nh.getParam(prefix + "saturation", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("saturation", param_int_value);
    g_dyn_config.saturation = param_int_value;
  }

  exist_param = nh.getParam(prefix + "sharpness", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("sharpness", param_int_value);
    g_dyn_config.sharpness = param_int_value;
  }

  exist_param = nh.getParam(prefix + "gain", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("gain", param_int_value);
    g_dyn_config.gain = param_int_value;
  }

  // focus
  exist_param = nh.getParam(prefix + "autofocus", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("focus_auto", param_bool_value);
    g_dyn_config.focus_auto = param_bool_value;

    if (param_bool_value == false)
    {
      int focus_absolute;
      nh.param(prefix + "focus", focus_absolute, -1);  //0-255, -1 "leave alone"
      setV4lParameter("focus_absolute", focus_absolute);
      g_dyn_config.focus_absolute = focus_absolute;
    }
  }

  // exposure
  exist_param = nh.getParam(prefix + "autoexposure", param_bool_value);
  if (exist_param == true)
  {
    // turn down exposure control (from max of 3)
    setV4lParameter("exposure_auto", param_bool_value ? 0 : 1);
    g_dyn_config.exposure_auto = param_bool_value ? 0 : 1;

    if (param_bool_value == false)
    {
      int exposure;
      nh.param(prefix + "exposure", exposure, 100);
      setV4lParameter("exposure_absolute", exposure);
      g_dyn_config.exposure_absolute = exposure;
    }
  }

  // white balance
  exist_param = nh.getParam(prefix + "auto_white_balance", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("white_balance_temperature_auto", param_bool_value);
    g_dyn_config.white_balance_temperature_auto = param_bool_value;

    if (param_bool_value == false)
    {
      int white_balance_temperature;
      nh.param(prefix + "white_balance", white_balance_temperature, 4000);
      setV4lParameter("white_balance_temperature", white_balance_temperature);
      g_dyn_config.white_balance_temperature = white_balance_temperature;
    }
  }
}

void setROSParam(const std::string& v4l_param, const int& value)
{
  std::map<std::string, std::string>::iterator param_iter;

  param_iter = g_param_list.find(v4l_param);

  if (param_iter == g_param_list.end())
    return;

  ros::NodeHandle nh;
  std::string prefix = (g_camera_node_name == "") ? g_camera_node_name : g_camera_node_name + "/";

  if (v4l_param == "exposure_auto")
  {
    nh.setParam(prefix + param_iter->second, (value == 1) ? false : true);
    ROS_INFO("Set param to parameter server : %s = %s", v4l_param.c_str(), (value == 1) ? "false" : "true");
  }
  else
  {
    nh.setParam(prefix + param_iter->second, value);
    ROS_INFO("Set param to parameter server : %s = %d", v4l_param.c_str(), value);
  }
}

// web setting
void paramCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "save")
  {
    saveParameter();
  }
  else if(msg->data == "reset")
  {
    resetParameter();
  }
}

bool setParamCallback(op3_camera_setting_tool::SetParameters::Request &req, op3_camera_setting_tool::SetParameters::Response &res)
{
  setV4lParameter("brightness", req.params.brightness);
  setV4lParameter("contrast", req.params.contrast);
  setV4lParameter("saturation", req.params.saturation);
  setV4lParameter("sharpness", req.params.sharpness);
  setV4lParameter("gain", req.params.gain);
  // focus
  setV4lParameter("focus_auto", req.params.focus_auto);
  if (req.params.focus_auto == false)
  {
    //0-255, -1 "leave alone"
    setV4lParameter("focus_absolute", req.params.focus_absolute);
  }

  // exposure
  // turn down exposure control (from max of 3)
  setV4lParameter("exposure_auto", req.params.exposure_auto);
  if (req.params.exposure_auto == 1)  // if it's manual
  {
    setV4lParameter("exposure_absolute", req.params.exposure_absolute);
  }

  // white balance
  setV4lParameter("white_balance_temperature_auto", req.params.white_balance_temperature_auto);
  if (req.params.white_balance_temperature_auto == false)
  {
    setV4lParameter("white_balance_temperature", req.params.white_balance_temperature);
  }

  res.returns = req.params;

  // save setting value to parameter server
  g_dyn_config.brightness                     = req.params.brightness;
  g_dyn_config.contrast                       = req.params.contrast;
  g_dyn_config.saturation                     = req.params.saturation;
  g_dyn_config.sharpness                      = req.params.sharpness;
  g_dyn_config.gain                           = req.params.gain;
  g_dyn_config.focus_auto                     = req.params.focus_auto;
  g_dyn_config.focus_absolute                 = req.params.focus_absolute;
  g_dyn_config.exposure_auto                  = req.params.exposure_auto;
  g_dyn_config.exposure_absolute              = req.params.exposure_absolute;
  g_dyn_config.white_balance_temperature_auto = req.params.white_balance_temperature_auto;
  g_dyn_config.white_balance_temperature      = req.params.white_balance_temperature;

  updateDynParam(g_dyn_config);

  return true;
}

bool getParamCallback(op3_camera_setting_tool::GetParameters::Request &req, op3_camera_setting_tool::GetParameters::Response &res)
{
  res.returns.brightness                     = g_dyn_config.brightness;
  res.returns.contrast                       = g_dyn_config.contrast;
  res.returns.saturation                     = g_dyn_config.saturation;
  res.returns.sharpness                      = g_dyn_config.sharpness;
  res.returns.gain                           = g_dyn_config.gain;
  res.returns.focus_auto                     = g_dyn_config.focus_auto;
  res.returns.focus_absolute                 = g_dyn_config.focus_absolute;
  res.returns.exposure_auto                  = g_dyn_config.exposure_auto;
  res.returns.exposure_absolute              = g_dyn_config.exposure_absolute;
  res.returns.white_balance_temperature_auto = g_dyn_config.white_balance_temperature_auto;
  res.returns.white_balance_temperature      = g_dyn_config.white_balance_temperature;

  return true;
}

void resetParameter()
{
  // reset parameter


  // publish current parameters
  publishParam();
}

void saveParameter()
{

}

void publishParam()
{
  op3_camera_setting_tool::CameraParams cam_params;

  cam_params.brightness                     = g_dyn_config.brightness;
  cam_params.contrast                       = g_dyn_config.contrast;
  cam_params.saturation                     = g_dyn_config.saturation;
  cam_params.sharpness                      = g_dyn_config.sharpness;
  cam_params.gain                           = g_dyn_config.gain;
  cam_params.focus_auto                     = g_dyn_config.focus_auto;
  cam_params.focus_absolute                 = g_dyn_config.focus_absolute;
  cam_params.exposure_auto                  = g_dyn_config.exposure_auto;
  cam_params.exposure_absolute              = g_dyn_config.exposure_absolute;
  cam_params.white_balance_temperature_auto = g_dyn_config.white_balance_temperature_auto;
  cam_params.white_balance_temperature      = g_dyn_config.white_balance_temperature;

  g_param_pub.publish(cam_params);
}
