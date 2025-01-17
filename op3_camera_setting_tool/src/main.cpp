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

/* Author: Kayman Jung, Jay Song*/

#include "op3_camera_setting_tool/op3_camera_setting_tool.h"

rclcpp::Node::SharedPtr node;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("op3_camera_setting_tool");

  // get param to use
  node->declare_parameter("video_device", "/dev/video0");
  node->get_parameter("video_device", g_device_name);

  // v4l param name, ros param name
  // g_param_list["brightness"] = "brightness";
  // g_param_list["contrast"] = "contrast";
  // g_param_list["saturation"] = "saturation";
  // g_param_list["sharpness"] = "sharpness";

  // g_param_list["focus_auto"] = "autofocus";
  // g_param_list["focus_absolute"] = "focus";

  // g_param_list["exposure_auto"] = "autoexposure";
  // g_param_list["exposure_absolute"] = "exposure";
  // g_param_list["gain"] = "gain";

  // g_param_list["white_balance_temperature_auto"] = "auto_white_balance";
  // g_param_list["white_balance_temperature"] = "white_balance";

  auto camera_param_sub = node->create_subscription<op3_camera_setting_tool_msgs::msg::V4lParameter>("/op3_camera/set_param", 1, setCameraParameterCallback);
  auto camera_params_sub = node->create_subscription<op3_camera_setting_tool_msgs::msg::V4lParameters>("/op3_camera/set_params", 1, setCameraParametersCallback);

  // web setting
  node->declare_parameter("yaml_path", "");
  g_has_path = node->get_parameter("yaml_path", g_param_path);

  g_param_pub = node->create_publisher<op3_camera_setting_tool_msgs::msg::CameraParams>("/op3_camera/camera_params", 1);
  g_param_command_sub = node->create_subscription<std_msgs::msg::String>("/op3_camera/param_command", 1, paramCommandCallback);
  g_set_param_service = node->create_service<op3_camera_setting_tool_msgs::srv::SetParameters>("/op3_camera/set_camera_params", setParamCallback);
  g_get_param_service = node->create_service<op3_camera_setting_tool_msgs::srv::GetParameters>("/op3_camera/get_camera_params", getParamCallback);
  g_default_setting_path = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME) + "/config/camera_parameter_default.yaml";

  // boost::recursive_mutex config_mutex;

  // get param to set camera
  node->declare_parameter("brightness", robotis_op::BRIGHTNESS_DEFAULT);
  node->declare_parameter("contrast",   robotis_op::CONTRAST_DEFAULT);
  node->declare_parameter("saturation", robotis_op::SATURATION_DEFAULT);
  node->declare_parameter("sharpness", robotis_op::SHARPNESS_DEFAULT);
  node->declare_parameter("gain", robotis_op::GAIN_DEFAULT);
  node->declare_parameter("focus_automatic_continuous", robotis_op::FOCUS_AUTOMATIC_CONTINUOUIS_DEFAULT);
  node->declare_parameter("focus_absolute", robotis_op::FOCUS_ABSOLUTE_DEFAULT);
  node->declare_parameter("auto_exposure", robotis_op::AUTO_EXPOSURE_DEFAULT);
  node->declare_parameter("exposure_time_absolute", robotis_op::EXPOSURE_TIME_ABSOLUTE_DEFAULT);
  node->declare_parameter("white_balance_automatic", robotis_op::WHITE_BALANCE_AUTOMATIC_DEFAULT);
  node->declare_parameter("white_balance_temperature", robotis_op::WHITE_BALANCE_TEMPERATURE_DEFAULT);

  // dynamic reconfigure migration
  // dynamic_reconfigure::Server<op3_camera_setting_tool::CameraParamsConfig>::CallbackType callback_fnc;
  // g_param_server.reset(new dynamic_reconfigure::Server<op3_camera_setting_tool::CameraParamsConfig>(config_mutex));

  // g_param_server->getConfigDefault(g_dyn_config);
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_[11];
  std::shared_ptr<rclcpp::ParameterCallbackHandle> callback_handle_[11];
  for (int i = 0; i < 11; i++)
  {
    param_subscriber_[i] = std::make_shared<rclcpp::ParameterEventHandler>(node);
    callback_handle_[i]  = param_subscriber_[i]->add_parameter_callback(robotis_op::param_names[i], paramCallback);
  }

  getROSParam();

  publishParam();

  //updateDynParam(g_dyn_config);

  // callback_fnc = boost::bind(&dynParamCallback, _1, _2);
  // g_param_server->setCallback(callback_fnc);

  RCLCPP_INFO(node->get_logger(), "start camera setting tool!");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

void changeDynParam(const rclcpp::Parameter& p)
{
  std::string param_name = p.get_name();
  if (param_name == "brightness")
  {
    g_dyn_config.brightness = p.as_int();
  }
  else if (param_name == "contrast")
  {
    g_dyn_config.contrast = p.as_int();
  }
  else if (param_name == "saturation")
  {
    g_dyn_config.saturation = p.as_int();
  }
  else if (param_name == "sharpness")
  {
    g_dyn_config.sharpness = p.as_int();
  }
  else if (param_name == "gain")
  {
    g_dyn_config.gain = p.as_int();
  }
  else if (param_name == "focus_automatic_continuous")
  {
    g_dyn_config.focus_automatic_continuous = p.as_bool();
  }
  else if (param_name == "focus_absolute")
  {
    g_dyn_config.focus_absolute = p.as_int();
  }
  else if (param_name == "auto_exposure")
  {
    g_dyn_config.auto_exposure = p.as_int();
  }
  else if (param_name == "exposure_time_absolute")
  {
    g_dyn_config.exposure_time_absolute = p.as_int();
  }
  else if (param_name == "white_balance_automatic")
  {
    g_dyn_config.white_balance_automatic = p.as_bool();
  }
  else if (param_name == "white_balance_temperature")
  {
    g_dyn_config.white_balance_temperature = p.as_int();
  }
}

void changeDynParam(const std::string& param_name, const int& value)
{
  if (param_name == "brightness")
  {
    g_dyn_config.brightness = value;
  }
  else if (param_name == "contrast")
  {
    g_dyn_config.contrast = value;
  }
  else if (param_name == "saturation")
  {
    g_dyn_config.saturation = value;
  }
  else if (param_name == "sharpness")
  {
    g_dyn_config.sharpness = value;
  }
  else if (param_name == "gain")
  {
    g_dyn_config.gain = value;
  }
  else if (param_name == "focus_automatic_continuous")
  {
    g_dyn_config.focus_automatic_continuous = value;
  }
  else if (param_name == "focus_absolute")
  {
    g_dyn_config.focus_absolute = value;
  }
  else if (param_name == "auto_exposure")
  {
    g_dyn_config.auto_exposure = value;
  }
  else if (param_name == "exposure_time_absolute")
  {
    g_dyn_config.exposure_time_absolute = value;
  }
  else if (param_name == "white_balance_automatic")
  {
    g_dyn_config.white_balance_automatic = value;
  }
  else if (param_name == "white_balance_temperature")
  {
    g_dyn_config.white_balance_temperature = value;
  }
}

void paramCallback(const rclcpp::Parameter& p)
{
  std::string param_name = p.get_name();
  changeDynParam(p);
  
  if (param_name == "brightness")
    setV4lParameter("brightness", g_dyn_config.brightness);
  else if (param_name == "contrast")
    setV4lParameter("contrast", g_dyn_config.contrast);
  else if (param_name == "saturation")
    setV4lParameter("saturation", g_dyn_config.saturation);
  else if (param_name == "sharpness")
    setV4lParameter("sharpness", g_dyn_config.sharpness);
  else if (param_name == "gain")
    setV4lParameter("gain", g_dyn_config.gain);
  else if (param_name == "focus_automatic_continuous") // focus
  {
    setV4lParameter("focus_automatic_continuous", g_dyn_config.focus_automatic_continuous);
    if (g_dyn_config.focus_automatic_continuous == false)
    {
      // 0-255, -1 "leave alone"
      setV4lParameter("focus_absolute", g_dyn_config.focus_absolute);
    }
  }
  else if (param_name == "auto_exposure") // exposure
  {
    // turn down exposure control (from max of 3)
    setV4lParameter("auto_exposure", g_dyn_config.auto_exposure);
    if (g_dyn_config.auto_exposure == 1) // if it's manual
    {
      setV4lParameter("exposure_time_absolute", g_dyn_config.exposure_time_absolute);
    }
  }
  else if (param_name == "white_balance_automatic")  // white balance
  {
    setV4lParameter("white_balance_automatic", g_dyn_config.white_balance_automatic);
    if (g_dyn_config.white_balance_automatic == false)
    {
      setV4lParameter("white_balance_temperature", g_dyn_config.white_balance_temperature);
    }
  }
  else if (param_name == "focus_absolute") // focus
  {
    if (g_dyn_config.focus_automatic_continuous == false)
    {
      // 0-255, -1 "leave alone"
      setV4lParameter("focus_absolute", g_dyn_config.focus_absolute);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Can't change focus_absolute. Check focus_automatic_continuous flag");
    }
  }
  else if (param_name == "exposure_time_absolute") // exposure
  {
    if (g_dyn_config.auto_exposure == 1) // if it's manual
    {
      setV4lParameter("exposure_time_absolute", g_dyn_config.exposure_time_absolute);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Can't change exposure_time_absolute. Check auto_exposure flag");
    }
  }
  else if (param_name == "white_balance_temperature") // exposure
  {
    if (g_dyn_config.white_balance_automatic == false)
    {
      setV4lParameter("white_balance_temperature", g_dyn_config.white_balance_temperature);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Can't change white_balance_temperature. Check white_balance_automatic flag");
    }
  }
}

// void updateDynParam(const std::string& param_name, const int& value)
// {
//   // g_param_server->updateConfig(config);
//   rclcpp::Parameter param(param_name, value);
//   node->set_parameter(param);
// }

void setCameraParameterCallback(const op3_camera_setting_tool_msgs::msg::V4lParameter::SharedPtr msg)
{
  setROSParam(msg->name, msg->value);
  //setV4lParameter(msg->name, msg->value);

  //updateDynParam(msg->name, msg->value);
  saveParameter();
}

void setCameraParametersCallback(const op3_camera_setting_tool_msgs::msg::V4lParameters::SharedPtr msg)
{
  for (int ix = 0; ix < msg->video_parameter.size(); ix++)
  {
    setROSParam(msg->video_parameter[ix].name, msg->video_parameter[ix].value);
    //setV4lParameter(msg->video_parameter[ix].name, msg->video_parameter[ix].value);
    //updateDynParam(msg->video_parameter[ix].name, msg->video_parameter[ix].value);
  }

  saveParameter();
}

void setV4lParameter(const std::string& param_name, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param_name << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  setV4lParameter(cmd);

  //setROSParam(param_name, std::stoi(value));
  changeDynParam(param_name, std::stoi(value));
}

void setV4lParameter(const std::string& param, const int& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", cmd.c_str());
  setV4lParameter(cmd);

  //setROSParam(param, value);
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
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s", output.c_str());
  }
  else
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "usb_cam_node could not run '%s'", cmd.c_str());
}

void getROSParam()
{
  int param_int_value;
  bool param_bool_value;
  bool exist_param;

  RCLCPP_INFO_STREAM(node->get_logger(), "Initial Camera Parameters");

  exist_param = node->get_parameter("brightness", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("brightness", param_int_value);
    g_dyn_config.brightness = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  brightness: " << g_dyn_config.brightness);
  }
  

  exist_param = node->get_parameter("contrast", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("contrast", param_int_value);
    g_dyn_config.contrast = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  contrast: " << g_dyn_config.contrast);
  }

  exist_param = node->get_parameter("saturation", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("saturation", param_int_value);
    g_dyn_config.saturation = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  saturation: " << g_dyn_config.saturation);
  }

  exist_param = node->get_parameter("sharpness", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("sharpness", param_int_value);
    g_dyn_config.sharpness = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  sharpness: " << g_dyn_config.sharpness);
  }

  exist_param = node->get_parameter("gain", param_int_value);  //0-255
  if (exist_param == true)
  {
    setV4lParameter("gain", param_int_value);
    g_dyn_config.gain = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  gain: " << g_dyn_config.gain);
  }

  // focus
  exist_param = node->get_parameter("focus_automatic_continuous", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("focus_automatic_continuous", param_bool_value);
    g_dyn_config.focus_automatic_continuous = param_bool_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  focus_automatic_continuous: " << (g_dyn_config.focus_automatic_continuous ? "TRUE" : "FALSE"));

    if (param_bool_value == false)
    {
      int focus_absolute;
      node->get_parameter_or("focus_absolute", focus_absolute, -1);  //0-255, -1 "leave alone"
      setV4lParameter("focus_absolute", focus_absolute);
      g_dyn_config.focus_absolute = focus_absolute;
      RCLCPP_INFO_STREAM(node->get_logger(), "    Set focus_absolute to " << focus_absolute);
    }
  }

  // exposure
  exist_param = node->get_parameter("auto_exposure", param_int_value);
  if (exist_param == true)
  {
    // turn down exposure control (from max of 3)
    setV4lParameter("auto_exposure", param_int_value);
    g_dyn_config.auto_exposure = param_int_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  auto_exposure: " << g_dyn_config.auto_exposure);

    if (param_int_value == 1)
    {
      int exposure;
      node->get_parameter_or("exposure_time_absolute", exposure, 100);
      setV4lParameter("exposure_time_absolute", exposure);
      g_dyn_config.exposure_time_absolute = exposure;
      RCLCPP_INFO_STREAM(node->get_logger(), "    Set exposure_time_absolute to " << exposure);
    }
  }

  // white balance
  exist_param = node->get_parameter("white_balance_automatic", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("white_balance_automatic", param_bool_value);
    g_dyn_config.white_balance_automatic = param_bool_value;
    RCLCPP_INFO_STREAM(node->get_logger(), "  white_balance_automatic: " << (g_dyn_config.white_balance_automatic ? "TRUE" : "FALSE"));

    if (param_bool_value == false)
    {
      int white_balance_temperature;
      node->get_parameter_or("white_balance_temperature", white_balance_temperature, 4000);
      setV4lParameter("white_balance_temperature", white_balance_temperature);
      g_dyn_config.white_balance_temperature = white_balance_temperature;
      RCLCPP_INFO_STREAM(node->get_logger(), "    Set white_balance_temperature to " << white_balance_temperature);
    }
  }
}

void setROSParam(const std::string& v4l_param, const int& value)
{
  node->set_parameter(rclcpp::Parameter(v4l_param, value));
  RCLCPP_INFO(node->get_logger(), "Set param to parameter server : %s = %d", v4l_param.c_str(), value);
}

// web setting
void paramCommandCallback(const std_msgs::msg::String::SharedPtr msg)
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

bool setParamCallback(const std::shared_ptr<op3_camera_setting_tool_msgs::srv::SetParameters::Request> req,
                      std::shared_ptr<op3_camera_setting_tool_msgs::srv::SetParameters::Response> res)
{
  setV4lParameter("brightness", req->params.brightness);
  setV4lParameter("contrast", req->params.contrast);
  setV4lParameter("saturation", req->params.saturation);
  setV4lParameter("sharpness", req->params.sharpness);
  setV4lParameter("gain", req->params.gain);
  // focus
  setV4lParameter("focus_automatic_continuous", req->params.focus_automatic_continuous);
  if (req->params.focus_automatic_continuous == false)
  {
    //0-255, -1 "leave alone"
    setV4lParameter("focus_absolute", req->params.focus_absolute);
  }

  // exposure
  // turn down exposure control (from max of 3)
  setV4lParameter("auto_exposure", req->params.auto_exposure);
  if (req->params.auto_exposure == 1)  // if it's manual
  {
    setV4lParameter("exposure_time_absolute", req->params.exposure_time_absolute);
  }

  // white balance
  setV4lParameter("white_balance_automatic", req->params.white_balance_automatic);
  if (req->params.white_balance_automatic == false)
  {
    setV4lParameter("white_balance_temperature", req->params.white_balance_temperature);
  }

  res->returns = req->params;

  // save setting value to parameter server
  g_dyn_config.brightness                     = req->params.brightness;
  g_dyn_config.contrast                       = req->params.contrast;
  g_dyn_config.saturation                     = req->params.saturation;
  g_dyn_config.sharpness                      = req->params.sharpness;
  g_dyn_config.gain                           = req->params.gain;
  g_dyn_config.focus_automatic_continuous     = req->params.focus_automatic_continuous;
  g_dyn_config.focus_absolute                 = req->params.focus_absolute;
  g_dyn_config.auto_exposure                  = req->params.auto_exposure;
  g_dyn_config.exposure_time_absolute         = req->params.exposure_time_absolute;
  g_dyn_config.white_balance_automatic        = req->params.white_balance_automatic;
  g_dyn_config.white_balance_temperature      = req->params.white_balance_temperature;

  //updateDynParam(g_dyn_config);
  saveParameter();

  return true;
}

bool getParamCallback(const std::shared_ptr<op3_camera_setting_tool_msgs::srv::GetParameters::Request> req,
                      std::shared_ptr<op3_camera_setting_tool_msgs::srv::GetParameters::Response> res)
{
  res->returns.brightness                     = g_dyn_config.brightness;
  res->returns.contrast                       = g_dyn_config.contrast;
  res->returns.saturation                     = g_dyn_config.saturation;
  res->returns.sharpness                      = g_dyn_config.sharpness;
  res->returns.gain                           = g_dyn_config.gain;
  res->returns.focus_automatic_continuous     = g_dyn_config.focus_automatic_continuous;
  res->returns.focus_absolute                 = g_dyn_config.focus_absolute;
  res->returns.auto_exposure                  = g_dyn_config.auto_exposure;
  res->returns.exposure_time_absolute         = g_dyn_config.exposure_time_absolute;
  res->returns.white_balance_automatic        = g_dyn_config.white_balance_automatic;
  res->returns.white_balance_temperature      = g_dyn_config.white_balance_temperature;

  return true;
}

void resetParameter()
{
  // reset parameter
  try
  {
    // load yaml
      YAML::Node config = YAML::LoadFile(g_default_setting_path.c_str());

    // check node name
    // std::string node_name;
    // if (doc[node->get_name()])
    //   node_name = node->get_name();
    // else if (doc["/**"])
    //   node_name = "/**";
    // else
    // {
    //   RCLCPP_ERROR(node->get_logger(), "Node name '%s' or /** not found in YAML file.", node->get_name());
    //   return;
    // }

    if (!config["/**"])
    {
      RCLCPP_ERROR(node->get_logger(), "Node name '%s' or /** not found in YAML file.", node->get_name());
      return;
    }

    YAML::Node doc = config["/**"]["ros__parameters"];

    // parse
    g_dyn_config.brightness = doc["brightness"].as<int>();
    g_dyn_config.contrast = doc["contrast"].as<int>();
    g_dyn_config.saturation = doc["saturation"].as<int>();
    g_dyn_config.sharpness = doc["sharpness"].as<int>();
    g_dyn_config.gain = doc["gain"].as<int>();
    g_dyn_config.focus_automatic_continuous = doc["focus_automatic_continuous"].as<bool>();
    g_dyn_config.focus_absolute = doc["focus_absolute"].as<int>();
    g_dyn_config.auto_exposure = doc["auto_exposure"].as<int>();
    g_dyn_config.exposure_time_absolute = doc["exposure_time_absolute"].as<int>();
    g_dyn_config.white_balance_automatic = doc["white_balance_automatic"].as<bool>();
    g_dyn_config.white_balance_temperature = doc["white_balance_temperature"].as<int>();

  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to Get default parameters : %s", g_default_setting_path.c_str());
    return;
  }

  // apply the parameters
  setV4lParameter("brightness", g_dyn_config.brightness);
  setV4lParameter("contrast", g_dyn_config.contrast);
  setV4lParameter("saturation", g_dyn_config.saturation);
  setV4lParameter("sharpness", g_dyn_config.sharpness);
  setV4lParameter("gain", g_dyn_config.gain);
  // focus
  setV4lParameter("focus_automatic_continuous", g_dyn_config.focus_automatic_continuous);
  if (g_dyn_config.focus_automatic_continuous == false)
  {
    //0-255, -1 "leave alone"
    setV4lParameter("focus_absolute", g_dyn_config.focus_absolute);
  }

  // exposure
  // turn down exposure control (from max of 3)
  setV4lParameter("auto_exposure", g_dyn_config.auto_exposure);
  if (g_dyn_config.auto_exposure == 1)  // if it's manual
  {
    setV4lParameter("exposure_time_absolute", g_dyn_config.exposure_time_absolute);
  }

  // white balance
  setV4lParameter("white_balance_automatic", g_dyn_config.white_balance_automatic);
  if (g_dyn_config.white_balance_automatic == false)
  {
    setV4lParameter("white_balance_temperature", g_dyn_config.white_balance_temperature);
  }

  //updateDynParam(g_dyn_config);
  saveParameter();

  // publish current parameters
  publishParam();
}

void publishParam()
{
  op3_camera_setting_tool_msgs::msg::CameraParams msg;

  msg.brightness                     = g_dyn_config.brightness;
  msg.contrast                       = g_dyn_config.contrast;
  msg.saturation                     = g_dyn_config.saturation;
  msg.sharpness                      = g_dyn_config.sharpness;
  msg.gain                           = g_dyn_config.gain;
  msg.focus_automatic_continuous     = g_dyn_config.focus_automatic_continuous;
  msg.focus_absolute                 = g_dyn_config.focus_absolute;
  msg.auto_exposure                  = g_dyn_config.auto_exposure;
  msg.exposure_time_absolute         = g_dyn_config.exposure_time_absolute;
  msg.white_balance_automatic        = g_dyn_config.white_balance_automatic;
  msg.white_balance_temperature      = g_dyn_config.white_balance_temperature;

  g_param_pub->publish(msg);
}

void saveParameter()
{
  if (g_has_path == false)
    return;

  YAML::Emitter yaml_out;

  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "/**";             // Node name
  yaml_out << YAML::Value << YAML::BeginMap;  // Start node map
  yaml_out << YAML::Key << "ros__parameters"; // ROS 2 parameters section
  yaml_out << YAML::Value << YAML::BeginMap;  // Start parameters map
  yaml_out << YAML::Key << "brightness" << YAML::Value << g_dyn_config.brightness;
  yaml_out << YAML::Key << "contrast" << YAML::Value << g_dyn_config.contrast;
  yaml_out << YAML::Key << "saturation" << YAML::Value << g_dyn_config.saturation;
  yaml_out << YAML::Key << "sharpness" << YAML::Value << g_dyn_config.sharpness;
  yaml_out << YAML::Key << "gain" << YAML::Value << g_dyn_config.gain;
  yaml_out << YAML::Key << "focus_automatic_continuous" << YAML::Value << g_dyn_config.focus_automatic_continuous;
  yaml_out << YAML::Key << "focus_absolute" << YAML::Value << g_dyn_config.focus_absolute;
  yaml_out << YAML::Key << "auto_exposure" << YAML::Value << g_dyn_config.auto_exposure;
  yaml_out << YAML::Key << "exposure_time_absolute" << YAML::Value << g_dyn_config.exposure_time_absolute;
  yaml_out << YAML::Key << "white_balance_automatic" << YAML::Value << g_dyn_config.white_balance_automatic;
  yaml_out << YAML::Key << "white_balance_temperature" << YAML::Value << g_dyn_config.white_balance_temperature;
  yaml_out << YAML::EndMap; // End parameters map
  yaml_out << YAML::EndMap; // End node map
  yaml_out << YAML::EndMap; // End main map

  // output to file
  std::ofstream fout(g_param_path.c_str());
  fout << yaml_out.c_str();
}
