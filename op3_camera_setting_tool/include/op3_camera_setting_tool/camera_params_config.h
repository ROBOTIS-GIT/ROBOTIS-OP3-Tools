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

/* Author: Zerom */

#ifndef CAMERA_PARAMS_CONFIG_H_
#define CAMERA_PARAMS_CONFIG_H_

#include <rclcpp/rclcpp.hpp>

namespace robotis_op
{

// constants
const int BRIGHTNESS_DEFAULT = 128;
const int CONTRAST_DEFAULT = 128;
const int SATURATION_DEFAULT = 128;
const int SHARPNESS_DEFAULT = 128;
const int GAIN_DEFAULT = 0;
const bool FOCUS_AUTOMATIC_CONTINUOUIS_DEFAULT = false;
const int FOCUS_ABSOLUTE_DEFAULT = -1;
const int AUTO_EXPOSURE_DEFAULT = 3;
const int EXPOSURE_TIME_ABSOLUTE_DEFAULT = 80;
const bool WHITE_BALANCE_AUTOMATIC_DEFAULT = true;
const int WHITE_BALANCE_TEMPERATURE_DEFAULT = 4000;

std::string param_names[11] = {
  "brightness",
  "contrast",
  "saturation",
  "sharpness",
  "gain",
  "focus_automatic_continuous",
  "focus_absolute",
  "auto_exposure",
  "exposure_time_absolute",
  "white_balance_automatic",
  "white_balance_temperature"
};

class CameraParamsConfig
{
public:
  int brightness;
  int contrast;
  int saturation;
  int sharpness;
  int gain;
  bool focus_automatic_continuous;
  int focus_absolute;
  int auto_exposure;
  int exposure_time_absolute;
  bool white_balance_automatic;
  int white_balance_temperature;

  CameraParamsConfig()
    : brightness(BRIGHTNESS_DEFAULT),
      contrast(CONTRAST_DEFAULT),
      saturation(SATURATION_DEFAULT),
      sharpness(SHARPNESS_DEFAULT),
      gain(GAIN_DEFAULT),
      focus_automatic_continuous(FOCUS_AUTOMATIC_CONTINUOUIS_DEFAULT),
      focus_absolute(FOCUS_ABSOLUTE_DEFAULT),
      auto_exposure(AUTO_EXPOSURE_DEFAULT),
      exposure_time_absolute(EXPOSURE_TIME_ABSOLUTE_DEFAULT),
      white_balance_automatic(WHITE_BALANCE_AUTOMATIC_DEFAULT),
      white_balance_temperature(WHITE_BALANCE_TEMPERATURE_DEFAULT)
  {
  }

  ~CameraParamsConfig()
  {
  }
};

}

#endif /* CAMERA_PARAMS_CONFIG_H_ */