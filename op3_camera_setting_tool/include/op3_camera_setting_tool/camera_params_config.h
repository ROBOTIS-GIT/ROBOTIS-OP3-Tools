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
const bool FOCUS_AUTO_DEFAULT = false;
const int FOCUS_ABSOLUTE_DEFAULT = -1;
const int EXPOSURE_AUTO_DEFAULT = 3;
const int EXPOSURE_ABSOLUTE_DEFAULT = 80;
const bool WHITE_BALANCE_TEMPERATURE_AUTO_DEFAULT = true;
const int WHITE_BALANCE_TEMPERATURE_DEFAULT = 4000;

class CameraParamsConfig
{
public:
  int brightness;
  int contrast;
  int saturation;
  int sharpness;
  int gain;
  bool focus_auto;
  int focus_absolute;
  int exposure_auto;
  int exposure_absolute;
  bool white_balance_temperature_auto;
  int white_balance_temperature;

  CameraParamsConfig()
    : brightness(BRIGHTNESS_DEFAULT),
      contrast(CONTRAST_DEFAULT),
      saturation(SATURATION_DEFAULT),
      sharpness(SHARPNESS_DEFAULT),
      gain(GAIN_DEFAULT),
      focus_auto(FOCUS_AUTO_DEFAULT),
      focus_absolute(FOCUS_ABSOLUTE_DEFAULT),
      exposure_auto(EXPOSURE_AUTO_DEFAULT),
      exposure_absolute(EXPOSURE_ABSOLUTE_DEFAULT),
      white_balance_temperature_auto(WHITE_BALANCE_TEMPERATURE_AUTO_DEFAULT),
      white_balance_temperature(WHITE_BALANCE_TEMPERATURE_DEFAULT)
  {
  }

  ~CameraParamsConfig()
  {
  }
};

}

#endif /* CAMERA_PARAMS_CONFIG_H_ */