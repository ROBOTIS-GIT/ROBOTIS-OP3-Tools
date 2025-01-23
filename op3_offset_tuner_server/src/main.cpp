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

/* Author: Jay Song*/

#include <rclcpp/rclcpp.hpp>
#include "op3_offset_tuner_server/op3_offset_tuner_server.h"

using namespace robotis_op;

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // TODO: singletone 을 변경해야 하나?
  auto server = std::make_shared<OffsetTunerServer>();
  if (server->initialize() == false)
    return 1;
  
  rclcpp::spin(server);

  rclcpp::shutdown();
  return 0;
}
