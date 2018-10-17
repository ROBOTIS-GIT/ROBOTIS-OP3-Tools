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

/* Author: Jay Song */

#ifndef OP3_ACTION_EDITOR_H_
#define OP3_ACTION_EDITOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <fcntl.h>
#include <ncurses.h>
#include <pthread.h>
#include <sys/wait.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "robotis_controller/robotis_controller.h"
#include "op3_action_module/action_module.h"

#define ROBOT_NAME "OP3"

namespace robotis_op
{

class ActionEditor
{
public:
  enum MirrorCommandType
  {
    LeftToRight = 1,
    RightToLeft = 2,
    SwitchEach = 3
  };
  enum MirrorTargetType
  {
    UpperBody = 1,
    LowerBody = 2,
    AllBody = 3
  };

  ActionEditor();
  ~ActionEditor();

  int _getch();
  int kbhit();

  // Move cursor
  void goToCursor(int col, int row);
  void moveUpCursor();
  void moveDownCursor();
  void moveLeftCursor();
  void moveRightCursor();

  bool initializeActionEditor(std::string robot_file_path, std::string init_file_path, std::string offset_file_path);

  // Disp & Drawing
  void drawIntro();
  void drawEnding();
  void drawPage();
  void drawStep(int index);
  void drawName();
  void drawStepLine(bool erase);
  void readStep();
  void clearCmd();
  void printCmd(const char *message);
  bool askSave();


  // Edit value
  void setValueUpDown(int offset);
  void setValue(int value);
  int  getValue();
  void toggleTorque();
  void storeValueToCache();
  void setValueFromCache();
  void clearCache();

  // Command process
  void beginCommandMode();
  void endCommandMode();
  void helpCmd();
  void nextCmd();
  void prevCmd();
  void pageCmd(int index);
  void timeCmd();
  void speedCmd();
  void playCmd();
  void playCmd(int mp3_index);
  void playCmd(const char* file_path);
  void listCmd();
  void turnOnOffCmd(bool on, int num_param, int *list);
  void mirrorStepCmd(int index, int mirror_type, int target_type);
  void writeStepCmd(int index);
  void deleteStepCmd(int index);
  void insertStepCmd(int index);
  void insertInterpolationStepCmd(int index, int ratio);
  void moveStepCmd(int src, int dst);
  void copyCmd(int index);
  void newCmd();
  void goCmd(int index);
  void saveCmd();
  void nameCmd();

  int screen_col_;
  int screen_row_;

private:
  void setSTDin();
  void resetSTDin();

  int convert4095ToPositionValue(int id, int w4095);
  int convertPositionValueTo4095(int id, int PositionValue);
  int convert4095ToMirror(int id, int w4095);

  bool loadMp3Path(int mp3_index, std::string &path);
  bool loadMirrorJoint();

  struct termios oldterm, new_term;
  ros::Publisher enable_ctrl_module_pub_;
  ros::Publisher play_sound_pub_;

  action_file_define::Page page_;
  action_file_define::Step step_;

  robotis_framework::RobotisController* ctrl_;
  robotis_framework::Robot* robot_;

  std::map<int, std::string> joint_id_to_name_;
  std::map<int, int> joint_id_to_row_index_;
  std::map<int, int> joint_row_index_to_id_;
  std::map<std::string, int> joint_name_to_id_;

  std::map<int, int> upper_body_mirror_joints_rl_;
  std::map<int, int> upper_body_mirror_joints_lr_;
  std::map<int, int> lower_body_mirror_joints_rl_;
  std::map<int, int> lower_body_mirror_joints_lr_;

  std::string default_editor_script_path_;
  std::string mirror_joint_file_path_;

  std::map<std::string, dynamixel::GroupSyncWrite *> port_to_sync_write_go_cmd_;

  bool begin_command_mode_;
  bool edited_;
  int page_idx_;

  int curr_col_ ;
  int curr_row_;
  int old_col_;
  int old_row_;

  int num_of_dxls_;

  int cmd_col_;
  int stp7_col_;
  int stp0_col_;
  int stp1_col_;
  int stp2_col_;
  int stp3_col_;
  int stp4_col_;
  int stp5_col_;
  int stp6_col_;
  int cwslope_col_;
  int ccwslope_col_;
  int name_col_;
  int addr_col_;
  int pagenum_col_;
  int pageparam_col_;

  int max_joint_name_length_;
  int first_joint_row_;
  int last_joint_row_;

  int pause_row_;
  int time_row_;
  int cmd_row_;

  int name_row_;
  int page_num_row_;
  int addr_row_;
  int play_count_row_;
  int step_num_row_;
  int play_time_row_;
  int accel_row_;
  int next_row_;
  int exit_row_;

  int profile_velocity_addr_;
  int cache_value_;

};

}

#endif /* OP3_ACTION_EDITOR_H_ */
