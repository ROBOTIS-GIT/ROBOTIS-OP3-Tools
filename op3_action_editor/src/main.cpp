/*******************************************************************************
* Copyright 201 ROBOTIS CO., LTD.
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

/* Author: JaySong, Kayman Jung */

#include "op3_action_editor/action_editor.h"

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;

void sighandler(int sig)
{
  struct termios term;
  tcgetattr( STDIN_FILENO, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr( STDIN_FILENO, TCSANOW, &term);

  system("clear");
  exit(0);
}

bool turnOnDynamixelPower(rclcpp::Node::SharedPtr node, const std::string &device_name, const int &baud_rate)
{
  // power on
  dynamixel::PortHandler *_port_h = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler(device_name.c_str());
  bool _set_port = _port_h->setBaudRate(baud_rate);
  if (_set_port == false)
  {
    RCLCPP_ERROR(node->get_logger(), "Error Set port");
    return false;
  }
  dynamixel::PacketHandler *_packet_h = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int _return = _packet_h->write1ByteTxRx(_port_h, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

  if(_return != COMM_SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to turn on the Power of DXLs!");
    return false;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Power on DXLs!");
  }

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto editor = std::make_shared<robotis_op::ActionEditor>();

  std::string offset_file;
  std::string robot_file;
  std::string dxl_init_file;
  std::string _device_name;
  int _baud_rate;

  editor->declare_parameter<std::string>("offset_file_path", "");
  editor->declare_parameter<std::string>("robot_file_path", "");
  editor->declare_parameter<std::string>("init_file_path", "");
  editor->declare_parameter<std::string>("device_name", SUB_CONTROLLER_DEVICE);
  editor->declare_parameter<int>("baud_rate", BAUD_RATE);
  editor->declare_parameter<bool>("gazebo", false);

  editor->get_parameter("offset_file_path", offset_file);
  editor->get_parameter("robot_file_path", robot_file);
  editor->get_parameter("init_file_path", dxl_init_file);
  editor->get_parameter("device_name", _device_name);
  editor->get_parameter("baud_rate", _baud_rate);
  bool simulation_mode = false;
  editor->get_parameter("gazebo", simulation_mode);

  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  int ch;

  if(simulation_mode == false)
  {
    if (turnOnDynamixelPower(editor, _device_name, _baud_rate) == false)
      return 0;
  }
  else
  {
    RCLCPP_WARN(editor->get_logger(), "SIMULATION_MODE");
  }

  if (editor->initializeActionEditor(robot_file, dxl_init_file, offset_file, simulation_mode) == false)
  {
    RCLCPP_ERROR(editor->get_logger(), "Failed to Initialize");
    return 0;
  }

  // disable ros log
  editor->disableInfoLogger();

  editor->drawIntro();

  while (rclcpp::ok())
  {
    ch = editor->_getch();

    if (ch == 0x1b)
    {
      ch = editor->_getch();
      if (ch == 0x5b)
      {
        ch = editor->_getch();
        if (ch == 0x41) // Up arrow key
          editor->moveUpCursor();
        else if (ch == 0x42) // Down arrow key
          editor->moveDownCursor();
        else if (ch == 0x44) // Left arrow key
          editor->moveLeftCursor();
        else if (ch == 0x43) // Right arrow key
          editor->moveRightCursor();
      }
    }
    else if (ch == '[')
      editor->setValueUpDown(-1);
    else if (ch == ']')
      editor->setValueUpDown(1);
    else if (ch == '{')
      editor->setValueUpDown(-10);
    else if (ch == '}')
      editor->setValueUpDown(10);
    else if (ch == ' ')
      editor->toggleTorque();
    else if (ch == ',')
      editor->storeValueToCache();
    else if (ch == '.')
      editor->setValueFromCache();
    else if (ch == '/')
      editor->clearCache();
    else if (ch >= 'A' && ch <= 'z')
    {
      char input[128] = {0,};
      char *token;
      int input_len;
      char cmd[80];
      int num_param;
      int iparam[30];

      int idx = 0;

      editor->beginCommandMode();

      printf("%c", ch);
      input[idx++] = (char)ch;

      while (1)
      {
        ch = editor->_getch();
        if (ch == 0x0A)
          break;
        else if (ch == 0x7F)
        {
          if (idx > 0)
          {
            ch = 0x08;
            printf("%c", ch);
            ch = ' ';
            printf("%c", ch);
            ch = 0x08;
            printf("%c", ch);
            input[--idx] = 0;
          }
        }
        else if ((ch >= 'A' && ch <= 'z') || ch == ' ' || (ch >= '0' && ch <= '9'))
        {
          if (idx < 127)
          {
            printf("%c", ch);
            input[idx++] = (char)ch;
          }
        }
      }

      fflush(stdin);
      input_len = strlen(input);
      if (input_len > 0)
      {
        token = strtok(input, " ");
        if (token != 0)
        {
          strcpy(cmd, token);
          token = strtok(0, " ");
          num_param = 0;
          while (token != 0)
          {
            iparam[num_param++] = atoi(token);
            token = strtok(0, " ");
          }

          if (strcmp(cmd, "exit") == 0)
          {
            if (editor->askSave() == false)
              break;
          }
          else if (strcmp(cmd, "re") == 0)
            editor->drawPage();
          else if (strcmp(cmd, "help") == 0)
            editor->helpCmd();
          else if (strcmp(cmd, "n") == 0)
            editor->nextCmd();
          else if (strcmp(cmd, "b") == 0)
            editor->prevCmd();
          else if (strcmp(cmd, "time") == 0)
            editor->timeCmd();
          else if (strcmp(cmd, "speed") == 0)
            editor->speedCmd();
          else if (strcmp(cmd, "page") == 0)
          {
            if (num_param > 0)
              editor->pageCmd(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "play") == 0)
          {
            editor->playCmd();
          }
          else if (strcmp(cmd, "playboth") == 0)
          {
            if (num_param > 0)
              editor->playCmd(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "set") == 0)
          {
            if (num_param > 0)
              editor->setValue(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "list") == 0)
            editor->listCmd();
          else if (strcmp(cmd, "on") == 0)
            editor->turnOnOffCmd(true, num_param, iparam);
          else if (strcmp(cmd, "off") == 0)
            editor->turnOnOffCmd(false, num_param, iparam);
          else if (strcmp(cmd, "mrl") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                   robotis_op::ActionEditor::AllBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "murl") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                   robotis_op::ActionEditor::UpperBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "mlrl") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                   robotis_op::ActionEditor::LowerBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "mlr") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                   robotis_op::ActionEditor::AllBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "mulr") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                   robotis_op::ActionEditor::UpperBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "mllr") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                   robotis_op::ActionEditor::LowerBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "ms") == 0)
          {
            if (num_param > 0)
              editor->mirrorStepCmd(iparam[0], robotis_op::ActionEditor::SwitchEach,
                                   robotis_op::ActionEditor::AllBody);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "w") == 0)
          {
            if (num_param > 0)
              editor->writeStepCmd(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "d") == 0)
          {
            if (num_param > 0)
              editor->deleteStepCmd(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "m") == 0)
          {
            if (num_param > 1)
              editor->moveStepCmd(iparam[0], iparam[1]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "i") == 0)
          {
            if (num_param == 0)
              editor->insertStepCmd(0);
            else
              editor->insertStepCmd(iparam[0]);
          }
          else if (strcmp(cmd, "int") == 0)
          {
            if (num_param == 2)
              editor->insertInterpolationStepCmd(iparam[0], iparam[1]);
            else
              editor->printCmd("Need 2 parameters");
          }
          else if (strcmp(cmd, "copy") == 0)
          {
            if (num_param > 0)
              editor->copyCmd(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "new") == 0)
            editor->newCmd();
          else if (strcmp(cmd, "g") == 0)
          {
            if (num_param > 0)
              editor->goCmd_2(iparam[0]);
            else
              editor->printCmd("Need parameter");
          }
          else if (strcmp(cmd, "save") == 0)
            editor->saveCmd();
          else if (strcmp(cmd, "name") == 0)
            editor->nameCmd();
          else
            editor->printCmd("Bad command! please input 'help'");
        }
      }

      editor->endCommandMode();
    }
  }

  editor->drawEnding();

  return 0;
}
