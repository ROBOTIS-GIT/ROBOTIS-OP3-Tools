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

#include "op3_offset_tuner_server/op3_offset_tuner_server.h"

#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

namespace robotis_op
{

OffsetTunerServer::OffsetTunerServer()
{
  offset_file_ = "";
  robot_file_ = "";
  init_file_ = "";
  controller_ = 0;
}

OffsetTunerServer::~OffsetTunerServer()
{

}

void OffsetTunerServer::setCtrlModule(std::string module)
{
  ros::NodeHandle ros_node;
  ros::Publisher set_ctrl_module_pub = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  std_msgs::String control_msg;

  control_msg.data = module;
  set_ctrl_module_pub.publish(control_msg);
}

bool OffsetTunerServer::initialize()
{
  // torque on dxls
  controller_ = robotis_framework::RobotisController::getInstance();

  dynamixel::PortHandler *port_handler = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler(
        SUB_CONTROLLER_DEVICE);
  bool set_port = port_handler->setBaudRate(BAUD_RATE);
  if (set_port == false)
    ROS_ERROR("Error Set port");
  dynamixel::PacketHandler *packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int return_data = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

  if(return_data != 0)
    ROS_ERROR("Torque on DXLs! [%s]", packet_handler->getRxPacketError(return_data));
  else
    ROS_INFO("Torque on DXLs!");

  port_handler->closePort();

  ros::NodeHandle ros_node;

  //Get File Path
  ros_node.param<std::string>("offset_path", offset_file_, "");
  ros_node.param<std::string>("robot_file_path", robot_file_, "");
  ros_node.param<std::string>("init_file_path", init_file_, "");

  if ((offset_file_ == "") || (robot_file_ == ""))
  {
    ROS_ERROR("Failed to get file path");
    return -1;
  }

  //Controller Initialize with robot file info
  if (controller_->initialize(robot_file_, init_file_) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }
  //controller_->LoadOffset(offset_file_);
  controller_->addMotionModule((robotis_framework::MotionModule*) BaseModule::getInstance());

  //Initialize RobotOffsetData
  for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = controller_->robot_->dxls_.begin();
       dxls_it != controller_->robot_->dxls_.end(); dxls_it++)
  {
    std::string joint_name = dxls_it->first;
    robot_offset_data_[joint_name] = new JointOffsetData(0, 0);
    robot_torque_enable_data_[joint_name] = true;
  }

  //Load Offset.yaml
  getInitPose(offset_file_);

  ROS_INFO(" ");
  ROS_INFO(" ");
  //For Data Check, Print All Available Data
  int i = 0;
  ROS_INFO_STREAM("num" <<" | "<<"joint_name" << " | " << "InitPose" << ", " << "OffsetAngleRad");
  for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
       map_it != robot_offset_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    ROS_INFO_STREAM(
          i <<" | "<<joint_name << " : " << joint_data->joint_init_pos_rad_ << ", " << joint_data->joint_offset_rad_);
    i++;
  }

  send_tra_sub_ = ros_node.subscribe("/robotis/base/send_tra", 10, &OffsetTunerServer::stringMsgsCallBack, this);
  joint_offset_data_sub_ = ros_node.subscribe("/robotis/offset_tuner/joint_offset_data", 10,
                                              &OffsetTunerServer::jointOffsetDataCallback, this);
  joint_torque_enable_sub_ = ros_node.subscribe("/robotis/offset_tuner/torque_enable", 10,
                                                &OffsetTunerServer::jointTorqueOnOffCallback, this);
  command_sub_ = ros_node.subscribe("/robotis/offset_tuner/command", 5, &OffsetTunerServer::commandCallback, this);
  offset_data_server_ = ros_node.advertiseService("robotis/offset_tuner/get_present_joint_offset_data",
                                                  &OffsetTunerServer::getPresentJointOffsetDataServiceCallback, this);

  return true;
}

void OffsetTunerServer::moveToInitPose()
{
  getInitPose(offset_file_);

  controller_->startTimer();
  BaseModule *base_module = BaseModule::getInstance();

  //make map, key : joint_name, value : joint_init_pos_rad;
  std::map<std::string, double> offset_init_pose;
  for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
       map_it != robot_offset_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    if (controller_->robot_->dxls_.find(joint_name) == controller_->robot_->dxls_.end())
    {
      continue;
    }
    else
    {
      offset_init_pose[joint_name] = joint_data->joint_init_pos_rad_ + joint_data->joint_offset_rad_;
    }
  }

  usleep(80 * 1000);
  base_module->poseGenerateProc(offset_init_pose);
  usleep(10 * 000);

  while (base_module->isRunning())
    usleep(50 * 1000);

  controller_->stopTimer();
  while (controller_->isTimerRunning())
    usleep(10 * 1000);

  if (controller_->isTimerRunning())
  {
    ROS_INFO("Timer Running");
  }

  setCtrlModule("none");
}

void OffsetTunerServer::stringMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->data);
}

void OffsetTunerServer::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "save")
  {
    YAML::Emitter yaml_out;
    std::map<std::string, double> offset;
    std::map<std::string, double> init_pose;
    for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
         map_it != robot_offset_data_.end(); map_it++)
    {
      std::string joint_name = map_it->first;
      JointOffsetData* joint_data = map_it->second;

      offset[joint_name] = joint_data->joint_offset_rad_;  // edit one of the nodes
      init_pose[joint_name] = joint_data->joint_init_pos_rad_;  // edit one of the nodes
    }

    yaml_out << YAML::BeginMap;
    yaml_out << YAML::Key << "offset" << YAML::Value << offset;
    yaml_out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << init_pose;
    yaml_out << YAML::EndMap;
    std::ofstream fout(offset_file_.c_str());
    fout << yaml_out.c_str();  // dump it back into the file

  }
  else if (msg->data == "ini_pose")
  {
    moveToInitPose();
  }
  else
  {
    ROS_INFO_STREAM("Invalid Command : " << msg->data);
  }
}

void OffsetTunerServer::jointOffsetDataCallback(const op3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg)
{
  if (controller_->isTimerRunning())
  {
    ROS_ERROR("Timer is running now");
    return;
  }

  //goal position
  ROS_INFO_STREAM(
        msg->joint_name << " " << msg->goal_value << " " << msg->offset_value << " " << msg->p_gain <<" " << msg->i_gain <<" " << msg->d_gain);

  std::map<std::string, JointOffsetData*>::iterator map_it;
  map_it = robot_offset_data_.find(msg->joint_name);
  if (map_it == robot_offset_data_.end())
  {
    ROS_ERROR("Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
    return;
  }

  double goal_pose_rad = msg->offset_value + msg->goal_value;
  uint32_t goal_pose_value = controller_->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
  uint8_t dxl_error = 0;
  uint32_t comm_result = COMM_SUCCESS;
  comm_result = controller_->writeCtrlItem(msg->joint_name,
                                           controller_->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
      goal_pose_value, &dxl_error);
  if (comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to write goal position");
    return;
  }
  else
  {
    robot_offset_data_[msg->joint_name]->joint_init_pos_rad_ = msg->goal_value;
    robot_offset_data_[msg->joint_name]->joint_offset_rad_ = msg->offset_value;
  }

  if (dxl_error != 0)
  {
    ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error "<< (int)dxl_error);
  }

  robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
  robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
  robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

}

void OffsetTunerServer::jointTorqueOnOffCallback(const op3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg)
{
  for (unsigned int i = 0; i < msg->torque_enable_data.size(); i++)
  {
    std::string joint_name = msg->torque_enable_data[i].joint_name;
    bool torque_enable = msg->torque_enable_data[i].torque_enable;
    ROS_INFO_STREAM(i <<" " << joint_name << torque_enable);

    std::map<std::string, JointOffsetData*>::iterator map_it;
    map_it = robot_offset_data_.find(joint_name);
    if (map_it == robot_offset_data_.end())
    {
      ROS_ERROR("Invalid Joint Name");
      continue;
    }
    else
    {
      int32_t comm_result = COMM_SUCCESS;
      uint8_t dxl_error = 0;
      uint8_t torque_enable_value = 0;

      if (torque_enable)
        torque_enable_value = 1;
      else
        torque_enable_value = 0;

      comm_result = controller_->writeCtrlItem(joint_name,
                                               controller_->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
                                               torque_enable_value, &dxl_error);
      if (comm_result != COMM_SUCCESS)
      {
        ROS_ERROR("Failed to write goal position");
      }
      else
      {
        robot_torque_enable_data_[joint_name] = torque_enable;
      }

      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error "<< (int)dxl_error);
      }
    }
  }

}

bool OffsetTunerServer::getPresentJointOffsetDataServiceCallback(
    op3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
    op3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res)
{

  ROS_INFO("GetPresentJointOffsetDataService Called");

  for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
       map_it != robot_offset_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    op3_offset_tuner_msgs::JointOffsetPositionData joint_offset_pos;

    int32_t present_pos_value = 0;
    uint8_t dxl_error = 0;
    int comm_result = COMM_SUCCESS;

    if (controller_->robot_->dxls_[joint_name]->present_position_item_ == NULL)
      continue;

    comm_result = controller_->readCtrlItem(joint_name,
                                            controller_->robot_->dxls_[joint_name]->present_position_item_->item_name_,
                                            (uint32_t*) &present_pos_value, &dxl_error);
    if (comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to read present pos");
      return false;
    }
    else
    {
      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM(joint_name << "  has error "<< (int)dxl_error);
      }

      joint_offset_pos.joint_name = joint_name;
      joint_offset_pos.goal_value = joint_data->joint_init_pos_rad_;
      joint_offset_pos.offset_value = joint_data->joint_offset_rad_;
      joint_offset_pos.present_value = controller_->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value);
      joint_offset_pos.p_gain = joint_data->p_gain_;
      joint_offset_pos.i_gain = joint_data->i_gain_;
      joint_offset_pos.d_gain = joint_data->d_gain_;

      res.present_data_array.push_back(joint_offset_pos);
    }
  }
  return true;
}

void OffsetTunerServer::getInitPose(const std::string &path)
{

  YAML::Node offset_yaml_node;
  bool has_offset_file = false;
  try
  {
    // load yaml
    offset_yaml_node = YAML::LoadFile(path.c_str());
    has_offset_file = true;
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load offset yaml file.");
    has_offset_file = false;
  }

  if (has_offset_file)
  {
    //Get Offset Data and Init_Pose for Offset Tuning
    YAML::Node offset_data_node = offset_yaml_node[OFFSET_ROSPARAM_KEY];

    //Initialize Offset Data in RobotOffsetData
    for (YAML::const_iterator node_it = offset_data_node.begin(); node_it != offset_data_node.end(); node_it++)
    {
      std::string joint_name = node_it->first.as<std::string>();
      double offset = node_it->second.as<double>();

      std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
      if (robot_offset_data_it != robot_offset_data_.end())
        robot_offset_data_[joint_name]->joint_offset_rad_ = offset;
    }


    YAML::Node offset_init_pose_node = offset_yaml_node[OFFSET_INIT_POS_ROSPARAM_KEY];

    //Initialize Init Pose for Offset Tuning in RobotOffsetData
    for (YAML::const_iterator node_it = offset_init_pose_node.begin(); node_it != offset_init_pose_node.end();
         node_it++)
    {
      std::string joint_name = node_it->first.as<std::string>();
      double offset_init_pose = node_it->second.as<double>();

      std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
      if (robot_offset_data_it != robot_offset_data_.end())
        robot_offset_data_[joint_name]->joint_init_pos_rad_ = offset_init_pose;
    }
  }
  else
  {
    double default_offset_value = 0.0;

    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = controller_->robot_->dxls_.begin();
         dxls_it != controller_->robot_->dxls_.end(); dxls_it++)
    {
      std::string joint_name = dxls_it->first;
      robot_offset_data_[joint_name]->joint_offset_rad_ = default_offset_value;
      robot_offset_data_[joint_name]->joint_init_pos_rad_ = default_offset_value;
    }
  }

  return;
}
}
