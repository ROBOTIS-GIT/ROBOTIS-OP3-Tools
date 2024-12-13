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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "../include/op3_gui_demo/qnode.hpp"

namespace robotis_op
{

void QNodeOP3::init_preview_walking(rclcpp::Node::SharedPtr ros_node)
{
  // // preview walking
  // foot_step_command_pub_ = ros_node->create_publisher<op3_online_walking_module_msgs::msg::FootStepCommand>("/robotis/online_walking/foot_step_command", 10);
  // walking_param_pub_ = ros_node->create_publisher<op3_online_walking_module_msgs::msg::WalkingParam>("/robotis/online_walking/walking_param", 10);
  // set_walking_footsteps_pub_ = ros_node->create_publisher<op3_online_walking_module_msgs::msg::Step2DArray>("/robotis/online_walking/footsteps_2d", 10);

  // body_offset_pub_ = ros_node->create_publisher<geometry_msgs::msg::Pose>("/robotis/online_walking/body_offset", 10);
  // foot_distance_pub_ = ros_node->create_publisher<std_msgs::msg::Float64>("/robotis/online_walking/foot_distance", 10);
  // wholebody_balance_pub_ = ros_node->create_publisher<std_msgs::msg::String>("/robotis/online_walking/wholebody_balance_msg", 10);
  // reset_body_msg_pub_ = ros_node->create_publisher<std_msgs::msg::Bool>("/robotis/online_walking/reset_body", 10);
  // joint_pose_msg_pub_ = ros_node->create_publisher<op3_online_walking_module_msgs::msg::JointPose>("/robotis/online_walking/goal_joint_pose", 10);

  // // humanoid_footstep_client_ = ros_node->create_client<humanoid_nav_msgs::srv::PlanFootsteps>("plan_footsteps");
  // marker_pub_ = ros_node->create_publisher<visualization_msgs::msg::MarkerArray>("/robotis/demo/foot_step_marker", 10);

  // // interactive marker
  // rviz_clicked_point_sub_ = ros_node->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 10, std::bind(&QNodeOP3::pointStampedCallback, this, std::placeholders::_1));
  // interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("Feet_Pose", "", false));

  // RCLCPP_INFO(ros_node->get_logger(), "Initialized node handle for preview walking");
}

bool QNodeOP3::transformPose(const std::string &from_id, const std::string &to_id, const geometry_msgs::msg::Pose &from_pose, geometry_msgs::msg::Pose &to_pose, bool inverse)
{
  // geometry_msgs::msg::TransformStamped desired_transform;

  // try
  // {
  //   tf_listener_->lookupTransform(from_id, to_id, tf2::TimePointZero, desired_transform);
  //   Eigen::Vector3d transform_position(desired_transform.getOrigin().x(),
  //                                      desired_transform.getOrigin().y(),
  //                                      desired_transform.getOrigin().z());
  //   Eigen::Quaterniond transform_orientation(desired_transform.getRotation().w(),
  //                                            desired_transform.getRotation().x(),
  //                                            desired_transform.getRotation().y(),
  //                                            desired_transform.getRotation().z());

  //   Eigen::Vector3d before_position, after_position;
  //   Eigen::Quaterniond before_orientation, after_orientation;

  //   tf2::fromMsg(from_pose.position, before_position);
  //   tf2::fromMsg(from_pose.orientation, before_orientation);

  //   // default : world to local
  //   if(inverse == false)
  //   {
  //     after_position = transform_orientation.inverse().toRotationMatrix() * (before_position - transform_position);
  //     after_orientation = before_orientation * transform_orientation.inverse();
  //   }
  //   else
  //   {
  //     after_position = transform_orientation.toRotationMatrix() * before_position + transform_position;
  //     after_orientation = transform_orientation * before_orientation;
  //   }

  //   to_pose.position = tf2::toMsg(after_position);
  //   to_pose.orientation = tf2::toMsg(after_orientation);
  // }
  // catch (tf2::TransformException &ex)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "%s", ex.what());
  //   return false;
  // }

  return true;
}

bool QNodeOP3::updateInteractiveMarker(const geometry_msgs::msg::Pose &pose)
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Update Interactive Marker Pose");

  // visualization_msgs::msg::InteractiveMarker interactive_marker;
  // bool result_getting = false;

  // result_getting = interactive_marker_server_->get(marker_name_, interactive_marker);
  // if (result_getting == false)
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No Interactive marker to set pose");
  //   return false;
  // }

  // // transform : local to world
  // geometry_msgs::msg::Pose world_pose;
  // bool result = transformPose("/world", "/body_link", pose, world_pose, true);
  // if(result == false) world_pose = pose;

  // interactive_marker_server_->setPose(interactive_marker.name, world_pose);
  // interactive_marker_server_->applyChanges();

  return true;
}

void QNodeOP3::getInteractiveMarkerPose()
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Interactive Marker Pose");

  // visualization_msgs::msg::InteractiveMarker _interactive_marker;
  // if (!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
  // {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No Interactive marker to get pose");
  //   return;
  // }

  // // transform : world to local
  // geometry_msgs::msg::Pose local_pose;
  // bool result = transformPose("/world", "/body_link", _interactive_marker.pose, local_pose);
  // if(result == false) local_pose = _interactive_marker.pose;

  // // update pose ui
  // Q_EMIT updateDemoPose(local_pose);

  // clearInteractiveMarker();
}

void QNodeOP3::clearInteractiveMarker()
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Clear Interactive Marker");

  // // clear and apply
  // interactive_marker_server_->clear();
  // interactive_marker_server_->applyChanges();
}

// footstep
void QNodeOP3::setWalkingFootsteps(const double &step_time)
{
  if (preview_foot_steps_.size() != preview_foot_types_.size())
  {
    log(Error, "Footsteps are corrupted.");
    return;
  }
  else if (preview_foot_steps_.size() == 0)
  {
    log(Warn, "No Footsteps");
    return;
  }

  op3_online_walking_module_msgs::msg::Step2DArray footsteps;

  for (int ix = 0; ix < preview_foot_steps_.size(); ix++)
  {
    op3_online_walking_module_msgs::msg::Step2D step;

    step.moving_foot = preview_foot_types_[ix];
    step.step2d = preview_foot_steps_[ix];

    footsteps.footsteps_2d.push_back(step);
  }

  footsteps.step_time = step_time;

  set_walking_footsteps_pub_->publish(footsteps);

  log(Info, "Set command to walk using footsteps");

  //clearFootsteps();
}

void QNodeOP3::clearFootsteps()
{
  // clear foot step marker array
  visualizePreviewFootsteps(true);

  preview_foot_steps_.clear();
  preview_foot_types_.clear();
}

void QNodeOP3::makeFootstepUsingPlanner()
{
  makeFootstepUsingPlanner(current_pose_);
}

void QNodeOP3::makeFootstepUsingPlanner(const geometry_msgs::msg::Pose &target_foot_pose)
{
  // //foot step service
  // auto request = std::make_shared<humanoid_nav_msgs::srv::PlanFootsteps::Request>();

  // geometry_msgs::msg::Pose2D start;
  // geometry_msgs::msg::Pose2D goal;
  // goal.x = target_foot_pose.position.x;
  // goal.y = target_foot_pose.position.y;

  // Eigen::Quaterniond goal_orientation;
  // tf2::fromMsg(target_foot_pose.orientation, goal_orientation);

  // Eigen::Vector3d forward, f_x(1, 0, 0);
  // forward = goal_orientation.toRotationMatrix() * f_x;
  // double theta = forward.y() > 0 ? acos(forward.x()) : -acos(forward.x());
  // goal.theta = theta;

  // request->start = start;
  // request->goal = goal;

  // std::stringstream call_msg;
  // call_msg << "Start [" << start.x << ", " << start.y << " | " << start.theta << "]" << " , Goal [" << goal.x << ", "
  //          << goal.y << " | " << goal.theta << "]";
  // log(Info, call_msg.str());

  // // clear visualization
  // visualizePreviewFootsteps(true);

  // // init foot steps
  // preview_foot_steps_.clear();
  // preview_foot_types_.clear();

  // auto result = humanoid_footstep_client_->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(ros_node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   auto response = result.get();
  //   if (response->result)
  //   {
  //     for (int ix = 0; ix < response->footsteps.size(); ix++)
  //     {
  //       // foot step log
  //       int type = response->footsteps[ix].leg;
  //       int foot_type;
  //       std::string foot_string;
  //       if (type == humanoid_nav_msgs::msg::StepTarget::right)
  //       {
  //         foot_type = op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING;
  //         foot_string = "right";
  //       }
  //       else if (type == humanoid_nav_msgs::msg::StepTarget::left)
  //       {
  //         foot_type = op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING;
  //         foot_string = "left";
  //       }
  //       else
  //         foot_type = op3_online_walking_module_msgs::msg::Step2D::STANDING;

  //       std::stringstream msg_stream;
  //       geometry_msgs::msg::Pose2D foot_pose = response->footsteps[ix].pose;

  //       // log footsteps
  //       msg_stream << "Foot Step #" << ix + 1 << " [ " << foot_string << "] - [" << foot_pose.x << ", " << foot_pose.y
  //                  << " | " << (foot_pose.theta * 180 / M_PI) << "]";
  //       log(Info, msg_stream.str());

  //       preview_foot_steps_.push_back(foot_pose);
  //       preview_foot_types_.push_back(foot_type);
  //     }

  //     double y_feet_offset = 0.186;
  //     ros_node_->get_parameter("/footstep_planner/foot/separation", y_feet_offset);
  //     geometry_msgs::msg::Pose2D target_r_foot_pose, target_l_foot_pose;
  //     target_r_foot_pose.x = goal.x - (-0.5*y_feet_offset)*sin(theta);
  //     target_r_foot_pose.y = goal.y + (-0.5*y_feet_offset)*cos(theta);
  //     target_r_foot_pose.theta = theta;

  //     target_l_foot_pose.x = goal.x - ( 0.5*y_feet_offset)*sin(theta);
  //     target_l_foot_pose.y = goal.y + ( 0.5*y_feet_offset)*cos(theta);
  //     target_l_foot_pose.theta = theta;

  //     if(preview_foot_types_[preview_foot_types_.size() - 1] == op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING)
  //     {
  //       preview_foot_steps_.push_back(target_l_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING);
  //       preview_foot_steps_.push_back(target_r_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING);
  //       preview_foot_steps_.push_back(target_l_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING);
  //     }
  //     else if(preview_foot_types_[preview_foot_types_.size() - 1] == op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING)
  //     {
  //       preview_foot_steps_.push_back(target_r_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING);
  //       preview_foot_steps_.push_back(target_l_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING);
  //     }
  //     else
  //     {
  //       preview_foot_steps_.push_back(target_r_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING);
  //       preview_foot_steps_.push_back(target_l_foot_pose);
  //       preview_foot_types_.push_back(op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING);
  //     }

  //     // visualize foot steps
  //     visualizePreviewFootsteps(false);
  //   }
  //   else
  //   {
  //     log(Info, "fail to get foot step from planner");
  //     return;
  //   }
  // }
  // else
  // {
  //   log(Error, "cannot call service");
  //   return;
  // }

  return;
}

void QNodeOP3::visualizePreviewFootsteps(bool clear)
{
  if (clear && preview_foot_steps_.size() == 0)
    return;

  visualization_msgs::msg::MarkerArray marker_array;
  auto now = rclcpp::Clock().now();
  visualization_msgs::msg::Marker rviz_marker;

  rviz_marker.header.frame_id = "body_link";
  rviz_marker.header.stamp = now;
  rviz_marker.ns = "foot_step_marker";

  rviz_marker.id = 1;
  rviz_marker.type = visualization_msgs::msg::Marker::CUBE;
  rviz_marker.action = (clear == false) ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;

  rviz_marker.scale.x = 0.128;
  rviz_marker.scale.y = 0.08;
  rviz_marker.scale.z = 0.01;

  double alpha = 0.7;
  double height = -0.229;

  geometry_msgs::msg::Pose local_pose, world_pose;
  bool result = transformPose("/world", "/body_link", world_pose, local_pose);
  if(result == true)
    height = local_pose.position.z;

  for (int ix = preview_foot_types_.size() - 1; ix >= 0; ix--)
  {
    // foot step marker array
    rviz_marker.id += 10;

    if (!clear)
    {
      Eigen::Vector3d marker_position(preview_foot_steps_[ix].x, preview_foot_steps_[ix].y, height);
      Eigen::Vector3d marker_position_offset;

      Eigen::Vector3d toward(1, 0, 0), direction(cos(preview_foot_steps_[ix].theta), sin(preview_foot_steps_[ix].theta),
                                                 0);
      Eigen::Quaterniond marker_orientation(Eigen::Quaterniond::FromTwoVectors(toward, direction));

      if (debug_)
      {
        std::stringstream msg;
        msg << "Foot Step #" << ix << " [ " << preview_foot_types_[ix] << "] - [" << rviz_marker.pose.position.x << ", "
            << rviz_marker.pose.position.y << "]";
        log(Info, msg.str());
      }
      alpha *= 0.9;

      // set foot step color
      if (preview_foot_types_[ix] == op3_online_walking_module_msgs::msg::Step2D::LEFT_FOOT_SWING)  // left
      {
        rviz_marker.color.r = 0.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 1.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, 0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }
      else if (preview_foot_types_[ix] == op3_online_walking_module_msgs::msg::Step2D::RIGHT_FOOT_SWING)  //right
      {
        rviz_marker.color.r = 1.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 0.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, -0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }

      marker_position = marker_position_offset + marker_position;

      rviz_marker.pose.position.x = marker_position.x();
      rviz_marker.pose.position.y = marker_position.y();
      rviz_marker.pose.position.z = marker_position.z();

      rviz_marker.pose.orientation.x = marker_orientation.x();
      rviz_marker.pose.orientation.y = marker_orientation.y();
      rviz_marker.pose.orientation.z = marker_orientation.z();
      rviz_marker.pose.orientation.w = marker_orientation.w();

      // tf2::toMsg(marker_position, rviz_marker.pose.position);
      // tf2::toMsg(marker_orientation, rviz_marker.pose.orientation);

      // apply foot x offset
    }

    marker_array.markers.push_back(rviz_marker);
  }

  // publish foot step marker array
  if (clear == false)
    log(Info, "Visualize Preview Footstep Marker Array");
  else
    log(Info, "Clear Visualize Preview Footstep Marker Array");

  marker_pub_->publish(marker_array);
}

// Preview walking
void QNodeOP3::sendFootStepCommandMsg(op3_online_walking_module_msgs::msg::FootStepCommand msg)
{
  foot_step_command_pub_->publish(msg);
  log( Info , "Send Foot Step Command Msg" );
}

void QNodeOP3::sendWalkingParamMsg(op3_online_walking_module_msgs::msg::WalkingParam msg)
{
  walking_param_pub_->publish(msg);
  log( Info, "Set Walking Parameter");
}

void QNodeOP3::sendBodyOffsetMsg(geometry_msgs::msg::Pose msg)
{
  body_offset_pub_->publish(msg);
  log( Info, "Send Body Offset");
}

void QNodeOP3::sendFootDistanceMsg(std_msgs::msg::Float64 msg)
{
  foot_distance_pub_->publish(msg);
  log( Info, "Send Foot Distance");
}

void QNodeOP3::sendResetBodyMsg( std_msgs::msg::Bool msg )
{
  reset_body_msg_pub_->publish( msg );
  log( Info , "Reset Body Pose" );
}

void QNodeOP3::sendWholebodyBalanceMsg(std_msgs::msg::String msg)
{
  wholebody_balance_pub_->publish( msg );
  log( Info , "Wholebody Balance Msg" );
}

void QNodeOP3::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Fail to load yaml file. [%s]", path.c_str());
    return;
  }

  op3_online_walking_module_msgs::msg::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  sendJointPoseMsg( msg );
}

void QNodeOP3::sendJointPoseMsg(op3_online_walking_module_msgs::msg::JointPose msg)
{
  joint_pose_msg_pub_->publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

}  // namespace robotis_op