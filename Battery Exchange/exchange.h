/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_TELEOP_H
#define OPEN_MANIPULATOR_TELEOP_H

#include <ros/ros.h>
#include <math.h> 
#include <sensor_msgs/JointState.h>

#include <termios.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Eigen>
#include "robotis_manipulator/robotis_manipulator.h"
#include "fiducial_msgs/FiducialTransform.h"
#include <fiducial_msgs/FiducialTransformArray.h>
#include <string>
#include <sstream>

#define PI 3.141592
#define DEGREE_DELTA 10
#define NUM_OF_JOINT 6
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

// states
#define STANDBY 0
#define ROTATE_PAD 1
#define SEEK_POS 2
#define SEEK_EDGE 3
#define ALIGN_EDGE 4
#define REFRESH 5
#define FIND_DRONE 6
#define SEEK_DRONE 7
#define GET_DRONE 8

class OpenManipulatorTeleop
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;
  
  // ROS Parameters
  bool with_gripper_;

  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_task_space_path_client_;
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;

  ros::Publisher joint_pub_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber aruco_detect_;

  std::vector<double> present_joint_angle_;
  std::vector<double> past_kinematic_position_;
  std::vector<double> present_kinematic_position_;
  std::vector<double> present_kinematic_position_rpy_;
  std::vector<fiducial_msgs::FiducialTransform> present_aruco_transform_;

  std::vector<double> drone_coords_;
  std::vector<double> edge_coords_;
  double heading_;
  double id2find_;
  double cam_height_;
  std::vector<double> target_;

  int state_;

  struct termios oldt_;

 public:

  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();


  void initClient();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void arucoDetectCallback(const fiducial_msgs::FiducialTransformArrayConstPtr& msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
  std::vector<double> getPastKinematicsPose();
  std::vector<double> getPresentKinematicsPoseRPY();
  std::vector<fiducial_msgs::FiducialTransform> getPresentArucoTransform();

  std::vector<double> getDroneCoords();
  std::vector<double> getEdgeCoords();
  double getHeading();
  double getIdToFind();

  void setPastKinematicsPose(void);

  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  
  bool setToolControl(std::vector<double> joint_angle);

  void printText();
  void setGoal(char ch);

  void restoreTerminalSettings(void);
  void disableWaitingForEnter(void);

  void updateOrientation(void);

};

#endif //OPEN_MANIPULATOR_TELEOP_H
