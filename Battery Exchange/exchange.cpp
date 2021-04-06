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

#include "open_manipulator_p_teleop/exchange.h"

OpenManipulatorTeleop::OpenManipulatorTeleop()
    :node_handle_(""),
     priv_node_handle_("~"),
     with_gripper_(false)
{
  with_gripper_ = priv_node_handle_.param<bool>("with_gripper", false);
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);
  present_kinematic_position_rpy_.resize(3);
  state_ = STANDBY;

  
  joint_pub_ = ros::Publisher(node_handle_.advertise<std_msgs::Float64>("/landing_pad/motor_joint_position_controller/command", 1));
  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization\n");
  
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

  
}

void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  aruco_detect_ = node_handle_.subscribe("/fiducial_transforms", 10, &OpenManipulatorTeleop::arucoDetectCallback, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  std::vector<double> temp_position_rpy;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  // return rpy
  Eigen::Quaterniond temp_orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Vector3d temp_rpy = robotis_manipulator::math::convertQuaternionToRPYVector(temp_orientation);

  temp_position_rpy.push_back(temp_rpy.coeffRef(0,0)); //r
  temp_position_rpy.push_back(temp_rpy.coeffRef(1,0)); //p
  temp_position_rpy.push_back(temp_rpy.coeffRef(2,0)); //y


  present_kinematic_position_ = temp_position;
  present_kinematic_position_rpy_ = temp_position_rpy;
}

void OpenManipulatorTeleop::arucoDetectCallback(const fiducial_msgs::FiducialTransformArrayConstPtr& msg)
{
  if (msg->transforms.empty())
  {
    return;
  }



  //const fiducial_msgs::FiducialTransform *ft = msg->transforms.data();
  //std::vector<double> temp_transform;

  //double id = ft->fiducial_id;
  //double x = ft->transform.translation.x * 100; // to cm
  //double y = ft->transform.translation.y * 100;
  //double z = ft->transform.translation.z * 100;
  // call corrections handler
  //correctionHandler(x, y, z);
  //temp_transform.push_back(id);
  //temp_transform.push_back(x);
  //temp_transform.push_back(y);
  //temp_transform.push_back(z);
  //std::cout << "ID: ";
  //std::cout << msg->transforms.at(0).fiducial_id;
  //std::cout << "\n";
  //std::cout << "Size: ";
  //std::cout << msg->transforms.size();
  //std::cout << "\n";
  
  present_aruco_transform_ = msg->transforms;
  //printf("Aruco x: %f y: %f z: %f\n", getPresentArucoTransform().at(0), getPresentArucoTransform().at(1), getPresentArucoTransform().at(2));
  

}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}
std::vector<double> OpenManipulatorTeleop::getPastKinematicsPose()
{
  return past_kinematic_position_;
}
std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPoseRPY()
{
  return present_kinematic_position_rpy_;
}
std::vector<fiducial_msgs::FiducialTransform> OpenManipulatorTeleop::getPresentArucoTransform()
{
  return present_aruco_transform_;
}

std::vector<double> OpenManipulatorTeleop::getDroneCoords()
{
  return drone_coords_;
}
std::vector<double> OpenManipulatorTeleop::getEdgeCoords()
{
  return edge_coords_;
}
double OpenManipulatorTeleop::getHeading(){
  return heading_;
}
double OpenManipulatorTeleop::getIdToFind()
{
  return id2find_;
}


bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.end_effector_name = "gripper";
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);
  srv.request.path_time = path_time;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{

  printf("\n");
  printf("q to quit\n");
  printf("-------------------------------------------------------------------------------\n");
  printf("Present Joint Angle J1: %f J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3),
         getPresentJointAngle().at(4),
         getPresentJointAngle().at(5));
  printf("Present Kinematics Position X: %f Y: %f Z: %f ROLL: %.3f PITCH: %.3f YAW: %.3f \n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2),
         getPresentKinematicsPoseRPY().at(0),
         getPresentKinematicsPoseRPY().at(1),
         getPresentKinematicsPoseRPY().at(2));
  printf("-------------------------------------------------------------------------------\n");
  setPastKinematicsPose();
  printf("x rot: %f y rot: %f z rot: %f\n", getPresentKinematicsPoseRPY().at(0), getPresentKinematicsPoseRPY().at(1), getPresentKinematicsPoseRPY().at(2));
  

}

void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(7, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(6, 0.0);
    // standby
  if(state_ == STANDBY){
    // init pose
    printf("state: STANDBY\n");
    // set next state
    state_ = ROTATE_PAD;

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }

  else if(state_ == ROTATE_PAD){
    std::string inf;
    priv_node_handle_.getParam("input_data", inf);
    std::stringstream ss(inf);
    std::vector<double> temp_drone;
    std::vector<double> temp_edge;
    double x, y, temp_id2find, temp_compass;

    state_ = SEEK_POS;

    ss >> x;
    ss >> y;
    temp_drone.push_back(x);
    temp_drone.push_back(y);
    ss >> temp_id2find;
    ss >> x;
    ss >> y;
    temp_edge.push_back(x);
    temp_edge.push_back(y);
    ss >> temp_compass;

    drone_coords_ = temp_drone;
    edge_coords_ = temp_edge;
    heading_ = temp_compass;
    id2find_ = temp_id2find;

    std_msgs::Float64 joint_angle;
    joint_angle.data = PI - atan2(drone_coords_.at(1), drone_coords_.at(0));
    joint_pub_.publish(joint_angle);

  }

  else if(state_ == SEEK_POS){
    printf("state: SEEK_POS\n");

    // set next state
    state_ = SEEK_EDGE;

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(1.5708);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }

  else if(state_ == SEEK_EDGE){
    printf("state: SEEK_EDGE\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    std::vector<fiducial_msgs::FiducialTransform> trans = getPresentArucoTransform();
    // find id
    //find(vec.begin(), vec.end(), elem) != vec.end()
    int32_t id = (int32_t) getIdToFind();
    for(int i = 0; i < trans.size(); i++)
    {
      if(trans.at(i).fiducial_id == id)
      {
        state_ = ALIGN_EDGE;
      }
    }
    if(state_ == SEEK_EDGE)
    {
      // move forward a bi
     goalPose.at(0) = DELTA;
     setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    }
  }

  else if(state_ == ALIGN_EDGE)
  {
    printf("state: ALIGN_EDGE\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    std::vector<fiducial_msgs::FiducialTransform> trans = getPresentArucoTransform();
    // find id
    //find(vec.begin(), vec.end(), elem) != vec.end()
    int32_t id = (int32_t) getIdToFind();
    for(int i = 0; i < trans.size(); i++)
    {
      if(trans.at(i).fiducial_id == id)
      {
        // get x,y,z
        double x = trans.at(i).transform.translation.x;
        double y = trans.at(i).transform.translation.y;
        cam_height_ = trans.at(i).transform.translation.z;

        double err = sqrt(x*x + y*y);
        if(err > 0.05/100) // correction
        {
          printf("X: %f Y: %f\n", y, x);
          printf("Error: %f\n", err);
          goalPose.at(0) = -y; // y
          goalPose.at(1) = -x; // x
          setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
        } 
        else
        {
          state_ = REFRESH;
          std::vector<std::string> joint_name;
          joint_name.push_back("joint1"); 
          joint_name.push_back("joint2");
          joint_name.push_back("joint3");
          joint_name.push_back("joint4"); 
          joint_name.push_back("joint5"); goalJoint.at(4) = -1.5708;
          joint_name.push_back("joint6");
          setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
        } 
        break;
      }
    } 
  }

  else if(state_ == REFRESH)
  {
    printf("state: REFRESH\n");
    state_ = FIND_DRONE;
  }

  else if(state_ == FIND_DRONE)
  {
    printf("state: FIND_DRONE\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    std::vector<fiducial_msgs::FiducialTransform> trans = getPresentArucoTransform();

    std::vector<double> coords_d = getDroneCoords();
    std::vector<double> coords_e = getEdgeCoords();
    double heading = getHeading();
    coords_e[0] = coords_e[0]*8.5/100;
    coords_e[1] = coords_e[1]*8.5/100;
    double tht_d = atan2(coords_d[1], coords_d[0]);
    double tht_e = atan2(coords_e[1], coords_e[0]);
    double x_e = sqrt(coords_e[0]*coords_e[0] + coords_e[1]*coords_e[1]);
    double x_a = getPresentKinematicsPose().at(0);
    double z_a = getPresentKinematicsPose().at(2);

    double delta_x = coords_d[0] - ((15+10)*sin(heading*(PI/180)));
    double delta_y = coords_d[1] - ((15+10)*cos(heading*(PI/180)));
    delta_x = delta_x*(0.01);
    delta_y = delta_y*(0.01);

    double Dx = x_e*cos(abs(tht_d-tht_e)) + x_a;
    double Dz = (z_a - cam_height_)+0.13;
    double tht_pa = PI - atan2(drone_coords_.at(1), drone_coords_.at(0));
    double Ax = delta_x*cos(tht_pa) - delta_y*sin(tht_pa) + Dx;
    double Ay = delta_x*sin(tht_pa) + delta_y*cos(tht_pa);
    double Az = Dz;

    double Mx = coords_d[0]*cos(tht_pa)*(0.01) - coords_d[1]*sin(tht_pa)*(0.01) + Dx;
    double My = coords_d[0]*sin(tht_pa)*(0.01) + coords_d[1]*cos(tht_pa)*(0.01);
    double ang = atan2(My-Ay, Mx-Ax);
    target_.resize(4, 0.0);
    target_.at(0) = Ax;
    target_.at(1) = Ay;
    target_.at(2) = Az;
    target_.at(3) = ang;

    printf("New X: %f Y: %f Z: %f\n", Ax, Ay, Az);
    printf("Marker X: %f Y: %f Z: %f\n", Mx, My, ang);
    printf("Angle calced: %f\n", tht_pa);

    state_ = SEEK_DRONE;
    goalPose.at(2) = DELTA*3;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }

  else if(state_ == SEEK_DRONE)
  {
    printf("state: SEEK_DRONE\n");
    double x = getPresentKinematicsPose().at(0);
    double y = getPresentKinematicsPose().at(1);
    double z = getPresentKinematicsPose().at(2);

    double x2 = target_.at(0);
    double y2 = target_.at(1);
    double z2 = target_.at(2);

    goalPose.at(0) = x2-x;
    goalPose.at(1) = y2-y;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    state_ = GET_DRONE;

  }

  else if(state_ == GET_DRONE){
    printf("state: GET_DRONE\n");

    double x2 = target_.at(0);
    double y2 = target_.at(1);
    double z2 = target_.at(2);

    std::vector<double> pose = getPresentKinematicsPoseRPY();
    
    // update pose?
    Eigen::Quaterniond temp_orientation = robotis_manipulator::math::convertRPYToQuaternion(0.0, 0.0, target_.at(3));
    goalPose.at(0) = x2;
    goalPose.at(1) = y2;
    goalPose.at(2) = z2;
    goalPose.at(3) = temp_orientation.w();
    goalPose.at(4) = temp_orientation.x();
    goalPose.at(5) = temp_orientation.y();
    goalPose.at(6) = temp_orientation.z();
    setTaskSpacePath(goalPose, PATH_TIME);


  }


}

void OpenManipulatorTeleop::updateOrientation(void)
{
    std::vector<double> goalPose;  goalPose.resize(7, 0.0);
    std::vector<double> pres_pose = getPresentKinematicsPose();
    std::vector<double> pres_joint = getPresentJointAngle();
    std::vector<double> init_rpy = getPresentKinematicsPoseRPY();
    if( pres_pose.at(0) != 0 || pres_pose.at(1) != 0)
    {
      // find angle
      double x1, x2, y1, y2, dist, theta;
      y1 = 0;
      x2 = pres_pose.at(0);
      y2 = pres_pose.at(1);
      x1 = sqrt( pow(x2, 2.0) + pow(y2, 2.0) );

      dist = sqrt( pow((x2-x1), 2.0) + pow((y2-y1), 2.0) );
      
      theta = asin((dist/2)/x1);
      if( y2 < 0) 
      {
        theta = theta * -1.0;
      }
      //theta = pres_joint.at(0);
      printf("x1 = %f y1 = %f x2 = %f y2 = %f dist = %f\n", x1, y1, x2, y2, dist);
      printf("Angle found: %f\n", theta);

      // update pose?
      Eigen::Quaterniond temp_orientation = robotis_manipulator::math::convertRPYToQuaternion(0.0, 0.0, theta);
      goalPose.at(0) = x2;
      goalPose.at(1) = y2;
      goalPose.at(2) = pres_pose.at(2);
      goalPose.at(3) = temp_orientation.w();
      goalPose.at(4) = temp_orientation.x();
      goalPose.at(5) = temp_orientation.y();
      goalPose.at(6) = temp_orientation.z();
      setTaskSpacePath(goalPose, PATH_TIME);
    }

}

void OpenManipulatorTeleop::setPastKinematicsPose(void)
{
  //printf("x: %f y: %f z: %f\n", getPresentKinematicsPose().at(0), getPresentKinematicsPose().at(1), getPresentKinematicsPose().at(2) );
  past_kinematic_position_ = getPresentKinematicsPose();
  
}
void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
    tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop");

  OpenManipulatorTeleop openManipulatorTeleop;

  ROS_INFO("OpenManipulator teleoperation using keyboard start");
  openManipulatorTeleop.disableWaitingForEnter();


  ros::spinOnce();
  openManipulatorTeleop.printText();

  char ch;
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch);

  }

  printf("input : q \tTeleop. is finished\n");
  openManipulatorTeleop.restoreTerminalSettings();

  return 0;
}
