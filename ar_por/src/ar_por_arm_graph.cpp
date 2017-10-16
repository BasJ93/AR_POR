/***************************************/
/* Authors: Bas Janssen, Dimitri Waard */
/* Fontys Hogeschool Engineering       */
/* 2017                                */
/*                                     */
/* Modified from Descartes tutorial    */
/***************************************/

// Core ros functionality like ros::init and spin
#include <ros/ros.h>
#include <ros/package.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

#include <descartes_planner/planning_graph.h>

//The path is build using points in the eigen convention
#include <eigen_conversions/eigen_msg.h>

//So we can visualize the path
#include <visualization_msgs/Marker.h>

//The headers to access files.
#include <iostream>
#include <fstream>

#include <math.h>

//Some tricks from boost
#include <boost/algorithm/string.hpp>

//Our character definitions
#include "Alfa.h"

//Definitions for the path visualization
#define   AXIS_LINE_WIDTH 0.001
#define   AXIS_LINE_LENGHT 0.01
#define   WORLD_FRAME "base_link"

// Offsets
float safeOffset = 0.1;
float xOffset = 0.355;
float yOffset = 0.0;
float step = 0.005;

typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;

ros::Publisher marker_publisher_;

//Interpolate a path between the "Home" pose and the first point in the path.
EigenSTL::vector_Affine3d addLeadin(EigenSTL::vector_Affine3d points)
{
  Eigen::Affine3d poseStart = points[0];
  
  double xStart = poseStart.translation().x();
  double yStart = poseStart.translation().y();
  double zStart = poseStart.translation().z();

  double xStep = (xStart - 0.30)/10;
  double yStep = (yStart)/10;
  double zStep = (0.40 - zStart)/10;

  Eigen::Affine3d pose;
  for(int i=0; i<11; i++)
  {
    pose = Eigen::Translation3d(0.30 + xStep * (10 - i), yStep * (10 - i), 0.40 - zStep * (10 - i));
    points.insert(points.begin(), pose);
  }
  return points;
}

//Interpolate a path between the last point in the path and the "Home" pose.
EigenSTL::vector_Affine3d addLeadout(EigenSTL::vector_Affine3d points)
{
  Eigen::Affine3d poseStart = points[points.size() - 1];
  
  double xStart = poseStart.translation().x();
  double yStart = poseStart.translation().y();
  double zStart = poseStart.translation().z();

  double xStep = (xStart - 0.30)/10;
  double yStep = (0.0 - yStart)/10;
  double zStep = (0.40 - zStart)/10;

  Eigen::Affine3d pose;
  for(int i=0; i<11; i++)
  {
    pose = Eigen::Translation3d(xStart - xStep * i, yStart + yStep * i, zStart + zStep * i);

    points.push_back(pose);
  }
  return points;
}

//Publish the path visualization to rviz
void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;

  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = WORLD_FRAME;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = WORLD_FRAME;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Affine3d prev = poses[0];
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > 0.01)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  marker_publisher_.publish(markers_msg);

}

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_trajectory_curve",1,true);

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  // Get Joint Names
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

  while(1)
  {
    TrajectoryVec path;
    EigenSTL::vector_Affine3d points;
    
    std::string text;
    std::cout << "Enter text to draw: ";
    std::getline(std::cin, text);

    std::cout << "Drawing " << text << "\n";

    float letterWidth = (0.200 - 0.003 * text.size()) / text.size();
    float ySpacing = letterWidth + 0.003;
    step = letterWidth / 10;
    yOffset = ySpacing * ((text.size())/2);

    for(char& c : text)
    {
      if(c == 'a' || c == 'A')
      {
        points = createA(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'b' || c == 'B')
      {
        points = createB(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'c' || c == 'C')
      {
        points = createC(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'd' || c == 'D')
      {
        points = createD(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'e' || c == 'E')
      {
        points = createE(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'f' || c == 'F')
      {
        points = createF(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'g' || c == 'G')
      {
        points = createG(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'h' || c == 'H')
      {
        points = createH(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'i' || c == 'I')
      {
        points = createI(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'j' || c == 'J')
      {
        points = createJ(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'k' || c == 'K')
      {
        points = createK(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'l' || c == 'L')
      {
        points = createL(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'm' || c == 'M')
      {
        points = createM(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'n' || c == 'N')
      {
        points = createN(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'o' || c == 'O')
      {
        points = createO(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'p' || c == 'P')
      {
        points = createP(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'q' || c == 'Q')
      {
        points = createQ(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'r' || c == 'R')
      {
        points = createR(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 's' || c == 'S')
      {
        points = createS(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 't' || c == 'T')
      {
        points = createT(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'u' || c == 'U')
      {
        points = createU(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'v' || c == 'V')
      {
        points = createV(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'w' || c == 'W')
      {
        points = createW(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'x' || c == 'X')
      {
        points = createX(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'y' || c == 'Y')
      {
        points = createY(points, safeOffset, xOffset, yOffset, step);
      }
      else if(c == 'z' || c == 'Z')
      {
        points = createZ(points, safeOffset, xOffset, yOffset, step);
      }

      yOffset -= ySpacing;

    }

    points = addLeadin(points);

    points = addLeadout(points);

    if(!points.empty())
    {
      publishPosesMarkers(points);

      std::string packagePath = ros::package::getPath("ar_por");
      std::ifstream pathfile (packagePath + "/paths/" + text + ".path");
      
      if (pathfile.is_open()) 
      {
        std::string line;
        trajectory_msgs::JointTrajectory result;
        result.header.stamp = ros::Time::now();
        result.header.frame_id = "base_link";
        result.joint_names = names;
        while ( getline (pathfile,line) )
        {
          std::vector<std::string> jointAngles;
          boost::split(jointAngles, line, boost::is_any_of("\t"));
          std::vector<double> joints;
          for(int i=0; i<6; i++)
          {
            joints.push_back(std::stod(jointAngles[i+1]));
          }
          trajectory_msgs::JointTrajectoryPoint pt;
          pt.positions = joints;
          // velocity, acceleration, and effort are given dummy values
          // we'll let the controller figure them out
          pt.velocities.resize(joints.size(), 0.0);
          pt.accelerations.resize(joints.size(), 0.0);
          pt.effort.resize(joints.size(), 0.0);
          // set the time into the trajectory
          pt.time_from_start = ros::Duration(std::stof(jointAngles[0]));
          // increment time

          result.points.push_back(pt); 
        }
        pathfile.close();
        if (!executeTrajectory(result))
        {
          ROS_ERROR("Could not execute trajectory!");
          return -4;
        }
      }
      else
      {
        std::string compute;
        std::cout << "Path has to be computed (this can take multipe minutes) continue?[y/n]: ";
        std::getline(std::cin, compute);

        if(compute.compare("y") == 0)
        {

          for(unsigned int i = 0; i < points.size(); i++)
          {
            const Eigen::Affine3d& pose = points[i];
            descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
            path.push_back(pt);
          }

          // 2. Create a robot model and initialize it
          descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

          // Name of description on parameter server. Typically just "robot_description".
          const std::string robot_description = "robot_description";

          // name of the kinematic group you defined when running MoveitSetupAssistant
          const std::string group_name = "manipulator";

          // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
          const std::string world_frame = "/base_link";

          // tool center point frame (name of link associated with tool)
          const std::string tcp_frame = "pencil_tip";

          if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
          {
            ROS_INFO("Could not initialize robot model");
            return -1;
          }

          // 3. Create a planner and initialize it with our robot model
//          descartes_planner::DensePlanner plannerDense;
//          plannerDense.initialize(model);

          TrajectoryVec result;
          // Feed the trajectory to the planner
/*          if (!plannerDense.planPath(path))
          {
            ROS_ERROR("Could not solve for a valid path");
            return -2;
          }
          if (!plannerDense.getPath(result))
          {
            ROS_ERROR("Could not retrieve path");
            return -3;
          }
*/

          descartes_planner::PlanningGraph _planningGraph(model);
          
          bool success = _planningGraph.insertGraph(&path);
          ROS_INFO("Graph insert: %s", success);
          
          // 5. Translate the result into a type that ROS understands
          // Generate a ROS joint trajectory with the result path, robot model, given joint names,
          // a certain time delta between each trajectory point
          trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 0.1);


          //Write the path to a path file, for later reference.
          std::string packagePath = ros::package::getPath("ar_por");
          std::ofstream pathfile (packagePath + "/paths/" + text + ".path");
          for(int i=0; i<joint_solution.points.size() - 1; i++)
          {
            outputFile << joint_solution.points[i].time_from_start << "\t";
            for(int j=0; j<6; j++)
            {
              outputFile << joint_solution.points[i].positions[j] << "\t";
            }
            outputFile << "\n";
          }
          outputFile.close();

          // 6. Send the ROS trajectory to the robot for execution
          if (!executeTrajectory(joint_solution))
          {
            ROS_ERROR("Could not execute trajectory!");
            return -4;
          }
        }
      }
    }
  }
  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
//  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
//The second argument is the angle the tool is allow to be rotated, e.g. 15 degrees.
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/30, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "base_link";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(5.0);
  
  ac.sendGoal(goal);

  if (ac.waitForResult()) //goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(30.0)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}
