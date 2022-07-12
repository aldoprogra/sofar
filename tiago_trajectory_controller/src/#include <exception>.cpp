#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tiago_trajectory_controller/Control_msg.h>
#include <ros/topic.h>

float arr[7];
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

arm_control_client_Ptr ArmClient;
arm_control_client_Ptr actionClient;
control_msgs::FollowJointTrajectoryGoal arm_goal;
control_msgs::FollowJointTrajectoryGoal goal;

void chatterCallback(const tiago_trajectory_controller::Control_msg::ConstPtr& msg)
{
  //std::cout<< msg->a <<"\n" ;
  arr[0]= msg->a;
  arr[1]= msg->b;
  arr[2]= msg->c;
  arr[3]= msg->d;
  arr[4]= msg->e;
  arr[5]= msg->f;
  arr[6]= msg->g;
  std::cout<< arr[3] <<"\n" ;

  ROS_INFO("Creating action client to arm controller ...");
  std::cout<< arr[3] <<"\n" ;


  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");


   arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }
   goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = -3.14;
  goal.trajectory.points[index].positions[3] = 1.57;
  goal.trajectory.points[index].positions[4] = -0.0;
  goal.trajectory.points[index].positions[5] = -0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = -3.14;
  goal.trajectory.points[index].positions[3] = 1.57;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  //std::cout<< arr[3] <<"\n" ;

  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);

}


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience


// Create a ROS action client to move TIAGo's arm
/*void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");
  std::cout<< arr[3] <<"\n" ;


  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}*/


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
/*void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = -3.14;
  goal.trajectory.points[index].positions[3] = 1.57;
  goal.trajectory.points[index].positions[4] = -0.0;
  goal.trajectory.points[index].positions[5] = -0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = arr[0];
  goal.trajectory.points[index].positions[1] = arr[1];
  goal.trajectory.points[index].positions[2] = arr[2];
  goal.trajectory.points[index].positions[3] = arr[3];
  goal.trajectory.points[index].positions[4] = arr[4];
  goal.trajectory.points[index].positions[5] = arr[5];
  goal.trajectory.points[index].positions[6] = arr[6];
  //std::cout<< arr[3] <<"\n" ;

  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
}

*/


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");

  
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  ros::Subscriber sub = nh.subscribe("webcam_coordinates", 10, chatterCallback);

  // Create an arm controller action client to move the TIAGo's arm
  //arm_control_client_Ptr ArmClient;
  //createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  //control_msgs::FollowJointTrajectoryGoal arm_goal;
  //waypoints_arm_goal(arm_goal);

  // Sends the command to start the given trajectory 1s from now
  //arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  //ArmClient->sendGoal(arm_goal);

  // Wait for trajectory execution
  //while(!(ArmClient->getState().isDone()) && ros::ok())
  //{
    //ros::Duration(4).sleep(); // sleep for four seconds
  //}
  
  ros::spin();
  
  return EXIT_SUCCESS;
}

