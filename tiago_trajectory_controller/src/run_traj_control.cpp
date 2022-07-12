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
#include <iostream>


std::string request;

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

arm_control_client_Ptr ArmClient;
control_msgs::FollowJointTrajectoryGoal arm_goal;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

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
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{

  std::cout<<"you are inside waypoints_arm_goal function \n";
  fflush(stdout);
  if (request == "raise") {
  std::cout<<"you are in the case of raising hand \n";
  fflush(stdout);

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
  goal.trajectory.points[index].time_from_start = ros::Duration(0.0);

  // Second trajectory point
  // Positions
  index += 1;
  std::cout<<"index increased \n";
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = -3.14;
  goal.trajectory.points[index].positions[3] = 1.57;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
   }
  else if(request == "wave"){
  std::cout<<"you are in the case of waiving \n";
  fflush(stdout);
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");
  goal.trajectory.points.resize(4);

  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.063;
  goal.trajectory.points[index].positions[1] = -0.67;
  goal.trajectory.points[index].positions[2] = -3.10;
  goal.trajectory.points[index].positions[3] = 2.08;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = -0.031;
  goal.trajectory.points[index].positions[6] = -2.07;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
    }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
                          
  index = index+1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.063;
  goal.trajectory.points[index].positions[1] = -0.73;
  goal.trajectory.points[index].positions[2] = -2.93;
  goal.trajectory.points[index].positions[3] = 1.84;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = -0.031;
  goal.trajectory.points[index].positions[6] = -2.07;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
    {
      goal.trajectory.points[index].velocities[j] = 1.0;
    }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(6.0);

  index = index+1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.063;
  goal.trajectory.points[index].positions[1] = -0.72;
  goal.trajectory.points[index].positions[2] = -2.93;
  goal.trajectory.points[index].positions[3] = 2.12;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = -0.031;
  goal.trajectory.points[index].positions[6] = -2.07;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
      {
        goal.trajectory.points[index].velocities[j] = 1.0;
    }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(8.0);
  index = index+1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.063;
  goal.trajectory.points[index].positions[1] = -0.73;
  goal.trajectory.points[index].positions[2] = -2.93;
  goal.trajectory.points[index].positions[3] = 1.83;
  goal.trajectory.points[index].positions[4] = -1.12;
  goal.trajectory.points[index].positions[5] = -0.031;
  goal.trajectory.points[index].positions[6] = -2.07;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
   for (int j = 0; j < 7; ++j)
     {
     goal.trajectory.points[index].velocities[j] = 1.0;
      }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(10.0);
  }
 }




void webcamCallback(const tiago_trajectory_controller::Control_msg::ConstPtr& msg){
                        
  std::cout<<"You are inside the webcamcallback \n";
  ROS_INFO("I heard: [%s]", msg->gesture.c_str());

  request = msg->gesture.c_str();
  createArmClient(ArmClient);
  fflush(stdout);
  waypoints_arm_goal(arm_goal);
  fflush(stdout);
                        
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);
  //while(!(ArmClient->getState().isDone()) && ros::ok())
    //{
      //ros::Duration(4).sleep(); // sleep for four seconds
    
  }

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");
  
  // Precondition: Valid clock
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/webcam_coordinates", 1000, webcamCallback);
  
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
  // Create an arm controller action client to move the TIAGo's arm
  ///arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

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

