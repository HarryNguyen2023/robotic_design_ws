#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"

#include "cobot_control_pkg/set_point_to_point_trajectory.h"
#include "cobot_control_pkg/set_goal_tolerances.h"
#include "cobot_control_pkg/move_robot_trajectory.h"
#include "cobot_control_pkg/run_finished.h"
//Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
//#include "cobot_control_pkg/move_robot_trajectory_safe.h"

int pp_sequence = 0;
double degree2rad = M_PI/180;
// Define the joint trajectory position of each pose 
const int joint_num = 6;
bool run = true;
bool sequence = true;
std::vector<std::string> joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
double sequence_pose[5][joint_num] = {{0.0, -90, 90, 0.0, 90, 0.0},
                                {-90, -90, 90, 0.0, 90, 0.0},
                                {-90, -73, 90, -17, 90, 0.0},
                                {0.0, -71, 112, 49, 90, 0.0},
                                {0.0, -90, 90, 0.0, 90, 0.0}};

// Function to execute run finished call back
bool runFinishedCallBack(cobot_control_pkg::run_finished::Request& req, cobot_control_pkg::run_finished::Response& res)
{
    run = req.run_finished;
    if(run)
        ROS_INFO("Finshied sequence %d", pp_sequence);
    else
        ROS_INFO("Unable to continue the sequence!");
    pp_sequence++;
    if(pp_sequence > 5)
    {
        pp_sequence = 0;
    }
    return true;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "ur5_follow_traj_client");
  ros::NodeHandle nh;

  // Advertise service
  ros::ServiceServer run_finished_server = nh.advertiseService("run_finished", runFinishedCallBack);

  //Create client objects for the services.
  ros::ServiceClient client_settraj = nh.serviceClient<cobot_control_pkg::set_point_to_point_trajectory>("set_point_to_point_trajectory");
  ros::ServiceClient client_setgoaltol = nh.serviceClient<cobot_control_pkg::set_goal_tolerances>("set_goal_tolerances");
  ros::ServiceClient client_moverobottraj = nh.serviceClient<cobot_control_pkg::move_robot_trajectory>("move_robot_trajectory");
  
  //Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
  // ros::ServiceClient client_moverobottrajsafe = nh.serviceClient"cobot_control_pkg::move_robot_trajectory_safe"("move_robot_trajectory_safe");

  ///////////////////////////////////////////
  //Create the request and response objects for setting the traj
  cobot_control_pkg::set_point_to_point_trajectory::Request reqsetraj;
  cobot_control_pkg::set_point_to_point_trajectory::Response respsettraj;
    ///////////////////////////////////////////
  //Create the request and response objects for setting the goal tolerances
  cobot_control_pkg::set_goal_tolerances::Request reqgoaltol;
  cobot_control_pkg::set_goal_tolerances::Response respgoaltol;

  //Set the trajectory to follow
  //Defines a sinoidal trajectory for the tilt joint and fixed value for the pan joint
  double moveduration = 10.0;//3 seconds

  //Set point to point trajectory
  reqsetraj.joint_names.resize(joint_num);
  reqsetraj.goalPoint.positions.resize(joint_num);
  reqgoaltol.jointGoalTolerances.resize(joint_num);

  reqsetraj.joint_names[0] = "shoulder_pan_joint";
  reqsetraj.joint_names[1] = "shoulder_lift_joint";
  reqsetraj.joint_names[2] = "elbow_joint";
  reqsetraj.joint_names[3] = "wrist_1_joint";
  reqsetraj.joint_names[4] = "wrist_2_joint";
  reqsetraj.joint_names[5] = "wrist_3_joint";

  reqgoaltol.jointGoalTolerances[0].name = "shoulder_pan_joint";
  reqgoaltol.jointGoalTolerances[1].name = "shoulder_lift_joint";
  reqgoaltol.jointGoalTolerances[2].name = "elbow_joint";
  reqgoaltol.jointGoalTolerances[3].name = "wrist_1_joint";
  reqgoaltol.jointGoalTolerances[4].name = "wrist_2_joint";
  reqgoaltol.jointGoalTolerances[5].name = "wrist_3_joint";

  for(int i = 0; i < joint_num; ++i)
  {
    reqsetraj.goalPoint.positions[i] = 0.0;
    reqgoaltol.jointGoalTolerances[i].position = 0.1;
    reqgoaltol.jointGoalTolerances[1].velocity = 0.1;
  }
  reqsetraj.goalPoint.time_from_start = ros::Duration(5.0);

  //call the set_trajectory service
  ros::service::waitForService("set_point_to_point_trajectory", ros::Duration(5));

  //jointGoalTolerances[1].acceleration = 0.6;
  reqgoaltol.goalTimeTolerance = ros::Duration(1.0);
  //call the set_trajectory service
  ros::service::waitForService("set_goal_tolerances", ros::Duration(5));
  client_setgoaltol.call(reqgoaltol,respgoaltol);

  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot
  cobot_control_pkg::move_robot_trajectory::Request reqmovetraj;
  cobot_control_pkg::move_robot_trajectory::Response respmovetraj;
  reqmovetraj.trajduration = moveduration;
  ros::service::waitForService("move_robot_trajectory", ros::Duration(5));


  //Uncomment next lines if you have implemented the motion along a safe trajectory (i.e. within limits)
  /*
  ///////////////////////////////////////////
  //Create the request and response objects for moving the robot with limits
  cobot_control_pkg::move_robot_trajectory_safe::Request reqmovetrajsafe;
  cobot_control_pkg::move_robot_trajectory_safe::Response respmovetrajsafe;
  reqmovetrajsafe.trajduration = moveduration;
  reqmovetrajsafe.limits.resize(6);
  reqmovetrajsafe.limits[0] = -100000.0; //xmin
  reqmovetrajsafe.limits[1] =  100000.0; //xmax
  reqmovetrajsafe.limits[2] = -100000.0; //ymin
  reqmovetrajsafe.limits[3] =  100000.0; //ymax
  reqmovetrajsafe.limits[4] = -100000.0; //zmin
  reqmovetrajsafe.limits[5] =  0.3; //zmax
  ros::service::waitForService("move_robot_trajectorysafe", ros::Duration(5));
 */
  std::cout<<"Press for start: "<<std::endl;
  std::cin.get();

  //Call function to send goal
  //Wait for user to press a key
  //ros::Rate rate(1/cycletime);
  while(ros::ok()) {
    ///////////////////////////////////////////
    //Send the goal to move the robot
    //move unconstrained
    if(run)
    {
        for(int i = 0; i < joint_num; ++i)
            reqsetraj.goalPoint.positions[i] = sequence_pose[pp_sequence][i] * degree2rad;
        std::cout<<"Free motion to configuration ("<<reqsetraj.goalPoint.positions[0]<<","
          <<reqsetraj.goalPoint.positions[1]<<")"<<std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottraj.call(reqmovetraj,respmovetraj);
        sequence = false;
        run = false;
    }
    //move contrained (preemptrs if out of cartesian limits)
    else
    {
        //Uncomment next line if you have implemented the motion along a safe trajectory (i.e. within limits)
        /*
        reqsetraj.goalPoint.positions[0] = -0.5 + 1.0*rand()/RAND_MAX;
        reqsetraj.goalPoint.positions[1] = -3.5 + 1.0*rand()/RAND_MAX;
        std::cout"""Contrained motion to configuration ("""reqsetraj.goalPoint.positions[0]""","
            ""reqsetraj.goalPoint.positions[1]""") - preempts if out of cartesian limits x["
            ""reqmovetrajsafe.limits[0]""","""reqmovetrajsafe.limits[1]"""],y["
            ""reqmovetrajsafe.limits[2]""","""reqmovetrajsafe.limits[3]"""],z["
            ""reqmovetrajsafe.limits[4]""","""reqmovetrajsafe.limits[5]"""]"""std::endl;
        client_settraj.call(reqsetraj,respsettraj);
        client_moverobottrajsafe.call(reqmovetrajsafe,respmovetrajsafe);
        */
    }
    //Wait for user to press a key
    // std::cout<<"\nPRESS (f) for a free motion or PRESS (c) for a constrained motion..."<<std::endl;
    // std::cin.get();
    ros::spinOnce();
    // std::cin>> c;

    // Wait until it's time for another iteration.
    //rate.sleep();
  }
  return 0;
}
