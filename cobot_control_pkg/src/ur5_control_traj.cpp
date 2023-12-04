#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>


//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html
//http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html



//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: error_code is %d", result->error_code);
  //error_string is not showing the message...
  //see http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html for the meaning of the error codes.
  //ROS_INFO_STREAM("Answer: error_string is "<< result->error_string);
}

//Callback function: Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  //ROS_INFO("************: desired angles are (%f,%f)", feedback->desired.positions[0], feedback->desired.positions[1]);
  ROS_INFO("Got Feedback: current positions     are (%f,%f)", feedback->actual.positions[0], feedback->actual.positions[1]);
  ROS_INFO("              current velocities    are (%f,%f)", feedback->actual.velocities[0], feedback->actual.velocities[1]);
  //Acceleration feedback field is not filled, so do not print:
  //ROS_INFO("              accelerations.size() = %d", feedback->actual.accelerations.size());
  //ROS_INFO("              current accelerations are (%f,%f)", feedback->actual.accelerations[0], feedback->actual.accelerations[1]);
}


//Sends the goal to the FollowJointTrajectory action server and waits for the result for trajduration seconds
//If not able to reach the goal within timeout, it is cancelled
bool moveRobotTrajectory(control_msgs::FollowJointTrajectoryGoal goal, double trajduration, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &rClient) 
{
  //Set timestamp and send goal
  goal.trajectory.header.stamp = ros::Time::now();
  //rClient.sendGoal(goal);
  rClient.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  
  //Wait for the action to return. Timeout set to the trajduration plus the goal time tolerance)
  bool finished_before_timeout = rClient.waitForResult(ros::Duration(trajduration) + goal.goal_time_tolerance);
  
  //Get final state
  actionlib::SimpleClientGoalState state = rClient.getState();
  if (finished_before_timeout) {
      //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
      //Reports SUCCEEDED if finished and goal reached
      ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
  } else {
      //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
      ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
      //Preempting task
      ROS_ERROR("I am going to preempt the task...");
      rClient.cancelGoal();
  }
  return true;
}


int main(int argc, char **argv) 
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "ur5_control_traj");
  ros::NodeHandle nh;
  
  //Define simple action client and wail for server
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClient("/arm_controller/follow_joint_trajectory");
  if(!robotClient.waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR(" *** action server not available *** ");
  };
  
  //Defines a sinoidal trajectory for the tilt joint and fixed value for the pan joint
  double cycletime = 10.0;//10 seconds
  int trajpoints = 10;
  double moveduration = cycletime/trajpoints;
  
  control_msgs::FollowJointTrajectoryGoal goalTraj;
  
  //Header stamp is set just before sending the goal in the moveRobotTrajectory function
  //goal.trajectory.header.stamp = ros::Time::now();
  
  //Set goal trajectory
  goalTraj.trajectory.joint_names.resize(6);       
  goalTraj.trajectory.joint_names[0] = "shoulder_pan_joint";
  goalTraj.trajectory.joint_names[1] = "shoulder_lift_joint";
  goalTraj.trajectory.joint_names[2] = "elbow_joint";
  goalTraj.trajectory.joint_names[3] = "wrist_1_joint";
  goalTraj.trajectory.joint_names[4] = "wrist_2_joint";
  goalTraj.trajectory.joint_names[5] = "wrist_3_joint";
    
  goalTraj.trajectory.points.resize(trajpoints);
  goalTraj.trajectory.points[0].positions.resize(6);
  goalTraj.trajectory.points[0].positions[0] = 0.0;
  goalTraj.trajectory.points[0].positions[1] = 0.0;
  goalTraj.trajectory.points[0].positions[2] = 0.0;
  goalTraj.trajectory.points[0].positions[3] = 0.0;
  goalTraj.trajectory.points[0].positions[4] = 0.0;
  goalTraj.trajectory.points[0].positions[5] = 0.0;
  goalTraj.trajectory.points[0].time_from_start = ros::Duration(moveduration);
  for(int i=1; i<trajpoints;i++)
  {
    goalTraj.trajectory.points[i].positions.resize(6);
    goalTraj.trajectory.points[i].positions[0] = 0.3*sin(6.28*i/trajpoints);
    goalTraj.trajectory.points[i].positions[1] = 0.0;
    goalTraj.trajectory.points[i].positions[2] = 0.0;
    goalTraj.trajectory.points[i].positions[3] = 0.0;
    goalTraj.trajectory.points[i].positions[4] = 0.0;
    goalTraj.trajectory.points[i].positions[5] = 0.0;
    goalTraj.trajectory.points[i].time_from_start = goalTraj.trajectory.points[i-1].time_from_start + ros::Duration(moveduration);
  }  
  //set last point velocity and acceleration to zero
  goalTraj.trajectory.points[trajpoints-1].velocities.resize(6);
  goalTraj.trajectory.points[trajpoints-1].velocities[0] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].velocities[1] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].velocities[2] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].velocities[3] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].velocities[4] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].velocities[5] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations.resize(6);
  goalTraj.trajectory.points[trajpoints-1].accelerations[0] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations[1] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations[2] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations[3] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations[4] = 0.0;
  goalTraj.trajectory.points[trajpoints-1].accelerations[5] = 0.0;
  
  //Set goal tolerances
  goalTraj.goal_tolerance.resize(6);       
  goalTraj.goal_tolerance[0].name = "shoulder_pan_joint";
  goalTraj.goal_tolerance[0].position = 0.1;
  goalTraj.goal_tolerance[0].velocity = 0.1;
  //goalTraj.goal_tolerance[0].acceleration = 0.6;
  goalTraj.goal_tolerance[1].name = "shoulder_lift_joint";
  goalTraj.goal_tolerance[1].position = 0.1;
  goalTraj.goal_tolerance[1].velocity = 0.1;
  goalTraj.goal_tolerance[2].name = "elbow_joint";
  goalTraj.goal_tolerance[2].position = 0.1;
  goalTraj.goal_tolerance[2].velocity = 0.1;
  goalTraj.goal_tolerance[3].name = "wrist_1_joint";
  goalTraj.goal_tolerance[3].position = 0.1;
  goalTraj.goal_tolerance[3].velocity = 0.1;
  goalTraj.goal_tolerance[4].name = "wrist_2_joint";
  goalTraj.goal_tolerance[4].position = 0.1;
  goalTraj.goal_tolerance[4].velocity = 0.1;
  goalTraj.goal_tolerance[5].name = "wrist_3_joint";
  goalTraj.goal_tolerance[5].position = 0.1;
  goalTraj.goal_tolerance[5].velocity = 0.1;
  //goalTraj.goal_tolerance[1].acceleration = 0.6;
  goalTraj.goal_time_tolerance = ros::Duration(1.0);
  
  
  //Call function to send goal
  //Wait for user to press a key
  std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
  std::cin.get();
    
  //ros::Rate rate(1/cycletime);
  while(ros::ok()) {
    moveRobotTrajectory(goalTraj, cycletime, robotClient);
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO CONTINUE..."<<std::endl;
    std::cin.get();
    
    // Wait until it's time for another iteration.
    //rate.sleep();
  }
  
}
