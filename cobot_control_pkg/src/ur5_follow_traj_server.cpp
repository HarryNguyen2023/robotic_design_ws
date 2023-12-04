#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"

#include "cobot_control_pkg/set_point_to_point_trajectory.h"
#include "cobot_control_pkg/set_goal_tolerances.h"
#include "cobot_control_pkg/move_robot_trajectory.h"
#include "cobot_control_pkg/run_finished.h"

// Global variables
control_msgs::FollowJointTrajectoryGoal goalTraj;
ros::ServiceClient run_finished_client;
bool run = false;
double trajduration;
int joint_num = 6;

// Function to execute call back when the action is done
void doneCB(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: error_code is %d", result->error_code);
}

// Function to execute call back when action is active
void activeCB()
{
    ROS_INFO("Robot starts moving");
}

// Function to execute call back during feedback
void feedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
    // Display current position and velocity of 2 joints
    ROS_INFO("Got Feedback: current positions     are (%f,%f)", feedback->actual.positions[0], feedback->actual.positions[1]);
    ROS_INFO("              current velocities    are (%f,%f)", feedback->actual.velocities[0], feedback->actual.velocities[1]);
}

// Function to move the robot according to the specified trajectory
bool moveRobotTrajectory(control_msgs::FollowJointTrajectoryGoal goal, double trajduration, 
                        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &rClient) 
{
    cobot_control_pkg::run_finished::Request req_run;
    cobot_control_pkg::run_finished::Response res_run;

    goal.trajectory.header.stamp = ros::Time::now();
    // Send goal to the action server
    rClient.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);
    // Check if the action is done before time out
    bool finished_before_timeout = rClient.waitForResult(ros::Duration(trajduration) + goal.goal_time_tolerance);
    // Get the final state at time out of the robot
    actionlib::SimpleClientGoalState goal_state = rClient.getState();
    if(finished_before_timeout)
    {
        ROS_INFO("Robot action has finished: %s", goal_state.toString().c_str());      
    }
    else
    {
        ROS_ERROR("Robot actions has stopped at state: %s", goal_state.toString().c_str());
        ROS_ERROR("Preempting task");
        rClient.cancelGoal();
    }
    req_run.run_finished = finished_before_timeout;
    run_finished_client.call(req_run, res_run);
    return finished_before_timeout;
}

// Function to handle move tolerance service callback
bool setGoalToleranceCallBack(cobot_control_pkg::set_goal_tolerances::Request& req, cobot_control_pkg::set_goal_tolerances::Response& res)
{
    // Define the goal tolerances
    for(int i = 0; i < joint_num; ++i)
    {
        goalTraj.goal_tolerance[i].name = req.jointGoalTolerances[i].name;
        goalTraj.goal_tolerance[i].position = req.jointGoalTolerances[i].position;
        goalTraj.goal_tolerance[i].velocity = req.jointGoalTolerances[i].velocity;
    }
    goalTraj.goal_time_tolerance = ros::Duration(req.goalTimeTolerance);
    ROS_INFO("Finished set trajectory tolerances");
    return true;
}

// Function to execute the set trajectory callback
bool setTrajectoryCallBack(cobot_control_pkg::set_point_to_point_trajectory::Request& req, cobot_control_pkg::set_point_to_point_trajectory::Response& res)
{
    for(int i = 0; i < joint_num; ++i)
    {
        goalTraj.trajectory.joint_names[i] = req.joint_names[i];
        goalTraj.trajectory.points[0].positions[i] = req.goalPoint.positions[i];
        goalTraj.trajectory.points[0].velocities[i] = 0.0;
        goalTraj.trajectory.points[0].accelerations[i] = 0.0;
    }
    goalTraj.trajectory.points[0].time_from_start = req.goalPoint.time_from_start;
    ROS_INFO("Finished set trajectory parameters");
    return true;
}

// Function to execute the move robot call back
bool moveTrajectoryCallback(cobot_control_pkg::move_robot_trajectory::Request& req, cobot_control_pkg::move_robot_trajectory::Response& res)
{
    trajduration = req.trajduration;
    run = true;
    ROS_INFO_STREAM("Robot is moving to pan: "<<goalTraj.trajectory.points[0].positions[0]<<" tilt: "<<goalTraj.trajectory.points[0].positions[1]);
    return true;
}

// Function to execute move safe call back
// bool moveSafeCallback(cobot_control_pkg::move_robot_trajectory_safe::Request& req, cobot_control_pkg::move_robot_trajectory_safe::Response& res)
// {
//     trajduration = req.trajduration;
//     run = true;
//     ROS_INFO_STREAM("Robot is moving to pan: "<<goalTraj.trajectory.points[0].positions[0]<<" tilt: "<<goalTraj.trajectory.points[0].positions[1]);
//     return true;
// }

int main(int argc, char **argv)
{
    // Initiate the node
    ros::init(argc, argv, "ur5_follow_traj_server");
    ros::NodeHandle nh;

    // Initiate the goal parameters
    goalTraj.goal_tolerance.resize(joint_num);
    goalTraj.trajectory.joint_names.resize(joint_num);
    goalTraj.trajectory.points.resize(1);
    goalTraj.trajectory.points[0].positions.resize(joint_num);
    goalTraj.trajectory.points[0].velocities.resize(joint_num);
    goalTraj.trajectory.points[0].accelerations.resize(joint_num);

    // Define action client and wait for server
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> robotClient("/arm_controller/follow_joint_trajectory");
    if(!robotClient.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Can not find action server!");
    }

    // Declare the service servers
    ros::ServiceServer tolerance_server = nh.advertiseService("set_goal_tolerances", setGoalToleranceCallBack);
    ros::ServiceServer trajectory_server = nh.advertiseService("set_point_to_point_trajectory", setTrajectoryCallBack);
    ros::ServiceServer move_server = nh.advertiseService("move_robot_trajectory", moveTrajectoryCallback);
    // ros::ServiceServer move_safe_server = nh.advertiseService("move_robot_trajectory_safe", moveSafeCallback);

    run_finished_client = nh.serviceClient<cobot_control_pkg::run_finished>("run_finished");
    ros::service::waitForService("run_finished", ros::Duration(5));

    while(ros::ok())
    {
        if(run)
        {
            moveRobotTrajectory(goalTraj, trajduration, robotClient);
            run = false;
        }
        ros::spinOnce();
    }
    return 0;
}