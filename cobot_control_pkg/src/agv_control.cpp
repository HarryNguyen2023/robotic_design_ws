#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int count  = 0;

int main(int argc, char** argv)
{
    // Intiate the node
    ros::init(argc, argv, "agv_control");
    ros::NodeHandle nh;

    // Declare velocity publisher
    ros::Publisher agv_vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    geometry_msgs::Twist agv_vel_msg;
    agv_vel_msg.linear.x = 0.4;
    agv_vel_msg.angular.z = 0.0;

    ros::Rate rate(50);
    while(ros::ok())
    {
        if(count < 1000)
        {
            agv_vel_publisher.publish(agv_vel_msg);
            count++;
        }
        else
        {
            agv_vel_msg.linear.x = 0;
            agv_vel_msg.angular.z = 0;
            agv_vel_publisher.publish(agv_vel_msg);
        } 
        rate.sleep();
    }
    return 0;
}