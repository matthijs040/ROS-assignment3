#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "../include/assignment3/subscriptionPublisher.h"
#include "../include/assignment3/Local-planner.h"

std::unique_ptr<nav_msgs::Path> Path;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("Received a plan path message!");
    if(!Path)
    {
        Path = std::make_unique<nav_msgs::Path>(*msg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_planner_node" );
    ros::NodeHandle n;
    ros::Subscriber pathSub = n.subscribe("/plan", 1, pathCallback);

    //SubscriptionPublisher<geometry_msgs::Twist, nav_msgs::Odometry> LPupdate("/cmd_vel", "/odom", &(Planner->followCarrot) );


    // Wait until receiving a path.
    ROS_INFO("awaiting a path on the /plan topic. \n");
    while(!Path)
    {
        ros::spinOnce();
    }

    // Path has been received.
    // Local planner sets up the publisher and subscriber.
    LocalPlanner planner = LocalPlanner(*Path, "/odom", "/cmd_vel");

    ros::spin();

    return 0;    
}
