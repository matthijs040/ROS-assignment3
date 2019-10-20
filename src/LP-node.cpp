#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "../include/assignment3/subscriptionPublisher.h"
#include "../include/assignment3/Local-planner.h"


nav_msgs::Path Path = nav_msgs::Path();
bool hasdata = false;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("Received a plan path message!");
    hasdata = true;

    Path = *msg;
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_planner_node" );
    ros::NodeHandle n;
    ros::Subscriber pathSub = n.subscribe("/plan", 1, pathCallback);

    //SubscriptionPublisher<geometry_msgs::Twist, nav_msgs::Odometry> LPupdate("/cmd_vel", "/odom", &(Planner->followCarrot) );


    // Wait until receiving a path.
    ROS_INFO("awaiting a path on the /plan topic. \n");
    while(!hasdata)
    {
        ros::spinOnce();
    }

    nav_msgs::Path path = Path;

    // Path has been received.
    // Local planner sets up the publisher and subscriber.
    LocalPlanner planner = LocalPlanner(path.poses, "/odom", "/cmd_vel");

    ros::spin();

    return 0;    
}

