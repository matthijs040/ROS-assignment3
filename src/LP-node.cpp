#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "../include/assignment3/Local-planner.h"

#include "../include/assignment3/tf-handler.hpp"


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
    ros::Publisher twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Wait until receiving a path.
    ROS_INFO("awaiting a path on the /plan topic. \n");
    while(!hasdata)
    {
        ros::spinOnce();
    }

    nav_msgs::Path path = Path;

    // Path has been received.
    // Local planner sets up the publisher and subscriber.
    LocalPlanner planner = LocalPlanner(path.poses);
    TFHandler tfrecv = TFHandler("/odom", "/base_link");

    // Main loop
    while( ros::ok() )
    {
        ros::spinOnce();

        tf::StampedTransform transform = tfrecv.getTransform();
        geometry_msgs::Twist twist = planner.processTF( transform ); 

        twistPub.publish(twist);

        ros::Duration(0.1).sleep();
    }

    return 0;    
}

