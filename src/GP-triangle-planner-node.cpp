#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "../include/assignment3/subscriptionPublisher.h"

nav_msgs::Path generateTriangularPath(const geometry_msgs::Point& widthAndHeight)
{
    nav_msgs::Path path = nav_msgs::Path();
    int width = widthAndHeight.x, height = widthAndHeight.y, offset = widthAndHeight.z;
        
    geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();

    //Bottom left pose.
    path.poses.push_back(pose);

    //Bottom right pose.
    pose.pose.position.x += width;
    path.poses.push_back(pose);

    //Top left pose.
    pose.pose.position.x += offset;
    pose.pose.position.y += height;
    path.poses.push_back(pose);

    return path;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "triangle_planner_node" );
    ros::NodeHandle n;

    SubscriptionPublisher<nav_msgs::Path, geometry_msgs::Point> rectReceiver("/plan", "/trispec", generateTriangularPath );

    ROS_INFO("awaiting a point message for width, height and offset of the triangle on the /trispec topic \n");

    ros::spin();

    return 0;    
}
