#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "../include/assignment3/subscriptionPublisher.h"


nav_msgs::Path generateRectangularPath(const geometry_msgs::Point& widthAndHeight)
    {
        ROS_INFO("Received a point message for generation of a rectangular path. \n");
        nav_msgs::Path path = nav_msgs::Path();
        int width = widthAndHeight.x, height = widthAndHeight.y;
        
        geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();

        //Bottom left pose.
        path.poses.push_back(pose);

        //Bottom right pose.
        pose.pose.position.x += width;
        path.poses.push_back(pose);

        //Top right pose.
        pose.pose.position.y += height;
        path.poses.push_back(pose);

        //Top left pose.
        pose.pose.position.x -= width;
        path.poses.push_back(pose);

        //Final pose.
        pose.pose.position.y -= height;
        path.poses.push_back(pose);

        return path;        
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rectangle_planner_node" );
    ros::NodeHandle n;

    SubscriptionPublisher<nav_msgs::Path, geometry_msgs::Point> rectReceiver("/plan", "/rectspec", generateRectangularPath );

    ROS_INFO("awaiting a point message for height and width of the rectangle on the /rectspec topic \n");

    ros::spin();

    return 0;    
}
