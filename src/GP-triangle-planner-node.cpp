#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "subscriptionPublisher.h"
#include "Global-planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rectangle-planner-node" );
    ros::NodeHandle n;

    GlobalPlanner GP;
    SubscriptionPublisher<nav_msgs::Path, geometry_msgs::Point> rectReceiver("/plan", "/trispec", GP.generateTriangularPath);

    std::cout << "awaiting a point message for height and width of the rectangle on the /trispec topic \n";

    ros::spin();

    return 0;    
}
