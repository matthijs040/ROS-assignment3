#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "subscriptionPublisher.h"
#include "Local-planner.h"




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rectangle-planner-node" );
    ros::NodeHandle n;

    LocalPlanner LP(nav_msgs::Path());
    SubscriptionPublisher<nav_msgs::Path, geometry_msgs::Point> rectReceiver("/plan", "/rectspec", LP.followCarrot);


    std::cout << "awaiting a point message for height and width of the rectangle on the /rectspec topic \n";

    ros::spin();

    return 0;    
}
