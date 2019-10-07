#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <list>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "math.h"


class LocalPlanner
{
    private:
    int lookDistance;
    std::list<geometry_msgs::PoseStamped> path; 
    std::list<geometry_msgs::PoseStamped>::iterator goal;

    double angularP = 1.2;

    bool isWithinTolerance(geometry_msgs::PoseStamped position, std::list<geometry_msgs::PoseStamped>::iterator goal, double tolerance)
    {

    }


    public:

    LocalPlanner(nav_msgs::Path msg)
    {   
        path = std::list<geometry_msgs::PoseStamped>();

        for(const auto& pose : msg.poses)
        {
            path.push_back(pose);
        }
        
        goal = path.begin();
    }

    geometry_msgs::Twist followCarrot(geometry_msgs::PoseStamped odom)
    {
        geometry_msgs::Twist move;

        int dx = goal->pose.position.x - odom.pose.position.x , dy = goal->pose.position.y - odom.pose.position.y;
        int Dsq =  pow(dx, 2) + pow(dy, 2) ;

        double alpha = 2 * dx / Dsq;
        double r = 1 / alpha;

        move.angular.z = angularP * alpha;
        move.linear.x = 50;
        move.linear.y = 50;

        if( isWithinTolerance(odom, goal, 0.0001) )
        {
            if(goal == path.end())
            { goal = path.begin(); }
            else
            { goal++; }
        }

        return move;

    }

};


#endif