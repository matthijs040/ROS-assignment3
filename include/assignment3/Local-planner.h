#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"


class LocalPlanner
{
    private:
    int lookDistance;


    public:

    LocalPlanner()
    {     }

    geometry_msgs::Twist followCarrot(geometry_msgs::PoseStamped goal)
    {
        geometry_msgs::Twist move;


        return move;

    }

};


#endif