#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

class GlobalPlanner
{
    private:


    public:

    GlobalPlanner()
    {     }

    nav_msgs::Path generateRectangularPath(const geometry_msgs::Point& widthAndHeight) const
    {
        nav_msgs::Path path;
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

        return path;        
    }

    nav_msgs::Path generateTriangularPath(const geometry_msgs::Point& widthAndHeight) const
    {
        nav_msgs::Path path;
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
    }

};


#endif