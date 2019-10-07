#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

class GlobalPlanner
{
    private:


    public:

    GlobalPlanner()
    {     }

    nav_msgs::Path generateRectangularPath(geometry_msgs::Point widthAndHeight)
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

    nav_msgs::Path generateTriangularPath()
    {

    }

};


#endif