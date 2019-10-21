#ifndef SERVOING_H
#define SERVOING_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <optional>
#include "tf/tf.h"

class Servoing 
{
    private:
    float angularP = 1.2;
    float linearP = 1.2;
    
    geometry_msgs::Point goal;

    public:

    bool angFinished, linFinished;

    // Create a Servoing object that, when polled for a new command with updated telemetry will provide the right command to reach the destination.
    // The end condition of which would be a Twist message with no motion.
    Servoing(geometry_msgs::Point goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    // Override the goal with a new goal position + orientation.
    void setGoal(geometry_msgs::Point goal)
    {
        this->goal = goal;
        angFinished = false;
        linFinished = false;
    }

    void setGoal(geometry_msgs::PoseStamped goal)
    {
        setGoal(goal.pose.position);
    }

    // Possible function when making the goal a vector of goals? Making it set the newer goal after reaching its current goal.
    // void queueGoal(geometry_msgs::Point::ConstPtr& goal);

    geometry_msgs::Twist updatePath(const geometry_msgs::Point position, const double currentAngle)
    {
        geometry_msgs::Twist move = geometry_msgs::Twist();

        if(!Servoing::angFinished)
        {

            const double desiredAngle =  atan2(goal.y - position.y, goal.x - position.x) ;

            if( currentAngle < desiredAngle + 0.01 && currentAngle > desiredAngle  - 0.01 )
            {
                // Reset starting angle. To be recalculated when a new goal is set.
                angFinished = true;
                //std::cout << "rotation finished. \n";
                //return geometry_msgs::Twist();
            }
            else
            {

                double deviation =  abs(desiredAngle - currentAngle);
                move.angular.z = angularP *  deviation ;
                //std::cout << "moving at X: " + std::to_string( angles::to_degrees(currentAngle) ) + " \n";

                // TODO: Threshold movement?

            }
        }
        if(!linFinished)
        {
             const double cX = position.x,  cY = position.y,  cZ = position.z,
                          gX = goal.x,      gY = goal.y,      gZ = goal.z;


            //currentAngle < desiredAngle + 0.0001 && currentAngle > desiredAngle  - 0.0001
            if(  ( cY < gY + 0.01 && cY > gY - 0.01 )
              && ( cX < gX + 0.01 && cX > gX - 0.01 )
              && ( cZ < gZ + 0.01 && cZ > gZ - 0.01 ) )
            {
                linFinished = true;
                std::cout << "linear motion finished. \n";
                return geometry_msgs::Twist();
            }
            else
            {
                //std::cout << "currently in shoot! \n";

                 double deviationX = abs(gX - cX);
                 double deviationY = abs(gY - cY);
                 double deviationZ = abs(gZ - cZ);

                if(deviationX < 1 && deviationY < 1)
                { }
                else if(deviationY < 1)
                { deviationY = 1; }
                else if(deviationX < 1)
                { deviationX = 1; }

                move.linear.x = abs(linearP * ( ( deviationX * deviationY) / 2 ) );
                //move.linear.y = abs(linearP * deviationY);
                //move.linear.z = abs(linearP * deviationZ);

                // TODO: Threshold movement?

            }
        }

        return move;
    }


    // Calculates the point-shoot twist required to make a robot move towards goal.
    // This calculation requires the most recent odom update.
    geometry_msgs::Twist updatePath(const nav_msgs::Odometry& odomUpdate)
    {
        geometry_msgs::Twist move = updatePath(odomUpdate.pose.pose.position, tf::getYaw(odomUpdate.pose.pose.orientation) );
        return move;
       
    }

}; // class Servoing

//Servoing_H
#endif