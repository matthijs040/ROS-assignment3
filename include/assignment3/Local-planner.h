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
    double lookaheadDistance = 20.0;
    std::vector<geometry_msgs::PoseStamped> path; 
    std::vector<geometry_msgs::PoseStamped>::iterator goal;

    ros::NodeHandle n;
    ros::Publisher cmdPub;
    ros::Subscriber odomSub;

    double angularP = 1.2;
    double linearP = 1.0;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        processOdom(*msg);
    }

    // Determines the solutions for the given quadratic equation using the "a b c" formula
    // Returns: In case of 2 solutions:  the 2 X solutions as pair< X1, X2>
    //          In case of 1 solution:   a pair of < X1, NaN>
    //          In case of ima solution: a pair of <Xr, Xi>
    // https://www.programiz.com/cpp-programming/examples/quadratic-roots
    inline std::pair<double, double> solveQuadEQ(const double a, 
                                                 const double b, 
                                                 const double c)
    {
        std::pair<double, double> result = std::make_pair<double, double>( nan("") , nan("") );

        // D = B^2 - 4ac
        double d = b*b - 4 * a *c;

        if(d > 0)
        {
            result.first =  (-b + sqrt(d) ) / (2 * a);
            result.second = (-b - sqrt(d) ) / (2 * a);
        }
        else if (d == 0)
        {
            result.first =  (-b + sqrt(d) ) / (2 * a);            
        }
        else
        {
            result.first = -b / (2 * a);
            result.second = sqrt(-d) / (2 * a);
        }

        return result;
    }

    // Calculates the line equation from two given intersections A and B.
    // Returns: The equation in the "Y = A * X + B" form where the pair of doubles is < A, B >.
    // https://www.mathsisfun.com/algebra/line-equation-2points.html
    inline std::pair<double, double> getLineEQ(const geometry_msgs::Point& A, const geometry_msgs::Point& B)
    {
        double dx = B.x - A.x, dy = B.y - A.y;
        double a = dy / dx;

        // Y = a * X + b  =====>  b = Y / ( a * X )
        double b = A.y / ( a * A.x );

        return std::make_pair<double&, double&>(a , b);
    }

    // Calculates the look-ahead point from the starting point A, the goal point B, the current position C and a looking radius r.
    // Returns: In case of 2 intersects, the look-ahead point in the direction of B.
    //          In case of 1 intersect, the intersect point.
    //          In case of no intersects, an empty point (?)
    geometry_msgs::Point getLookaheadPoint(const geometry_msgs::Point& C, double r)
    {
        const geometry_msgs::Point B = this->goal->pose.position, 
                                   A = (this->goal--)->pose.position; 
        
        std::pair<double, double> lineEQ = getLineEQ(A, B);
        double a = lineEQ.first, b = lineEQ.second;

        // (X - C.X)^2 + (Y - C.Y)^2 = r^2
        // X^2 - 2*C.x * X + C.x^2 + Y^2 - 2C.y * Y + C.y^2 = r^2
        // X^2 - 2*C.x * X + C.x^2 + (a*X+b)^2 - 2*C.y * (a*X+b) + C.y^2 - r^2 = 0
        // X^2 - 2*C.x * X + C.x^2 + a * X^2 + 2*a*b*X + b^2 - 2C.y * a * X + 2C.y * b + C.y^2 - r^2 = 0
        // X^2 + a * X^2 + | 2 * a * b * X   - 2C.y * a * X  | C.x^2 + b^2 + 2C.y * b + c.y^2 - r^2 

        std::pair solution = solveQuadEQ(a + 1, (2 * a * b) - (2 * C.y * a), (C.x*C.x) + (b * b) + (2 * C.y * b) + (C.y * C.y) - ( r * r ) );


        
        // If both solutions are invalid. D < 0
        if(solution.first == nan("") )
        { 
            return geometry_msgs::Point(); 
        }
        // if only the second solution is invalid.
        else if(solution.second == nan("") )
        {
            // Point constructor does not allow non-default parameters on construction. >.<
            geometry_msgs::Point p = geometry_msgs::Point();
            p.x = solution.first;
            p.y = a * solution.first + b;
            return p;
        }
        else
        {
            double yr1 = a * solution.first + b, yr2 = a * solution.second + b;
            double dy1 = abs(B.y) - abs(yr1), dy2 = abs(B.y) - abs(yr2);
            double dx1 = abs(B.x) - abs(solution.first), dx2 = abs(B.x) - abs(solution.second);

            geometry_msgs::Point p = geometry_msgs::Point();
            if(dx1 + dy1 < dx2 + dy2)
            {
                p.x = solution.first; p.y = yr1;
                return p;
            }
            else
            {
                p.x = solution.second; p.y = yr2;
                return p;                
            }
        }
    }

    inline bool isWithinTolerance(nav_msgs::Odometry position, std::vector<geometry_msgs::PoseStamped>::iterator goal, double tolerance)
    {
        if( ( position.pose.pose.position.x < goal->pose.position.x + tolerance && position.pose.pose.position.x > goal->pose.position.x - tolerance ) &&
            ( position.pose.pose.position.y < goal->pose.position.y + tolerance && position.pose.pose.position.y > goal->pose.position.y - tolerance ) &&
            ( position.pose.pose.position.z < goal->pose.position.z + tolerance && position.pose.pose.position.z > goal->pose.position.z - tolerance ) )
        { return true; }
        else
        { return false; }
    }


    public:

    LocalPlanner(nav_msgs::Path path, std::string odometryTopicName, std::string cmdTopicName)
    {   
        if(path.poses.size() < 2)
        {
            throw std::out_of_range("A path cannot be created between less than two points. msg.poses is less than 2.");
        }
        this->path = path.poses;
        goal = this->path.begin();

        odomSub = n.subscribe(odometryTopicName, 1, &LocalPlanner::odomCallback, this );
        cmdPub  = n.advertise<geometry_msgs::Twist>(cmdTopicName, 1);
    }

    // NOTE: not sure if function should be public.
    void processOdom(const nav_msgs::Odometry& odom)
    {
        if( isWithinTolerance(odom, goal, 0.0001) )
        {
            if(goal == path.end())
            { goal = path.begin(); }
            else
            { goal++; }
            return;
        }
        geometry_msgs::Twist move;

        // TODO: Get look-ahead point from look ahead distance and its intersect with path.




        const double dx = goal->pose.position.x - odom.pose.pose.position.x , dy = goal->pose.position.y - odom.pose.pose.position.y;
        double Dsq =  pow(dx, 2) + pow(dy, 2) ;

        double curvature = 2 * dx / Dsq;
        //double radius = 1 / curvature;

        move.angular.z = angularP * curvature;

        move.linear.x = abs(linearP * dx);
        move.linear.y = abs(linearP * dy);
        move.linear.z = abs(linearP * 0);



        cmdPub.publish(move);

    }

};


#endif