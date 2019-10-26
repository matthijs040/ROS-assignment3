#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"

#include "math.h"
#include "servoing.h"

class LocalPlanner
{
    private:
    double lookaheadDistance = 2.0;
    std::vector<geometry_msgs::PoseStamped> path; 
    int goal;

    ros::NodeHandle n;
    ros::Publisher cmdPub;
    ros::Subscriber odomSub;
    Servoing controller;
    bool finalgoal;

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
    // Returns: The equation in the "y = a * x + b" form where the pair of doubles is < a, b >.
    // https://www.mathsisfun.com/algebra/line-equation-2points.html
    inline std::pair<double, double> getLineEQ(const geometry_msgs::Point& A, const geometry_msgs::Point& B)
    {
        double dx = B.x - A.x, dy = B.y - A.y;
        double a = dy / dx;

        // y = a * x + b  =====>  b = y / ( a * x )
        double b = A.y / ( a * A.x );

        // If it is a straight horizontal line b is equal to the y coordinate of either point.
        if(a == 0)
        { 
            b = A.y; 
        }

        // If it is a straight vertical line a == 0, b is equal to the x coordinate of either point.
        if( std::isinf(a) )
        {
            b = A.x;
        }

        if( std::isnan(b) )
        { 
            //throw std::runtime_error("Cannot calculate line between the two given points.");
            ROS_WARN("b, the result of lineEQ returned NAN! Setting b to 0 to keep running!");
            b = 0;
        }

        return std::make_pair<double&, double&>(a , b);
    }

    // Calculates the look-ahead point from the starting point A, the goal point B, the current position C and a looking radius r.
    // Returns: In case of 2 intersects, the look-ahead point in the direction of B.
    //          In case of 1 intersect, the intersect point.
    //          In case of no intersects, an empty point (?)
    geometry_msgs::Point getLookaheadPoint(const geometry_msgs::Point& C, double r)
    {
        const geometry_msgs::Point B = path.at( goal ).pose.position, 
                                   A = path.at( goal - 1 ).pose.position; 
        
        std::pair<double, double> solution;
        
        std::pair<double, double> lineEQ = getLineEQ(A, B);
        double a = lineEQ.first, b = lineEQ.second;


        // If getLineEQ returns a vertical line. The formula is X = b
        if( std::isinf(a) )
        {
            //Y^2 - 2CyY + b^2 - 2Cxb + Cx^2 + Cy^2 - r^2 = 0

            const double aTerms = 1;
            const double bTerms = -2 * C.y;
            const double cTerms = pow(b, 2) - 2 * C.x * b + pow(C.x, 2) + pow(C.y, 2) - pow(r, 2) ;

            solution = solveQuadEQ( aTerms, bTerms, cTerms );

            // If both solutions are invalid. D < 0
            if( std::isnan(solution.first) )
            { 
                return geometry_msgs::Point(); 
            }
            // if only the second solution is invalid.
            else if( std::isnan(solution.second) )
            {
                // Point constructor does not allow non-default parameters on construction. >.<
                geometry_msgs::Point p = geometry_msgs::Point();
                p.x = solution.first;
                p.y = a * solution.first + b;
                return p;
            }
            else
            {
                const double xr1 = b, xr2 = b;
                const double dy1 = std::abs(B.y - solution.first), dy2 = std::abs(B.y - solution.second);
                const double dx1 = std::abs(C.x - xr1), dx2 = std::abs(C.x - xr2);

                 geometry_msgs::Point p = geometry_msgs::Point();
                if(dx1 + dy1 < dx2 + dy2)
                {
                    p.x = xr1; p.y = solution.first;
                    return p;
                }
                else
                {
                    p.x = xr2; p.y = solution.second;
                    return p;                
                }
            }
        }
        else
        {
            // (X - C.X)^2 + (Y - C.Y)^2 = r^2
            // X^2 - 2*C.x * X + C.x^2 + Y^2 - 2C.y * Y + C.y^2 = r^2
            // X^2 - 2*C.x * X + C.x^2 + (a*X+b)^2 - 2*C.y * (a*X+b) + C.y^2 - r^2 = 0
            // X^2 - 2*C.x * X + C.x^2 + a * X^2 + 2*a*b*X + b^2 - 2C.y * a * X + 2C.y * b + C.y^2 - r^2 = 0
            // X^2 + a * X^2 + | 2 * a * b * X   - 2C.y * a * X  | C.x^2 + b^2 + 2C.y * b + c.y^2 - r^2 

            const double aTerms = pow(a, 2) + 1;
            const double bTerms = ( -2 * C.x ) + ( 2 * a * (b - C.y) );
            const double cTerms = pow(C.x, 2) + pow( (b - C.y), 2) - pow(r, 2);

            solution = solveQuadEQ( aTerms, bTerms, cTerms );
        }

        // If both solutions are invalid. D < 0
        if( std::isnan(solution.first) )
        { 
            return geometry_msgs::Point(); 
        }
        // if only the second solution is invalid.
        else if( std::isnan(solution.second) )
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
            double dy1 = std::abs(B.y - yr1), dy2 = std::abs(B.y - yr2);
            double dx1 = std::abs(B.x - solution.first), dx2 = std::abs(B.x - solution.second);

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

    inline bool isWithinTolerance(const geometry_msgs::Point& position, const geometry_msgs::PoseStamped& goal, double tolerance)
    {
        if( ( position.x < goal.pose.position.x + tolerance && position.x > goal.pose.position.x - tolerance ) &&
            ( position.y < goal.pose.position.y + tolerance && position.y > goal.pose.position.y - tolerance ) &&
            ( position.z < goal.pose.position.z + tolerance && position.z > goal.pose.position.z - tolerance ) )
        { return true; }
        else
        { return false; }
    }

    inline bool isWithinTolerance(const nav_msgs::Odometry& position, const geometry_msgs::PoseStamped& goal, double tolerance)
    {
        isWithinTolerance(position.pose.pose.position, goal, tolerance);
    }

    std::vector<geometry_msgs::PoseStamped> validatePath(std::vector<geometry_msgs::PoseStamped> path)
    {
        if(path.size() < 2 )
        {
            throw std::out_of_range("path");
        }
        else
        {
            return path;
        }
    }

    public:

    LocalPlanner(std::vector<geometry_msgs::PoseStamped> path, std::string odometryTopicName, std::string cmdTopicName)
    : path( validatePath(path) ), goal( 1 ), controller( path.at(goal).pose.position )
    {   
        odomSub = n.subscribe(odometryTopicName, 1, &LocalPlanner::odomCallback, this );
        cmdPub  = n.advertise<geometry_msgs::Twist>(cmdTopicName, 1);
        finalgoal = false;
    }

    // NOTE: not sure if function should be public.
    void processOdom(const nav_msgs::Odometry& odom)
    {
        geometry_msgs::Point lookaheadPoint = getLookaheadPoint(odom.pose.pose.position, lookaheadDistance);

        if( isWithinTolerance(lookaheadPoint, path.at(goal), 0.1) )
        {
            if( goal != path.size() - 1 )
            { 
                goal++;
            }
            else
            { finalgoal = true;}
        }

        if(finalgoal)
        {
            controller.setGoal(path.at(goal));        
        }
        else
        {
            controller.setGoal(lookaheadPoint);
        }

        cmdPub.publish( controller.updatePath(odom) );

    }

};


#endif