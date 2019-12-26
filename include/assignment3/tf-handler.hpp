#ifndef TF_HANDLER_HPP
#define TF_HANDLER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

/**
 * @brief Class for the handling of TF data on a user defined time-interval.
 * 
 */
class TFHandler
{
    private:

    ros::NodeHandle n;
    std::function< void( tf::StampedTransform ) > receiver;
    std::string joint1, joint2;

    ros::WallTimerCallback wtCallback;    
    ros::WallTimer timer;

    tf::TransformListener* listener;


    /**
     * @brief Timer callback to send the most recent transform to the user provided function.
     * NOTE: getTransform might error and send a default transform. 
     * Input validation must be preformed in the user provided function.
     */
    void timerCB( void )
    {
        if( receiver )
        {
            tf::StampedTransform tf = getTransform(); 
            receiver( tf );
        }
    }

    public:
    /**
     * @brief Construct a new TFHandler object
     * In case the user provides a receiver function an internal timer is constructed.
     * This timer fires on the user provided frequency. When this happens, the latest transform is sent to the receiver
     * @param joint1 
     * @param joint2 
     * @param recv 
     */
    TFHandler( std::string joint1 = "/joint", std::string joint2 = "/base", std::function< void( tf::StampedTransform ) > recv = nullptr )
    : n( ros::NodeHandle() )
    , receiver( recv )
    , joint1(joint1)
    , joint2(joint2)
    {
        listener = new tf::TransformListener();

        // Receiver function provided.
        if( receiver )
        {
            // Construct the timer and its callback to send data to the receiver.
            
            wtCallback = boost::bind( &TFHandler::timerCB, this ); 
            timer = n.createWallTimer( ros::WallDuration( 0.1 ) , wtCallback);
        }
    }

    /**
     * @brief User callable function to get the latest StampedTransform the listener has received.
     *  Looking up a transform is an operation that can throw exceptions. 
     *  These are all caught and, in case of an exception, a default StampedTransform is returned.
     * @return tf::StampedTransform 
     */
    tf::StampedTransform getTransform()
    {
        tf::StampedTransform transform = tf::StampedTransform();

        try
        {
           listener->lookupTransform(joint1, "/base_link", ros::Time(0), transform );         
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        return transform;
        
    } 
};

#endif
