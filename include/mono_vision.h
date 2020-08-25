#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class MonoVision{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;

    public:
    MonoVision();
    ~MonoVision();
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
};