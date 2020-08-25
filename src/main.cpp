#include "mono_vision.h"
#include "utils.h"

MonoVision::MonoVision():it_(nh_)
{
    image_sub = it_.subscribe("/mybot/camera/image_raw", 1 , &MonoVision::imageCb, this);
    cv::namedWindow("Image from camera"); 
}

MonoVision::~MonoVision()
{
    cv::destroyWindow("Image from camera");
}

void MonoVision::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("CV bridge exception %s",e.what());
        return;
    }

    cv::imshow("Image from camera", cv_ptr->image);
    cv::waitKey(3);


}


int main(int argc , char** argv)
{
    ros::init(argc, argv ,"Mono_vision");
    MonoVision mv;
    ros::spin();
    return 0;
    
}