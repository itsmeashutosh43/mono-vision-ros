#include "mono_vision.h"
#include "utils.h"

MonoVision::MonoVision():it_(nh_)
{
    sub_ = it_.subscribeCamera("/mybot/camera/image_rect", 1 , &MonoVision::imageCb, this);
    cv::namedWindow("Image from camera"); 
}

MonoVision::~MonoVision()
{
    cv::destroyWindow("Image from camera");
}

void MonoVision::imageCb(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    
    try{
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("CV bridge exception %s",e.what());
        return;
    }

    curr_image_c = cv_ptr->image;
    cv::cvtColor(curr_image_c, curr_image, COLOR_BGR2GRAY);
    if (init)
    {
        init = false;
        cam_model_.fromCameraInfo(info_msg);
        intrinsic = cam_model_.fullIntrinsicMatrix();
        featureDetection(curr_image, points1);
        std::cout<<"Here"<<std::endl;

        prev_image_c = curr_image_c;
        return;
    }
    cv::cvtColor(prev_image_c, prev_image, COLOR_BGR2GRAY);
    featureTracking(prev_image, curr_image ,points1, points2, status);

    E = cv::findEssentialMat(points2, points1, intrinsic, RANSAC, 0.99, 1.0 ,mask);

    std::cout<<E<<std::endl;

    cv::imshow("Image from camera", cv_ptr->image);
    cv::waitKey(3);
    prev_image_c = curr_image_c;

}


int main(int argc , char** argv)
{
    ros::init(argc, argv ,"Mono_vision");
    MonoVision mv;
    ros::spin();
    return 0;
    
}