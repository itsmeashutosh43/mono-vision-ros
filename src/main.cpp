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
    featureDetection(curr_image, points1);

    if (init)
    {
        init = false;
        cam_model_.fromCameraInfo(info_msg);
        intrinsic = cam_model_.fullIntrinsicMatrix();
        std::cout<<"Here"<<std::endl;

        prev_image_c = curr_image_c;
        return;
    }
    cv::cvtColor(prev_image_c, prev_image, COLOR_BGR2GRAY);

    featureTracking(prev_image, curr_image ,points1, points2, status);

    E = cv::findEssentialMat(points2, points1, intrinsic, RANSAC, 0.99, 1.0 ,mask);

    cv::recoverPose(E, points2 , points1 , intrinsic, R, t, mask);

    if(first_transformation)
    {
        first_transformation = false;
        R_f = R.clone();
        t_f = t.clone();
    }

    
    if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)) )
    {
        t_f = t_f + (R_f * t);
        R_f = R * R_f;

        // R is instanaeous change in orientation
        // Send R with pose_stamped odom message
    }

    std::cout << R_f << std::endl;



    cv::imshow("Image from camera", cv_ptr->image);
    cv::waitKey(3);
    prev_image_c = curr_image_c.clone();
    points2 = points1;


}


int main(int argc , char** argv)
{
    ros::init(argc, argv ,"Mono_vision");
    MonoVision mv;
    ros::spin();
    return 0;
    
}