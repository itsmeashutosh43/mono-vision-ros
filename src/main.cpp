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
        std::cout<<intrinsic<<std::endl;
        kp = featureDetection(curr_image, points1);
        prev_image_c = curr_image_c.clone();
        return;
    }

    cv::cvtColor(prev_image_c, prev_image, COLOR_BGR2GRAY);

    vector<uchar> status;
    featureTracking(prev_image, curr_image ,points1, points2, status);

    cv::KeyPoint::convert(points2 , kp1);

    E = cv::findEssentialMat(points2, points1, intrinsic, RANSAC, 0.99, 2.0 ,mask);

    cv::recoverPose(E, points2 , points1 , intrinsic, R, t, mask);
    now = ros::Time::now();

    if(first_transformation)
    {
        first_transformation = false;
        R_f = R.clone();
        t_f = t.clone();
        R_prev = R.clone();
        prev = now;
        return;
    }

    
        
    // R is instanaeous change in orientation
    // Send R with pose_stamped odom message
    float dt = now.toSec() - prev.toSec();
    cv::Mat dR = R - R_prev;
    std::cout<<dt<<std::endl;
    S = (dR / dt) * R.t();

        

    std::cout<<points1.size()<<std::endl;

    if (points1.size() < 60)
    {
        kp = featureDetection(prev_image,points1);
        featureTracking(prev_image,curr_image,points1, points2, status);
        cv::KeyPoint::convert(points2 , kp1);
    }

    cv::drawKeypoints(curr_image_c, kp1, curr_image_kp);

    
    
    cv::imshow("Image from camera", curr_image_kp);
    cv::waitKey(3);
    prev_image_c = curr_image_c.clone();
    points1 = points2;
    prev = now;
    R_prev = R.clone();


}


int main(int argc , char** argv)
{
    ros::init(argc, argv ,"Mono_vision");
    MonoVision mv;
    ros::spin();
    return 0;
    
}