#include "mono_vision.h"
#include "utils.h"

MonoVision::MonoVision():it_(nh_)
{
    sub_ = it_.subscribeCamera("/camera/rgb/image_rect", 1 , &MonoVision::imageCb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("cam_odom", 1000);
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
    preprocess_points(points1, pre_points1,transformation1);
    assert(points1.size() == pre_points1.size());

    preprocess_points(points2, pre_points2,transformation2);
    assert(points2.size() == pre_points2.size());

    E = cv::findEssentialMat(pre_points2, pre_points1, intrinsic, RANSAC, 0.99999, 2.0 ,mask);

    cv::Mat w, u, vt;
    SVD::compute(E, w, u, vt);

    cv::Mat d = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 0); 

    
    E = u * d * vt;
    
    transformation1.convertTo(transformation1,CV_64F);
    transformation2.convertTo(transformation2,CV_64F);

    E = transformation1.t() * E * transformation2;

    if (!calcErr(E, points1, points2))
    {
        return;
    }


    cv::recoverPose(E, points2 , points1 , intrinsic, R, t, mask);

    // Since the robot cannot move about the z axis, R(2,2) of the rotation matrix R should be 1
    if (!nearlyEquals(R.at<double>(2,2)))
    {
        return;
    }

    cleanRotation(R);

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

    cv::Mat fundamentalMatrix = (cv::Mat)cv::findFundamentalMat(points1, points2);
    
    
        
    // R is instanaeous change in orientation
    // Send R with pose_stamped odom message
    //std::cout<<E<<std::endl;
    //std::cout<<R*R.t()<<std::endl;
    float dt = now.toSec() - prev.toSec();
    cv::Mat dR = R - R_prev;
    S = (dR / dt) * R.t();
    std::cout<<S<<std::endl;

    double z = S.at<float>(1,0);

    nav_msgs::Odometry data;
    data.child_frame_id = "camera";

    data.twist.twist.angular.x = 0;
    data.twist.twist.angular.y = 0;
    data.twist.twist.angular.z = z;

    odom_pub_.publish(data);



        

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
