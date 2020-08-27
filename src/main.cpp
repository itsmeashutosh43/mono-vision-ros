#include "mono_vision.h"
#include "utils.h"

MonoVision::MonoVision():it_(nh_)
{
    sub_ = it_.subscribeCamera("/mybot/camera/image_rect", 1 , &MonoVision::imageCb, this);
    cv::namedWindow("Image from camera"); 
    cv::namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

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
        std::cout<<"Here"<<std::endl;
        kp = featureDetection(curr_image, points1);
        prev_image_c = curr_image_c;
        return;
    }
    cv::cvtColor(prev_image_c, prev_image, COLOR_BGR2GRAY);

    vector<uchar> status;
    featureTracking(prev_image, curr_image ,points1, points2, status);

    cv::KeyPoint::convert(points2 , kp1);

    
    E = cv::findEssentialMat(points2, points1, intrinsic, RANSAC, 0.99, 1.0 ,mask);

    cv::recoverPose(E, points2 , points1 , intrinsic, R, t, mask);

    if(first_transformation)
    {
        first_transformation = false;
        R_f = R.clone();
        t_f = t.clone();
    }

    
    if ((t.at<double>(0) > t.at<double>(2)) && (t.at<double>(0) > t.at<double>(1)) )
    {
        t_f = t_f + 1 * (R_f * t);
        R_f = R * R_f;

        // R is instanaeous change in orientation
        // Send R with pose_stamped odom message
        float yaw, pitch , roll;

        yaw = std::atan(R.at<double>(1,0)/R.at<double>(0,0));
        pitch = std::atan(R.at<double>(2,0)/std::sqrt(R.at<double>(2,1) * R.at<double>(2,1) + R.at<double>(2,2) * R.at<double>(2,2)));
        roll = std::atan(R.at<double>(2,1) / R.at<double>(2,2));

        std::cout<< "Yaw "<<yaw << " pitch "<<pitch<<" Roll "<<roll<<std::endl;

    }


    std::cout<<points1.size()<<std::endl;

    if (points1.size() < 100)
    {
        std::cout<<"mm here"<<std::endl;
        kp = featureDetection(prev_image,points1);
        featureTracking(prev_image,curr_image,points1, points2, status);
        cv::KeyPoint::convert(points2 , kp1);
    }

    cv::drawKeypoints(curr_image_c, kp1, curr_image_kp);
    
    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(1)) + 100;
    cv::circle(traj, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);
    
    cv::imshow("Trajectory",traj);

    cv::imshow("Image from camera", curr_image_kp);
    cv::waitKey(3);
    prev_image_c = curr_image_c.clone();
    points1 = points2;


}


int main(int argc , char** argv)
{
    ros::init(argc, argv ,"Mono_vision");
    MonoVision mv;
    ros::spin();
    return 0;
    
}