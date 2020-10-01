#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <nav_msgs/Odometry.h>


class MonoVision{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    cv_bridge::CvImagePtr cv_ptr;
    image_geometry::PinholeCameraModel cam_model_;
    nav_msgs::Odometry msg;
    ros::Time now; 
    ros::Time prev;
    ros::Publisher odom_pub_;  

    bool init = true;
    bool first_transformation = true;

    std::vector<cv::Point2f> points1, points2, pre_points1, pre_points2;
    cv::Mat transformation1;
    cv::Mat transformation2;
    std::vector<cv::KeyPoint> kp, kp1;
    cv::Mat E,R,t,mask,F;
    cv::Mat R_f, t_f, S, R_prev;
    cv::Mat prev_image , prev_image_c;
    cv::Mat distortionCoeffs;
    cv::Mat curr_image, curr_image_c , curr_image_kp;
    cv::Matx33d intrinsic;
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);

    public:
    MonoVision();
    ~MonoVision();
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg); 
};