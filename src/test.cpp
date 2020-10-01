#include<ros/ros.h>
#include<nav_msgs/Odometry.h>


class Test{
    public:

    void odomCallback(const nav_msgs::Odometry& odom)
    {
        odom_initialized = true;
        odom_time = odom.header.stamp;
        odom_omega = odom.twist.twist.angular.z;

    }

    void camOdomCallback( const nav_msgs::Odometry& odom)
    {
        cam_odom_initialized = true;
        cam_odom_time = odom.header.stamp;
        cam_odom_omega = odom.twist.twist.angular.z;

    }

    void spin(const ros::TimerEvent& e)
    {
        if ((!cam_odom_initialized) || (!odom_initialized)) return;

        diff = std::fabs(odom_time.toSec() - cam_odom_time.toSec());

        if (diff > 0.1) return;

        double err = std::fabs(odom_omega - cam_odom_omega);

        if (err < 0.01) return;

        acc_err += err;
        std::cout << "Accumulated error is "<< acc_err << std::endl; 



    }

    Test()
    {
        odom_sub = nh_.subscribe("/odom", 1 , &Test::odomCallback, this);
        cam_odom_sub = nh_.subscribe("/cam_odom", 1 , &Test::camOdomCallback, this);
        timer = nh_.createTimer(ros::Duration(0.5), &Test::spin , this);
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub, cam_odom_sub;
    ros::Time odom_time, cam_odom_time;
    ros::Timer timer;
    double diff,  acc_err;
    double odom_omega, cam_odom_omega;
    bool odom_initialized = false;
    bool cam_odom_initialized = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MonoVisionTest");
    ros::NodeHandle nh;
    Test t;
    ros::spin();

}