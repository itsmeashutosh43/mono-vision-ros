#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <limits.h>
#include <unistd.h>


using namespace cv;
using namespace std;


void featureDetection(cv::Mat &img,std::vector<cv::Point2f>& points)
{
    vector<cv::KeyPoint> key_point1;
    int fast_threshold = 10;
    bool nonMaxSupress = true;
    cv::FAST(img,key_point1,fast_threshold,nonMaxSupress);

    KeyPoint::convert(key_point1, points);

}


void featureTracking(cv::Mat& img1, cv::Mat& img2,std::vector<cv::Point2f>& points1 , std::vector<cv::Point2f>& points2,std::vector<uchar>& status)
{
    std::vector<float> err;
    cv::Size winSize = cv::Size(21,21);

    cv::TermCriteria termCrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30,0.1);

    cv::calcOpticalFlowPyrLK(img1,img2,points1,points2,status,err,winSize,3,termCrit,0,0.001);


    int indexCorrection = 0;

    for(int i = 0; i< status.size() ; i++)
    {
        cv::Point2f pt = points2.at(i - indexCorrection);

        if ((status.at(i) == 0) || (pt.x < 0)  || (pt.y <0))
        {
            if ((pt.x < 0) ||(pt.y < 0))
            {
                status.at(i) = 0;
            }
            points1.erase(points1.begin() + i - indexCorrection);
            points2.erase(points2.begin() + i - indexCorrection);

            indexCorrection++;
        }
    } 
}

