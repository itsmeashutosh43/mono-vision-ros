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
#include <cmath>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <limits.h>
#include <unistd.h>


using namespace cv;
using namespace std;


vector<cv::KeyPoint> featureDetection(cv::Mat &img,std::vector<cv::Point2f>& points)
{
    vector<cv::KeyPoint> key_point1;
    int fast_threshold = 40;
    bool nonMaxSupress = true;
    cv::FAST(img,key_point1,fast_threshold,nonMaxSupress);

    KeyPoint::convert(key_point1, points);


    return key_point1;

}


void featureTracking(cv::Mat& img1, cv::Mat& img2,std::vector<cv::Point2f>& points1 , std::vector<cv::Point2f>& points2,std::vector<uchar>& status)
{
    std::vector<float> err;
    cv::Size winSize = cv::Size(21,21);

    cv::TermCriteria termCrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30,0.1);

    cv::calcOpticalFlowPyrLK(img1,img2,points1,points2,status,err,winSize,1,termCrit,0,0.001);


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

std::string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}



void preprocess_points(std::vector<cv::Point2f> &points, std::vector<cv::Point2f> &prepoints, cv::Mat& transformation)
{
    //Calculate the centre abong all points
    float avgX = 0, avgY = 0;
    for (int i = 0 ; i < points.size() ; i++)
    {
        avgX += points.at(i).x;
        avgY += points.at(i).y;
    }

    avgX /= points.size();
    avgY /= points.size();

    //centre the points
    std::vector<cv::Point2f> temp_points;
    float avg_dist = 0;
    for (int i = 0 ; i < points.size() ; i++)
    {
        float newX = points.at(i).x - avgX;
        float newY = points.at(i).y - avgY;
        temp_points.push_back(cv::Point2f( newX, newY ));
        avg_dist += std::sqrt( std::pow(newX,2) + std::pow(newY,2) ); 
    }

    avg_dist /= points.size();

    double s = std::sqrt(2) / avg_dist;

    transformation = (Mat_<float>(3,3) << s, 0, -s * avgX, 0, s, -s * avgY, 0, 0, 1);

    cv::Mat cv_points = cv::Mat(points);

    cv::Mat hpoints(cv::Size(points.size(), 3), CV_32FC1);; 
    cv::convertPointsToHomogeneous(cv_points,hpoints);

    hpoints = hpoints.reshape(1,points.size());

    
    string ty =  type2str(transformation.type() );
    string ty1 =  type2str(hpoints.type());
    
    cv::Mat mat = hpoints * transformation ;
    cv::convertPointsFromHomogeneous(mat, prepoints);
    
}

