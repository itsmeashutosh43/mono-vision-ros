# mono-vision-ros
ROS-based monocular vision algorithm. 


## Overview
The standard approach is to extract a sparse set of salient image features (e.g. points,lines) in each image; match them in successive frames using invariant feature descriptors; robustly recover both camera motion and structure using epipolar geometry; finally, refine the pose and structure through reprojection error minimization.

This is a fully robocentric representation of the filter state where we avoid major consistency problems while exhibiting accurate tracking performance and high robustness.

## Formulation
The main task of VO is to compute relative transformation between two consecutive frames and then concatenate to get the entire trajectory. However, due to high noise in visual odometry, one bad estimation can affect the entire trajectory. Additionally, we get an estimation of scaled translation whose scale in unknown. Therefore we have used visual odometry **only to estimate the robot's instantaneous angular velocity**. This estimation should not be used as a standalone estimation. It should be fused with IMU estimates. 

## General Steps
This project is inspired by [this repo] where all steps are explained in much detail. However, there are many changes that I made to make this Structure from motion problem to become more robust to raw data. 

Firstly, I will breifly mention each detailed steps and finally I will describe my additional changes in greater detail.

### Steps
1) Undistort the camera image. This was done with image_proc ROS package.

2) Calculate features in the first frame

3) Calculate corresponding features in the second image by estimating optical flow.

4) Translate and scale the feature data points so they are centered at origin and the average distance to the origin is root 2.

5) Calculate essential matrix on the scaled datapoint.

6) Enforce the essential matrix to have rank 2.

7) Rescale the Essential matrix. 

8) Use essential matrix to filter out bad estimates.

9) Calculate rotation matrix R from essential matrix.

10) Give R in SO(3) we can calculate angular velocity.


## How to Run
```
cd /in/your/workspace/src
git clone https://github.com/itsmeashutosh43/mono-vision-ros.git
cd ..
catkin_make
roslaunch mono_vision_ros mybot.launch
rosrun mono_vision_ros mono_vision # in another terminal

```

## Publications: 

* /cam_odom [nav_msgs/Odometry]

## Limitations

1) For consistent estimation, visual odometry needs to have over 1000-1500 features per frame. However, in this simulation the camera can only capture around 100 features per frame. 

2) Bad Estimations of rotation matrix are filtered out therefore it cannot be used to integrate over time to get pose estimation. 

3) I've used this method in a real-robot where it worked well in conjuction with other open source softwares in a rich environment. However I cannot share that code, therefore I've developed this project as a small replication of it. **This code repo is not ready to be used in real-world applications.** 





## References

Mono Vision implementation on KITTI dataset https://github.com/avisingh599/mono-vo

Tutorial on Visual Odometry - by Davide Scaramuzza (http://rpg.ifi.uzh.ch/visual_odometry_tutorial.html)




## More details soon...

Contact: chapagainashutosh8@gmail.com

