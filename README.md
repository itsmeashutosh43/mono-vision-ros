# mono-vision-ros
ROS-based monocular vision algorithm. 

The standard approach is to extract a sparse set of salient image features (e.g. points,lines) in each image; match them in successive frames using invariant feature descriptors; robustly recover both camera motion and structure using epipolar geometry; finally, refine the pose and structure through reprojection error minimization.

This is a fully robocentric representation of the filter state where we avoid major consistency problems while exhibiting accurate tracking performance and high robustness.

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


## References

Mono Vision implementation on KITTI dataset https://github.com/avisingh599/mono-vo

Tutorial on Visual Odometry - by Davide Scaramuzza (http://rpg.ifi.uzh.ch/visual_odometry_tutorial.html)



Contact: chapagainashutosh8@gmail.com

## More details soon...
