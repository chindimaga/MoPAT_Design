# MoPAT_Design
> This is the source repo for MoPAT Design Project under Dr. Gaurav Trivedi, EEE Department, IITG.

# Running ROS2 package(still in development)
1. Clone the repo, copy the ROS2/mopat_pkg to workspace/src
2. Clone ros2-perception, specifically for CvBridge package. Steps - https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/#dashing
3. Install all dependencies and build the packages using "colcon build --symlink-install"
4. Use ros2 launch mopat_pkg sim\_launch.py


##### NOTE: To get logger output-
1. Go to opt/ros/dashing/lib/python3.6/site-packages/launch/actions
2. Open the file "execute process" as admin (sudo)
3. Find "emulate\_tty=False" in the file and set it to "emulate\_tty=True"

This would get the INFO logger output when using launch files

Based on - https://github.com/ros2/launch/issues/188
