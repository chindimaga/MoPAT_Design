# MoPAT_Design
> This is the source repo for MoPAT Design Project under Dr. Gaurav Trivedi, EEE Department, IITG.

# For this one, install ROS2 Dashing

# Running ROS2 package(still in development) 
### NB: -> represents next command, not in the same line

1. Create new directory and move in: mkdir -p ~/mopat\_ws/src -> cd ~/mopat\_ws/src
2. Copy mopat\_pkg from MoPAT\_Design/ROS2 to src
3. Clone ros2-perception, specifically for CvBridge package: git clone https://github.com/ros-perception/vision_opencv.git
4. Checkout to ros2 branch: cd vision\_opencv -> git checkout ros2
5. Install all dependencies
6. Build the pacakges: colcon build --symlink-install

To run simulation: ros2 launch mopat\_pkg sim\_launch

To run main system: ros2 launch mopat\_pkg main\_launch

To run individual nodes: ros2 run mopat\_pkg <node_name>

### NB: For more details on CvBridge: https://index.ros.org/p/cv_bridge/github-ros-perception-vision_opencv/#dashing

##### NOTE: To get logger output-
1. Go to opt/ros/dashing/lib/python3.6/site-packages/launch/actions
2. Open the file "execute process" as admin (sudo)
3. Find "emulate\_tty=False" in the file and set it to "emulate\_tty=True"

This would get the INFO logger output when using launch files

Based on - https://github.com/ros2/launch/issues/188
