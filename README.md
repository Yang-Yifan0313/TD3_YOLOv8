# Enhanced Obstacle Avoidance in Robot


This project implements an advanced mobile robot navigation system that integrates Deep Reinforcement Learning (DRL) with YOLOv8-based object detection. The system operates in a ROS Gazebo simulator environment, utilizing a Twin Delayed Deep Deterministic Policy Gradient (TD3) neural network architecture for path planning and collision avoidance.

Key Features:

  1.Sensor Fusion:
  
     *RGB camera for visual input
     *Depth camera for distance measurement
     *LiDAR sensor for obstacle detection and ranging
     *Integration of visual and LiDAR data to avoid the curse of dimensionality

  2.Object Detection:

     *Implementation of YOLOv8 for real-time movable obstacle detection
     *Dynamic environmental information processing

  3.Navigation System:

     *Continuous action space for smooth navigation
     *Real-time obstacle avoidance


Technical Implementation:

  * Framework: ROS Noetic
  * Operating System: Ubuntu 20.04
  * Python Version: 3.8.10
  * Deep Learning Framework: PyTorch 1.10
  * Simulation Environment: ROS Gazebo
  * Neural Network: Twin Delayed Deep Deterministic Policy Gradient (TD3)




**Reference**


More detailed information can be found in the paper! However, it is still under writing~~

## Installation
Main dependencies: 

* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [PyTorch](https://pytorch.org/get-started/locally/)
* [Tensorboard](https://github.com/tensorflow/tensorboard)

Clone the repository:
```shell
$ cd ~
$ git clone https://github.com/Yang-Yifan0313/TD3_YOLOv8.git
```

You should know that this repository includes two workspaces, namely catkin_ws and YOLOv8_detection.
Compile the workspace:
```shell
$ cd ~/TD3_YOLOv8/catkin_ws
### Compile
$ catkin_make_isolated
### Compile another
$ cd ~/TD3_YOLOv8/YOLOv8_detection
$ catkin_make_isolated
```

Open a terminal and set up sources:
```shell
###Open the gazebo world file, simulated agent file and visualization platform.
$ cd ~/TD3_YOLOv8/catkin_ws/src/multi_robot_scenario
$ roslaunch multi_robot_scenario TD3_world.launch  
$ roslaunch gazebo_ros pioneer3dx.gazebo.launch
$ rviz

###Start YOLOv8_detection  program
###If you have correct conda environment
$ conda activate dxtorch
$ cd ~/TD3_YOLOv8/YOLOv8_detection/src/Yolov8_ros/yolov8_ros/launch
$ roslaunch yolov8_ros yolo_v8.launch
###Display RGB image and depth image with added boxes 
```

Run the training:
```shell
$ cd ~/TD3_YOLOv8/TD3
$ python3 train_velodyne_td3.py
```

To check the training process on tensorboard:
```shell
$ cd ~/TD3_YOLOv8/TD3
$ tensorboard --logdir runs
```

To kill the training process:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```

Once training is completed, test the model:
```shell
$ cd ~/TD3_YOLOv8/TD3
$ python3 test_velodyne_td3.py
```




