# CERLAB UAV Autonomy Framework
Welcome to the **CERLAB UAV Autonomy Framework**, a versatile and modular framework for autonomous unmanned aerial vehicles (UAVs). This framework comprises distinct components (simulator, perception, mapping, planning, and control) to achieve autonomous navigation, unknown exploration, and target inspection.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

**Contact Email**: zhefanx@andrew.cmu.edu

![intro](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/23a78d4f-a7a3-4c68-b80f-c6dbf6b0f090)

## I. The Autonomy Modules Introduction
The funtionality of each autonomy module included in this framework in alphabetical order:
 - ```autonomous_flight```: The autonomous flight package integrating all other modules for various tasks. [details](https://github.com/Zhefan-Xu/autonomous_flight)
 - ```global_planner```: The global waypoint planner library for autonomous robots. [details](https://github.com/Zhefan-Xu/global_planner)
 - ```trajectory_planner```: The trajectory olanning library for autonomous robots. [details](https://github.com/Zhefan-Xu/trajectory_planner)
 - ```map_manager```: The 3D mapping library for autonomous robots. [details](https://github.com/Zhefan-Xu/map_manager)
 - ```uav_simulator```: The lightweight Gazebo/ROS-based simulator for unmanned aerial vehicles. [details](https://github.com/Zhefan-Xu/uav_simulator)
 - ```time_optimizer```: The optimal trajectory time allocation library for autonomous robots. [details](https://github.com/Zhefan-Xu/time_optimizer)
 - ```tracking_controller```: The trajectory tracking controller for autonomous robots. [details](https://github.com/Zhefan-Xu/tracking_controller)
 - ```onboard_detector```: The dynamic obstacle detection and tracking algorithm for autonomous robots. [details](https://github.com/Zhefan-Xu/onboard_detector)
 - ```remote_control```: The Rviz configuration and launch files for easy visualization. [details](https://github.com/Zhefan-Xu/remote_control)

## II. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on the ROS packages: [octomap](https://wiki.ros.org/octomap), [mavros](https://wiki.ros.org/mavros), and [vision_msgs](https://wiki.ros.org/vision_msgs). Installing the package with the following commands:

```
# step1: install dependencies
sudo apt install ros-${ROS_DISTRO}-octomap* && sudo apt install ros-${ROS_DISTRO}-mavros* && sudo apt install ros-${ROS_DISTRO}-vision-msgs

# step 2: clone this repo to your workspace
cd ~/catkin_ws/src
git clone --recursive https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy.git

# optional: switch to simulation branch for autonomous_flight
# the default branch is for real flight and PX4 simulation
cd path/to/autonomous_flight
git checkout simulation

# step 3: follow the standard catkin_make procedure
cd ~/catkin_ws
catkin_make
```
## III. Run Autonomy DEMO
This section shows the most typical ones: **navigation**, **exploration**, and **inspection**. Note that the default environment and the flight parameters might be different from the demos shown as below. Please check [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) for changing the simulation environments and [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight) for adjusting flight parameters.

Before getting started, please make sure you are in the ```simulation``` branch of the submodule [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight) for the following demos (please check the link for detailed explanations):
```
cd path/to/autonomous_flight
git branch

# if the output says you are not in the simulation branch, please run the following (otherwise please ignore): 
git checkout simulation
cd ~/catkin_ws
catkin_make
```

### a. **Autonomous Navigation:**  Navigating to a given goal position and avoiding collisions.    

```
# start simulator
roslaunch uav_simulator start.launch

# open the Rviz visualization
roslaunch remote_control dynamic_navigation.rviz # if your test env has dynamic obstacles

# run the navigation program
roslaunch autonomous_flight dynamic_navigation.launch # if your test env has dynamic obstacles

# --------------------------------------------------------------------------------------
# (alternatively, if your test env is purely  static, you can run the following instead)
# open the Rviz visualization
roslaunch remote_control navigation.rviz # if your test env only has static obstacles

# run the navigation program
roslaunch autonomous_flight navigation.launch # if your test env only has static obstacles
```

Once the robot is hovering at the predefined height (check the terminal output messages), you can use the ```2D Nav Goal``` to click a goal point in ```Rviz``` and you can see example results shown below:

https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/31f4e6eb-857c-43d0-a02c-8defa8eea12c


### b. **Autonomous Exploration: Exploraing an unknown environments and create a map.**

```
# start simulator
roslaunch uav_simulator start.launch

# open the Rviz visualization
roslaunch remote_control exploration.rviz 

# run the navigation program
roslaunch autonomous_flight dynamic_exploration.launch
```

The example exploration process is shown in the video demo as below:

https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/e0d953de-a542-49c3-86ca-b44d77ff7653


### c. **Autonomous Inspection: Navigating to the target and inspecting it with a zig-zag path.**

```
# start simulator
roslaunch uav_simulator start.launch

# open the Rviz visualization
roslaunch remote_control inspection.rviz 

# run the navigation program
roslaunch autonomous_flight dynamic_inspection.launch
```

The example inspection process is shown in the video demo as below:

https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/0e580d08-7003-4732-a5b0-5d4041f7d3fd


## IV. PX4 Simulation & Real Flight 
This section talks about running this framework in the PX4-based simulation or conducting real flight experiments. Please first follow the PX4 simulation installation guide as provided in [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator).

Before getting started, please make sure you are in the ```px4``` branch of the submodule [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight) for the following demos (please check the link for detailed explanations):
```
cd path/to/autonomous_flight
git branch

# if the output says you are not in the px4 branch, please run the following (otherwise please ignore): 
git checkout px4
cd ~/catkin_ws
catkin_make
```

### a. PX4 Simulation Experiments
The purpose of having another PX4 simulation (besides the simulator we have shown in the previous section) is to simulate **ALL** behaviors that we might encounter in the real flight. To run the same demos in the previous section, the only change we need to do is to run the following command to start the simulator instead.
```
# start PX4 simulator
roslaunch uav_simulator px4_start.launch
```

### b. Real Flight Experiments
Once you have tested the flight in the PX4 simulation, the real flight experiments will have exactly the same behavior as you saw in the simulation. The inputs required for this framework in the real flight experiments are:
 - ```The robot pose/odometry```: The framework requires a SLAM/VIO system that can estimate the robot states.
 - ```The depth image```: The framework expects the depth image to detect objects and construct the map.

Check all the parameters in the [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight)  accordingly before the actual flight!!!

### c. Examples of Real Flight Experiments
a. The example of real flight experiment for **autonomous navigation**:
  
https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/f635a4c9-6996-44d2-85fe-5fecaed33054

b. The example of real flight experiment for **autonomous exploration**:
  
https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/ea838535-b052-4713-b1b2-4690bf4a7369

c. The example of real flight experiment for **autonomous inspection**:
  
https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/4878fc3a-528d-4c82-a621-71ffadc092ab



## V. Citation and Reference
If you find this work useful, please consider to cite our papers:
- Zhefan Xu\*, Xiaoyang Zhan\*, Yumeng Xiu, Christopher Suzuki, Kenji Shimada, "Onboard dynamic-object detection and tracking for autonomous robot navigation with RGB-D camera”, IEEE Robotics and Automation Letters (RA-L), 2024. [\[paper\]](https://ieeexplore.ieee.org/document/10323166) [\[video\]](https://youtu.be/9dKX3BRnxyw).
- Zhefan Xu\*, Christopher Suzuki\*, Xiaoyang Zhan, Kenji Shimada, "Heuristic-based Incremental Probabilistic Roadmap for Efficient UAV Exploration in Dynamic Environments”, arxiv, 2023. [\[paper\]](https://arxiv.org/pdf/2303.00132.pdf) [\[video\]](https://youtu.be/fjVJCgDemjc?si=9nsWhReMeJH5JC3Q).
- Zhefan Xu and Kenji Shimada, “Quadcopter Trajectory Time Minimization and Robust Collision Avoidance via Optimal Time Allocation”, axriv, 2023. [\[paper\]](https://arxiv.org/abs/2309.08544) [\[video\]](https://youtu.be/wI8KGcxsyMI?si=1QfDPrm8s6Hfv8vf)
- Zhefan Xu, Baihan Chen, Xiaoyang Zhan, Yumeng Xiu, Christopher Suzuki, and Kenji Shimada, “A Vision-Based Autonomous UAV Inspection Framework for Unknown Tunnel Construction Sites With Dynamic Obstacles”, IEEE Robotics and Automation Letters (RA-L), 2023. [\[paper\]](https://ieeexplore.ieee.org/document/10167713) [\[video\]](https://youtu.be/MSNp-hg9RCQ?si=VQ1Yl3OWrw9MfNe1)
- Zhefan Xu\*, Xiaoyang Zhan\*, Baihan Chen, Yumeng Xiu, Chenhao Yang, and Kenji Shimada, "A real-time dynamic obstacle tracking and mapping system for UAV navigation and collision avoidance with an RGB-D camera”, IEEE International Conference on Robotics and Automation (ICRA), 2023. [\[paper\]](https://ieeexplore.ieee.org/abstract/document/10161194) [\[video\]](https://youtu.be/u5zblVx8KRc?si=3c2AC9mc6pZBUypd).
- Zhefan Xu, Yumeng Xiu, Xiaoyang Zhan, Baihan Chen, and Kenji Shimada, “Vision-aided UAV Navigation and Dynamic Obstacle Avoidance using Gradient-based B-spline Trajectory Optimization”, IEEE International Conference on Robotics and Automation (ICRA), 2023. [\[paper\]](https://ieeexplore.ieee.org/abstract/document/10160638) [\[video\]](https://youtu.be/xlMAL8aBHHg?si=4E5vShz7spxZDzps)
- Zhefan Xu, Di Deng, and Kenji Shimada, “Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap”, IEEE Robotics and Automation Letters (RA-L), 2021. [\[paper\]](https://ieeexplore.ieee.org/document/9362184) [\[video\]](https://youtu.be/ileyP4DRBjU?si=KFJLt-rLCa3tFaRH)

## VI. Acknowledgement
The author would like to express his sincere gratitude to Professor Kenji Shimada for his great support and all CERLAB UAV team members who contribute to the development this research.

## VII. Write at the End
This repository concludes my first two-year Ph.D. work at CMU, and I would like to share it to contribute to the autonomous UAV research community. At the beginning, I truly felt the frustration as an autonomous robot researcher due to the lack of a comprehensive development framework. As a result, I hope my code can provide researchers a comprehensive and easy-to-understand platform and let them focus on algorithm/theory design instead of the software pipeline. In the meanwhile, this research and developed framework are not perfect and can be further improved; <ins>**thus, I am actively looking for collaborators in research and development to improve the UAV/robot autonomy level. Please don't hesitate to reach out for potential collaboration!**<ins>

