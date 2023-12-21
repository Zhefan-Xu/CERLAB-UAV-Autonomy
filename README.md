# CERLAB UAV Autonomy Framework
Welcome to the **CERLAB UAV Autonomy Framework**, a versatile and modular framework for autonomous unmanned aerial vehicles (UAVs). This framework comprises distinct components (simulator, perception, mapping, planning, and control) to achieve autonomous navigation, unknown exploration, and target inspection.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

![intro](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/assets/55560905/23a78d4f-a7a3-4c68-b80f-c6dbf6b0f090)


## I. Installation Guide
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
## II. Run Autonomy DEMO
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


## III. PX4 Simulation & Real Flight 
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


## IV. The Autonomy Modules
This section introduces the funtionality of each autonomy module included in this framework.

## V. Citation and Reference:
If you find this work useful, please cite the paper:
```
@inproceedings{xu2023vision,
  title={Vision-aided UAV navigation and dynamic obstacle avoidance using gradient-based B-spline trajectory optimization},
  author={Xu, Zhefan and Xiu, Yumeng and Zhan, Xiaoyang and Chen, Baihan and Shimada, Kenji},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1214--1220},
  year={2023},
  organization={IEEE}
}
```
