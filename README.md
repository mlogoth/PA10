## Synopsis

ROS packages for the integration of MoveIt framework, ros_control and Gazebo for the Mitsubishi PA10 7-DOF Arm.

## Dependences
* **ROS Packages:**
    * [MoveIt ROS](http://moveit.ros.org/)
    * [joint state publisher](http://wiki.ros.org/joint_state_publisher)
    * [robot state publisher](http://wiki.ros.org/robot_state_publisher)
    * [joint limits interface](http://wiki.ros.org/joint_limits_interface)
    * [xacro](http://wiki.ros.org/xacro)
    * [ros_control](http://wiki.ros.org/ros_control)
    * [controller_manager](http://wiki.ros.org/controller_manager)
    * [gazebo ros control](http://wiki.ros.org/gazebo_ros_control)

*Installation*:
```sh
$ sudo apt-get install ros-indigo-moveit-ros-full ros-indigo-moveit-ros-control-interface ros-indigo-moveit-planners* ros-indigo-ompl
```
```sh
$ sudo apt-get install ros-indigo-joint-state-publisher ros-indigo-robot-state-publisher ros-indigo-joint-limits-interface 
```
```sh
$ sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers ros-indigo-gazebo-ros-control
```
* **Controllers to be used:**
    * velocity controllers
    * effort controllers
    * position controllers
    * [joint trajectory controller](http://wiki.ros.org/joint_trajectory_controller)
    * [joint state controller](http://wiki.ros.org/joint_state_controller)

*Installation*:
```sh
$ sudo apt-get install ros-indigo-velocity-controllers ros-indigo-position-controllers ros-indigo-effort-controllers ros-indigo-joint-trajectory-controller ros-indigo-joint-state-controller
```
## Install
```sh
$ roscd && cd ../src
$ git clone https://github.com/mlogoth/PA10.git
$ cd .. && catkin_make
```

## Demo
Visualization of PA10 running MoveIt framework.
```sh
roslaunch pa10_moveit_config demo.launch
```
Fake controllers are used.

## Real Robot Experiment
1) run the ```tcpcontrol``` script on __PA10 controller computer__:
```sh
./Desktop/TCPModes/tcpvelocities/tcpcontrol
```
2) run the hardware interface and the moveit framework on your computer:
```sh
roslaunch pa10_moveit_config demo_velocity.launch
```
3) Move the robot end effector using MoveIt framework.

## Gazebo
:bangbang:

** Not ready Yet **
