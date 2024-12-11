# Agricultural Robot
ENPM662 - Introduction to Robot Modeling Project 2: Agricultural Robot model and simulation for picking apples 

The following project made use of the CAD software Solidworks and software framework ROS and additional tools like Gazebo/RVizto simulate the task of a vehicle attached with a robot arm specialized in picking apples from trees.

![Screenshot 2024-12-08 144511](https://github.com/user-attachments/assets/3315576c-3ff7-4c6a-adf1-fe34a796a89b)

## Operating system
Ubuntu 20.04

## Required installations
- Python >= 3.8
- sympy
- ROS2 galatic
- Gazebo (consider migration for 2025 to Gazebo Garden)
- RVIZ2

## Key Contributions of Each Member

Jonathan -CAD models of the environment as the trees& apples plus set stable URDF assembling with vehicle done in (https://github.com/Jomaxdrill/ROS_vehicle) . Keep track of progress and testing/debug of every step of the project. Helped fix integrity issues of the model. Created & organized github repository. 

Kent- CAD model of the Robot Arm, DH table, forward kinematics validation and report of the project.

Robens- Built upon initial ros package by adding Lidar Sensor,IMU and vacuum gripper plugins & controllers. Debugging of interactions between objects. coding of teleop script and autonomous mode.

Hamsa - Inverse kinematics validation, workspace study , testing of trajectories execution for robot arm. Help in controller scripts.

## GENERAL SETUP
-Verify you have ROS2 galatic distribution installed and also CMAKE necessary installations. Refer to: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
On command line run:
```sh
echo $ROS_DISTRO
```
-Install previously the following packages and any additional in your linux distribution running on the terminal the command:
```sh 
sudo apt install python3-colcon-common-extensions
```
-Install all necessary dependencies and libraries with pip for insrtance. Recommended to run in your own python environment.

## Steps to build and run project.

### Install python packages 

In case of missing libraries you can use pip to install them:
```sh
pip install NAME_OF_LIBRARY
pip install 
pip install numpy
pip install matplotlib
```

### Create the workspace and install all dependencies priorly
Create a folder in your system and locate the src pkg
```sh
  mkdir -p ~/test_ws/src
  cd ~/test_ws/src
```
Clone the repository package inside this folder

```sh
   git clone https://github.com/Jomaxdrill/fruit_mobile_robot.git
```

You should get a folder called **fruit_vehicle_arm** with the content of the repository inside. It should be like the following example:

![image](https://github.com/user-attachments/assets/5bff6047-e9c1-4706-8eaa-b3508b1cbf40)

Downlod all the dependencies before runnning a build.
```sh
rosdep install -i --from-path src --rosdistro galactic -y
```
Run this command to be at root of your workspace (~/test_ws) and build your workspace
```sh
cd ../
colcon build 
```

In case you got an error while building regarding an include folder not found run this command while being in the workspace folder

![image](https://github.com/user-attachments/assets/0f4deb20-c25f-4379-8862-62dd9c4c906a)

```sh
mkdir -p ~/src/fruit_vehicle_arm/include/fruit_vehicle_arm
```
And run again the build command

```sh
colcon build --packages-select fruit_vehicle_arm
```

Source ROS (Package will be identified) However you can do make this default when opening the terminal by modifying the .bashrc file. <ins>Don't forget your user password to give permissions </ins>
```sh
sudo nano ~/.bashrc
```
![image](https://github.com/user-attachments/assets/56625fea-d3f4-4354-8d2e-7433444ea24b)

```sh
 source /opt/ros/galactic/setup.bash
```
**Source package to be identified**. Do this for every new terminal you open locating the Workspace folder:

```sh
source install/setup.bash
```
![image](https://github.com/user-attachments/assets/a2567ff8-e8f2-4170-8662-526f9e996bc6)


Before launch also run the following commands to install the controller dependencies:

```sh
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
sudo apt-get install ros-galactic-controller-manager
```
## Download all the additionals

### Add the odometry package

Odometry package is needed to track robot position and orientation as it contains plugin features. Vacuum gripper package is also necessary to be able to catch apples in gazebo.

To add it go to the folder **additionals** inside the package folder **fruit_vehicle_arm** and copy the folder **odometry** to the src folder of your workspace. Final result should look like this 

![image](https://github.com/user-attachments/assets/fd3d1fe8-e276-4cb2-94cf-b360c766586b)

You could confirm it was sucessfully done by running a general command **colcon build** and giving no errors .
```sh
colcon build 
```

![image](https://github.com/user-attachments/assets/e80ae09e-a7f7-432f-8738-2e86683d3a08)

### Add gazebo models
To run the world successfully custom objects of tree and apples need to be downloaded. 

To add it go to the folder **additionals** inside the package folder **fruit_vehicle_arm** and copy the folders **app_body** and **tree_body** to the location of your gazebo
Result should look like this:

![image](https://github.com/user-attachments/assets/0e0ba9b3-bb8a-42dc-88a6-66c4745eb35a)

![image](https://github.com/user-attachments/assets/8f11caf9-5a75-4e2b-b15a-0cee717c256c)


### Gazebo

Run the following command at the root of your workspace once you source it 

```sh
ros2 launch fruit_vehicle_arm gazebo.launch.py
```

This will display the robot in the world with apples and trees ready to run after the autonomous mode script.
Sometimes at first could not due to bugs with controller manager. Open a new terminal , locate the workspace folder and run again the command.

![image](https://github.com/user-attachments/assets/7baf4d11-24e4-435e-8eb3-6b82331bdd5e)


### RVIZ

To launch rviz open a new terminal, source the package and run: 

```sh
ros2 launch fruit_vehicle_arm display.launch.py
```
![image](https://github.com/user-attachments/assets/55048379-1dc8-4b6c-a653-2bc9f41e9a53)


Use the joint state publisher provided to try some poses for the arm for example

![image](https://github.com/user-attachments/assets/59c9436d-1387-49dc-9515-44cbf95966de)

To visualize lidar scanner run before in other terminal the gazebo simulation and after launch RVIZ again and use the **Add** button and look for select the **laser scanner** plugin 

![image](https://github.com/user-attachments/assets/ee4c938e-207f-4f44-8925-e966d6756841)

After check for the scan topic and it should be visible the lidar.

![image](https://github.com/user-attachments/assets/455e6d84-c6d9-4ba6-b621-6748abbc29ff)

As an extra step be sure inside the topic opetions, the Reliability Policy is set as **Best Effort**

![image](https://github.com/user-attachments/assets/dbe71e73-730e-4d4b-b87e-acc7cd1fd40d)


## Running scripts 

Inside the src folder of the package, these are the following scripts you can execute:

![image](https://github.com/user-attachments/assets/26bd8a88-1ecc-4bfe-abcb-65df7179351f)


### Forward and inverse kinematics 

Locate the folder where the scripts are is:

```sh
cd ~\src\fruit_vehicle_arm\src
```
Run the corresponding script:
```sh
python3 forward_position_kinematics_validation.py
```
This script will ask for the user to provide joint angles to give a final end effector matrix. This applies only for the robot arm.

Aditionally there is a matlab script (test_robot.m) using peter corke robotics toolbox for visualize the robot and interact with it.

![Screenshot 2024-12-05 140523](https://github.com/user-attachments/assets/1e3bff87-50c1-4728-b05e-63e8205146e8)

```sh
python3 Inverse_kinematics_and_validation.py
```
This script simulates the end effector of the robot drawing a circle f radius 2 inches. The plotted circle on the YZ - plane.

### Doing the grasping task

Be sure to open the gazebo by following the previous steps to open it. 

Open a new terminal,source the package and run:

```sh 
ros2 run fruit_vehicle_arm truck_and_arm_autonomous_control.py
```

The agricultural robot will spawn from position [0.0, 0.25, 0.05] and move from its origin near the trees to catch the apples attached to them with its robot arm.

![image](https://github.com/user-attachments/assets/031c92bd-e54f-4a63-bfcd-4dc81da03476)

### Teleoperation:

Close any previously running gazebo terminals then

Build and source fruit_vehicle_arm then launch gazebo with the command:

```sh
  colcon build --packages-select fruit_vehicle_arm
```

```sh
  ros2 launch fruit_vehicle_arm gazebo.launch.py
```
Open a new terminal and run:

```sh
  ros2 run fruit_vehicle_arm teleop.py
```

Robot can now be controlled with W, A, S, D with W and S increasing and decreasing velocity 
respectively and A and D turning the wheel left and right respectively. 

For moving the arm using numbers key combinations of 
- Keys arm1: 1,5
- Keys arm2: 2,6 
- Keys arm3: 3,7 
- Keys arm4: 4,8 

### Check the odom topic
Open a new terminal,source package and run:

```sh 
ros2 run fruit_vehicle_arm odom_subscriber.py
```
This will give information of the current position of the robot 

If odometry was settled correctly you can visualize it via the topic **/odom**, using programs as rqt_plot or plotjugger. For more information refer to the following useful link tutorial. All credits to the author https://www.youtube.com/watch?v=MnMGjvYxlUk
You can also run if installed
```sh 
ros2 run rqt_plot rqt_plot
```
![image](https://github.com/user-attachments/assets/e4f99df3-3217-4aff-919c-cc4ecce84ab1)

## Videolinks
### Apple picking 
https://drive.google.com/file/d/1bWp0tc4YbuVi7mIBvuK-GSt5amngVuD6/view?usp=sharing
### Teleoperation 
https://drive.google.com/file/d/17SFGPVSB7CHLmiUaYZs7CN_LTJuTty-l/view?usp=sharing
### Use with RVIZ
https://drive.google.com/file/d/1abLG8P37SsgXIo9mUsSpZqwMaZRfMZdK/view?usp=sharing



