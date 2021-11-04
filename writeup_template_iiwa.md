## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

#### Installation steps:


Clone this repository to your home directory:

    ```sh
    $ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
    ```
    
Install missing ROS dependencies using the rosdep install command:

    $ cd ~/catkin_ws/
    $ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

Run catkin_make from within your workspace to build the project:

    $ cd ~/catkin_ws/
    $ catkin_make

Run the following shell commands to source the setup files:

    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

For demo mode make sure the demo flag is set to true in inverse_kinematics.launch file under

    ~/catkin_ws/src/kuka_arm/launch/
    
#### Demo Mode:
  
run forward kinematics test:

    $ roslaunch kuka_arm forward_kinematics.launch
    
run simulator:

    $ rosrun kuka_arm safe_spawner.sh

run IK Server:

    $ rosrun kuka_arm IK_server.py 


### Kinematic Analysis 
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

URDF joint position and orientation extraction from the kr210.urdf.xacro file: 

      <!-- joints -->
      <joint name="fixed_base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </joint>
      <joint name="joint_1" type="revolute">
        <origin xyz="0 0 0.33" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="link_1"/>
        <axis xyz="0 0 1"/>
        <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
      </joint>
      <joint name="joint_2" type="revolute">
        <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
        <parent link="link_1"/>
        <child link="link_2"/>
        <axis xyz="0 1 0"/>
        <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
      </joint>
      <joint name="joint_3" type="revolute">
        <origin xyz="0 0 1.25" rpy="0 0 0"/>
        <parent link="link_2"/>
        <child link="link_3"/>
        <axis xyz="0 1 0"/>
        <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
      </joint>
      <joint name="joint_4" type="revolute">
        <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
        <parent link="link_3"/>
        <child link="link_4"/>
        <axis xyz="1 0 0"/>
        <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
      </joint>
      <joint name="joint_5" type="revolute">
        <origin xyz="0.54 0 0" rpy="0 0 0"/>
        <parent link="link_4"/>
        <child link="link_5"/>
        <axis xyz="0 1 0"/>
        <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
      </joint>
      <joint name="joint_6" type="revolute">
        <origin xyz="0.193 0 0" rpy="0 0 0"/>
        <parent link="link_5"/>
        <child link="link_6"/>
        <axis xyz="1 0 0"/>
        <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
      </joint>

URDF joint positions and orientations to generate an extraction table:

0   | joint | parent | child | x | y | z | r | p | y
--- | --- | --- | --- | --- |--- | --- | --- | --- | --- |
0 | fixed_base | base_footprint | base_link | 0 | 0 | 0 | 0| 0 | 0
1 | joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0
2 | joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0
3 | joint_3 | link_2 | link_3  | 0 | 0 | 1.25 | 0 | 0 | 0
4 | joint_4 | link_3 | link_4  | 0.96 | 0 | -0.054  | 0 | 0 | 0
5 | joint_5 | link_4 | link_5  | 0.54 | 0 | 0 | 0 | 0 | 0
6 | joint_6 | link_5 | link_6  | 0.193 | 0 | 0 | 0 | 0 | 0
7 | gripper | link_6 | grip_link  | 0.193 | 0 | 0 | 0 | 0 | 0
. | total | |  | 2.153 | 0 | 1.946 | 0 | 0 | 0


0   | joint | parent | child | x | y | z | r | p | y
--- | --- | --- | --- | --- |--- | --- | --- | --- | --- |
0 | joint_0 | base_footprint | base_link | 0 | 0 | 0 | 0| 0 | 0
1 | joint_1 | link_0 | link_1 | 0 | 0 | 0.15 | 0 | 0 | 0
2 | joint_2 | link_1 | link_2 | 0 | 0 | 0.19 | 90 | 0 | 180
3 | joint_3 | link_2 | link_3  | 0 | 0.21 | 0 | 90 | 0 | 180
4 | joint_4 | link_3 | link_4  | 0.0 6 | 0 | -0.054  | 0 | 0 | 0
5 | joint_5 | link_4 | link_5  | 0.54 | 0 | 0 | 0 | 0 | 0
6 | joint_6 | link_5 | link_6  | 0.193 | 0 | 0 | 0 | 0 | 0
7 | gripper | link_6 | grip_link  | 0.193 | 0 | 0 | 0 | 0 | 0
. | total | |  | 2.153 | 0 | 1.946 | 0 | 0 | 0


Translate x and z coordinates into link length and link offset for finding dh parameter table:

![alt text](IMAGES/image4.jpg)

Elements of the DH Parameter table for producing individual transforms and homogeneous transform:

    Origin O(i) = intersection between Xi and Zi axis

    a = Link Length: a(i-1) = Zi-1 - Zi along the X axis

    d = Link Offset: d(i) = Xi-1 - Xi along Z axis

    alpha = Link Twist: alpha(i-1) = angle from Z(i-1) to Z(i) 
    
    q = theta = Joint Angle: theta(i) = angle from X(i-1) to X(i) measured about Zi using right hand rule.


DH Parameters in respect to a 6DOF drawing: 

![alt text](IMAGES/image3.jpg)


Begin the coding by importing all the proper libraries:

    import numpy as np
    from numpy import array
    from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
    from sympy.matrices import Matrix

Establish variables for the dh table and homogeneous transform:

    q1, q2, q3, q4,q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4,d5,d6,d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


Kuka KR210 robot DH parameters:


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - 90 | 0.35 | 0 | -90 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 |  -90 | -0.05 | 1.5 | q4
4->5 | 90 | 0 | 0 | q5
5->6 | -90 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7


