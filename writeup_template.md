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

    $ git clone https://github.com/mkhuthir/RoboND-Kinematics-Project.git ~/catkin_ws 

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
  
To run forward kinematics test us:

    $ roslaunch kuka_arm forward_kinematics.launch
    
To run simulator use:

    $ rosrun kuka_arm safe_spawner.sh

To run IK Server use:

    $ rosrun kuka_arm IK_server.py 


### Kinematic Analysis 
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

#### URDF joint position and orientation extraction from the kr210.urdf.xacro file: 

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

#### URDF joint positions and orientations extraction table:

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

#### Translate x and z coordinates into link length and link offset for dh parameter table:

drawing one here !!!!!!!!



#### show all the DH Parameters in respect to a 6DOF drawing: 


drawing two here !!!!!!!!!!!!!!! 


#### Elements of the DH Parameter table for producing individual transforms and homogeneous transform:

Origin O(i) = intersection between Xi and Zi axis

a = Link Length: a(i-1) = Zi-1 - Zi along the X(i-1) axis

d = Link Offset: d(i) = X(i-1) - X(i) along Z(i) axis

alpha = Link Twist: alpha(i-1) = angle from Z(i-1) to Z(i) measured about Xi-1 using right hand rule

q = theta = Joint Angle: theta(i) = angle from X(i-1) to X(i) measured about Zi using right hand rule. all joint angles will be zero at initial Robot state in KR210 except joint 2 which has a -90 degree constant offset between X(1) and X(2).


#### begin the coding by importing all the stuff:

    import numpy as np
    from numpy import array
    from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
    from sympy.matrices import Matrix

#### establish variables for the dh table and homogeneous transform:

    q1, q2, q3, q4,q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4,d5,d6,d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


#### Kuka KR210 robot DH parameters:


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - 90 | 0.35 | 0 | -90 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 |  -90 | -0.05 | 1.5 | q4
4->5 | 90 | 0 | 0 | q5
5->6 | -90 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7


#### set the dh parameter information as a dictionary:

    dh_Params = { alpha0:      0, a0:     0, d1: 0.75, q1:         q1,    
                  alpha1: -pi/2., a1:  0.35, d2:    0, q2:-pi/2. + q2,
                  alpha2:      0, a2:  1.25, d3:    0, q3:         q3,
                  alpha3: -pi/2., a3:-0.054, d4:  1.5, q4:         q4,
                  alpha4:  pi/2., a4:     0, d5:    0, q5:         q5,
                  alpha5: -pi/2., a5:     0, d6:    0, q6:         q6,
                  alpha6:      0, a6:     0, d7:0.303, q7:         0 }



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.



#### generate function to return the homogeneous transform between each link:

image one!!!!!


    def h_transform(alpha,a,d,q):

        h_t = Matrix([[           cos(q),           -sin(q),          0 ,            a],
                      [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                      [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
                      [                0,                 0,           0,            1]])

        return h_t


#### perform the homogeneous transform between the links:

    T0_T1 = h_transform(alpha0,a0,d1,q1).subs(dh_Params)
    T1_T2 = h_transform(alpha1,a1,d2,q2).subs(dh_Params) 
    T2_T3 = h_transform(alpha2,a2,d3,q3).subs(dh_Params)
    T3_T4 = h_transform(alpha3,a3,d4,q4).subs(dh_Params)
    T4_T5 = h_transform(alpha4,a4,d5,q5).subs(dh_Params)
    T5_T6 = h_transform(alpha5,a5,d6,q6).subs(dh_Params)
    T6_T7 = h_transform(alpha6,a6,d7,q7).subs(dh_Params)


#### get the composition of all transforms from base to gripper multiply the individual matrices: 


    T0_T2 = ( T0_T1 * T1_T2 )
    T0_T3 = ( T0_T2 * T2_T3 )
    T0_T4 = ( T0_T3 * T3_T4 )
    T0_T5 = ( T0_T4 * T4_T5 )
    T0_T6 = ( T0_T5 * T5_T6 )
    T0_T7 = ( T0_T6 * T6_T7 )


#### Correction Needed to Account for Orientation Difference between definition of Gripper Link_G in URDF versus DH Convention:


    R_y = Matrix([[ cos(-pi/2.),        0, sin(-pi/2.), 0 ],
                  [           0,       1.,           0, 0 ],
                  [-sin(-pi/2.),        0, cos(-pi/2.), 0 ],
                  [           0,        0,           0, 1 ]])

    R_z = Matrix([[     cos(pi), -sin(pi),           0, 0 ],
                  [     sin(pi),  cos(pi),           0, 0 ],
                  [           0,        0,          1., 0 ],
                  [           0,        0,           0, 1.]])

    R_corr = (R_z * R_y)


#### Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7 with orientation correction applied:


    T0_T7_corr = (T0_T7 * R_corr)
    
#### Test the results:      

test_01: 

    T0_7 = T0_T7_corr.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
    
 image from python testing: 
    
 image from rviz joint information : 
 
 test_02: 
 
    T0_7 = T0_T7_corr.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
    
 image from python testing: 
    
 image from rviz joint information : 





#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


