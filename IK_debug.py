from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 


    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


    # set the dh parameter information as a dictionary
    dh_Params = { alpha0:      0, a0:     0, d1: 0.75, q1:         q1,    
                  alpha1: -pi/2., a1:  0.35, d2:    0, q2:-pi/2. + q2,
                  alpha2:      0, a2:  1.25, d3:    0, q3:         q3,
                  alpha3: -pi/2., a3:-0.054, d4:  1.5, q4:         q4,
                  alpha4:  pi/2., a4:     0, d5:    0, q5:         q5,
                  alpha5: -pi/2., a5:     0, d6:    0, q6:         q6,
                  alpha6:      0, a6:     0, d7:0.303, q7:         0 }


    # generate function to return the homogeneous transform between each link

    def h_transform(alpha,a,d,q):

        h_t = Matrix([[           cos(q),           -sin(q),          0 ,            a],
                      [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                      [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
                      [                0,                 0,           0,            1]])

        return h_t


    # perform the homogeneous transform between the links

    T0_T1 = h_transform(alpha0,a0,d1,q1).subs(dh_Params)
    T1_T2 = h_transform(alpha1,a1,d2,q2).subs(dh_Params) 
    T2_T3 = h_transform(alpha2,a2,d3,q3).subs(dh_Params)
    T3_T4 = h_transform(alpha3,a3,d4,q4).subs(dh_Params)
    T4_T5 = h_transform(alpha4,a4,d5,q5).subs(dh_Params)
    T5_T6 = h_transform(alpha5,a5,d6,q6).subs(dh_Params)
    T6_T7 = h_transform(alpha6,a6,d7,q7).subs(dh_Params)


    # To get the composition of all transforms from base to gripper multiply the individual matrices 


    T0_T2 = ( T0_T1 * T1_T2 )
    T0_T3 = ( T0_T2 * T2_T3 )
    T0_T4 = ( T0_T3 * T3_T4 )
    T0_T5 = ( T0_T4 * T4_T5 )
    T0_T6 = ( T0_T5 * T5_T6 )
    T0_T7 = ( T0_T6 * T6_T7 )


   # Correction Needed to Account for Orientation Difference Between
   # Definition of Gripper Link_G in URDF versus DH Convention

    R_y = Matrix([[ cos(-pi/2.),        0, sin(-pi/2.), 0 ],
                  [           0,       1.,           0, 0 ],
                  [-sin(-pi/2.),        0, cos(-pi/2.), 0 ],
                  [           0,        0,           0, 1 ]])

    R_z = Matrix([[     cos(pi), -sin(pi),           0, 0 ],
                  [     sin(pi),  cos(pi),           0, 0 ],
                  [           0,        0,          1., 0 ],
                  [           0,        0,           0, 1.]])

    R_corr = (R_z * R_y)

    # Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
    # With orientation correction applied

    T0_T7_corr = (T0_T7 * R_corr)


    #ik code starts here
    # Requested end-effector (EE) position
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    
    # store EE position in a matrix
    EE = Matrix([[px],
                 [py],
                 [pz]])

    # Requested end-effector (EE) orientation (roll pitch yaw that will be used to identify the rotation matrix for the ee)
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x,
         req.poses[x].orientation.y,
         req.poses[x].orientation.z,
         req.poses[x].orientation.w])

    # Find EE rotation matrix RPY (Roll, Pitch, Yaw)

    r,p,y = symbols('r p y')

    # Roll
    ROT_x = Matrix([[       1,       0,       0],
                    [       0,  cos(r), -sin(r)],
                    [       0,  sin(r),  cos(r)]])
    # Pitch
    ROT_y = Matrix([[  cos(p),       0,  sin(p)],
                    [       0,       1,       0],
                    [ -sin(p),       0,  cos(p)]])
    # Yaw
    ROT_z = Matrix([[  cos(y), -sin(y),       0],
                    [  sin(y),  cos(y),       0],
                    [       0,       0,       1]])

    ROT_EE = ROT_z * ROT_y * ROT_x


    # Correction Needed to Account for Orientation Difference Between
    # Definition of Gripper Link_G in URDF versus DH Convention

    ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
    ROT_EE = ROT_EE * ROT_corr

    # substiute the r p y to find the ee rotation matrix
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    # Calculate Wrest Center
    WC = EE - (0.303) * ROT_EE[:,2]

    # Calculate joint angles
    # Calculate theat1
    theta1 = atan2(WC[1],WC[0])

    # find the 3rd side of the triangle
    side_A = 1.501
    side_C = 1.25
    side_B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))

    #Cosine Laws SSS to find all inner angles of the triangle
    a = acos((side_B*side_B + side_C*side_C - side_A*side_A) / (2*side_B*side_C))
    b = acos((side_A*side_A + side_C*side_C - side_B*side_B) / (2*side_A*side_C))
    c = acos((side_A*side_A + side_B*side_B - side_C*side_C) / (2*side_A*side_B))

    #Find theta2 and theta3
    theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
    theta3 = pi/2 - (b+0.036) # 0.036 accounts for sag in link4 of -0.054m
    
    # Extract rotation matrix R0_3 from transformation matrix T0_3 the substitute angles q1-3
    R0_3 = T0_T1[0:3,0:3] * T1_T2[0:3,0:3] * T2_T3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

    # Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
    R3_6 = R0_3.inv("LU") * ROT_EE
    print("hey")
    print(R3_6)
    print(R3_6[2,2])

    # Euler angles from rotation matrix
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
    
    # select best solution based on theta5
    if (theta5 > pi) :
        theta4 = atan2(-R3_6[2,2], R3_6[0,2]) ## REVIEW THIS LINE SOLVING FOR THETA
        theta6 = atan2(R3_6[1,1],-R3_6[1,0])
    else:
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1],R3_6[1,0])



    ## Insert IK code here!
    
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    FK = T0_T7.evalf(subs={q1:theta1,q2:theta2,q3:theta3,q4:theta4,q5:theta5,q6:theta6})
    #go   = T0_T7.evalf(subs={q1:-0.65,q2:0.45, q3:-0.36,q4:0.95,q5:0.79,q6:0.49})  
    #print(go)

    ## End your code input for forward kinematics here!
    #######################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0],WC[1],WC[2]]       # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
