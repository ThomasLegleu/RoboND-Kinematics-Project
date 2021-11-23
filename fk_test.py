#import all the stuff
import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix

#establish variables for the dh table and homogeneous transform

q1, q2, q3, q4,q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4,d5,d6,d7 = symbols('d1:8')
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

#T0_7 = T0_T7_corr.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})

T0_7 = T0_T7_corr.evalf(subs={q1: 0.77, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
print(T0_T7_corr)
print(T0_7)

