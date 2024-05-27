#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):

        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5

        theta1 = self.theta1
        theta2 = self.theta2

        ctheta1 = np.cos(theta1)
        ctheta2 = np.cos(theta2)
        stheta1 = np.sin(theta1)
        stheta2 = np.sin(theta2)


        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        c1 = np.cos(q1)
        c2 = np.cos(q2)
        c3 = np.cos(q3)
        c4 = np.cos(q4)
        c5 = np.cos(q5)
        c6 = np.cos(q6)
        c7 = np.cos(q7)

        s1 = np.sin(q1)
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s4 = np.sin(q4)
        s5 = np.sin(q5)
        s6 = np.sin(q6)
        s7 = np.sin(q7)

        ###Dependancies for J11

        el1_11 = -l3*(c1*s3 + c2*c3*s1) - l2*s1*s2
        el2_11 = -l4*stheta1*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)
        el3_11 = -l4*ctheta1*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)
        el4_11 = -l5*ctheta2*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        el5_11 = -l5*stheta2*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        
        J_11 = el1_11 + el2_11 + el3_11 + el4_11 + el5_11
        
        ###Dependancies for J12

        el1_12 = l4*stheta1*(c1*c2*s4 - c1*c3*c4*s2)
        el2_12 = -l5*stheta2*(s6*(c1*c2*c4 + c1*c3*s2*s4) - c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5))
        el3_12 = -l5*ctheta2*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5))
        el4_12 = -l4*ctheta1*(c1*c2*c4 + c1*c3*s2*s4) + l2*c1*c2 - l3*c1*c3*s2
        J_12 = el1_12 + el2_12 + el3_12 + el4_12

        ###Dependancies for J13

        el1_13 = l5*ctheta2*(s6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) 
        el2_13 = -l3*(c3*s1 + c1*c2*s3) 
        el3_13 = -l5*stheta2*(c6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) + s4*s6*(c3*s1 + c1*c2*s3)) 
        el4_13 = -l4*stheta1*c4*(c3*s1 + c1*c2*s3) 
        el5_13 = -l4*ctheta1*s4*(c3*s1 + c1*c2*s3)

        J_13 = el1_13 + el2_13 + el3_13 + el4_13 + el5_13

        ###Dependancies for J14

        el1_14 = l4*stheta1*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) 
        el2_14 = -l5*stheta2*(s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2))
        el3_14 = -l5*ctheta2*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) 
        el4_14 = -l4*ctheta1*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)

        J_14 = el1_14 + el2_14 + el3_14 + el4_14   

        ###Dependancies for J15

        J_15 = (l5*stheta2*c6 - l5*ctheta2*s6)*(c3*c5*s1 + c1*c2*c5*s3 - c1*s2*s4*s5 + c4*s1*s3*s5 - c1*c2*c3*c4*s5)
        
        ###Dependancies for J16

        el1_16 = l5*ctheta2*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) 
        el2_16 = - l5*stheta2*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))
        
        J_16 = el1_16 + el2_16

        ###Dependancies for J16

        J_17 = 0

        ###Dependancies for J21

        el1_21 = l2*c1*s2 - l3*(s1*s3 - c1*c2*c3)
        el2_21 = -l4*stheta1*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) 
        el3_21 = -l4*ctheta1*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) 
        el4_21 = -l5*ctheta2*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) 
        el5_21 = -l5*stheta2*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) 
        
        J_21 = el1_21 + el2_21 + el3_21 + el4_21 + el5_21

        ###Dependancies for J22

        el1_22 = l4*stheta1*(c2*s1*s4 - c3*c4*s1*s2) 
        el2_22 = -l4*ctheta1*(c2*c4*s1 + c3*s1*s2*s4) 
        el3_22 = -l5*ctheta2*(c6*(c2*c4*s1 + c3*s1*s2*s4) + s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5))
        el4_22 = -l5*stheta2*(s6*(c2*c4*s1 + c3*s1*s2*s4) - c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5)) 
        el5_22 = l2*c2*s1 - l3*c3*s1*s2

        J_22 = el1_22 + el2_22 + el3_22 + el4_22 + el5_22

        ###Dependancies for J23

        el1_23 = l3*(c1*c3 - c2*s1*s3) 
        el2_23 = -l5*ctheta2*(s6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) 
        el3_23 = l5*stheta2*(c6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) + s4*s6*(c1*c3 - c2*s1*s3)) 
        el4_23 = l4*stheta1*c4*(c1*c3 - c2*s1*s3) 
        el5_23 = l4*ctheta1*s4*(c1*c3 - c2*s1*s3)

        J_23 = el1_23 + el2_23 + el3_23 + el4_23 + el5_23 

        ###Dependancies for J24

        el1_24 = l5*ctheta2*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) 
        el2_24 = l5*stheta2*(s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) 
        el3_24 = -l4*stheta1*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) 
        el4_24 = l4*ctheta1*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)

        J_24 = el1_24 + el2_24 + el3_24 + el4_24
        
        ###Dependancies for J25

        J_25 = -(l5*stheta2*c6 - l5*ctheta2*s6)*(c1*c3*c5 - c2*c5*s1*s3 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5)
        
        ###Dependancies for J26

        el1_26 = l5*stheta2*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) 
        el2_26 = -l5*ctheta2*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))
        
        J_26 = el1_26 + el2_26

        ###Dependancies for J27

        J_27 = 0

        ###Dependancies for J31

        J_31 = 0

        ###Dependancies for J32

        el1_32 = l5*ctheta2*(s6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) + c6*(c4*s2 - c2*c3*s4)) 
        el2_32 = -l5*stheta2*(c6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) - s6*(c4*s2 - c2*c3*s4)) 
        el3_32 = -l4*stheta1*(s2*s4 + c2*c3*c4)
        el4_32 = l4*ctheta1*(c4*s2 - c2*c3*s4) 
        el5_32 = - l2*s2 - l3*c2*c3

        J_32 = el1_32 + el2_32 + el3_32 + el4_32 + el5_32

        ###Dependancies for J33

        J_33 = s2*(l3*s3 + l4*stheta1*c4*s3 + l4*ctheta1*s3*s4 - l4*ctheta1*c3*c6*s5 + l4*ctheta1*c6*s3*s4 + l4*ctheta1*c3*s5*s6 + l4*ctheta1*s3*s4*s6 + l4*ctheta1*c4*c5*c6*s3 - l4*ctheta1*c4*c5*s3*s6)

        ###Dependancies for J34

        el1_34 = l4*stheta1*(c2*c4 + c3*s2*s4)
        el2_34 = l4*ctheta1*(c2*s4 - c3*c4*s2) 
        el3_34 = l5*ctheta2*(c6*(c2*s4 - c3*c4*s2) - c5*s6*(c2*c4 + c3*s2*s4)) 
        el4_34 = l5*stheta2*(s6*(c2*s4 - c3*c4*s2) + c5*c6*(c2*c4 + c3*s2*s4))

        J_34 = el1_34 + el2_34 + el3_34 + el4_34

        ###Dependancies for J35

        J_35 = -(l5*stheta2*c6 - l5*ctheta2*s6)*(c5*s2*s3 + c2*s4*s5 - c3*c4*s2*s5)

        ###Dependancies for J36

        el1_36 = -l5*ctheta2*(c6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) 
        el2_36 = -l5*stheta2*(s6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))
        
        J_36 = el1_36 + el2_36

        ###Dependancies for J37

        J_37 = 0


        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J


    def tf_A01(self, r_joints_array):

        l1 = self.l1
        q1 = r_joints_array[0]
        c1 = np.cos(q1)
        s1 = np.sin(q1)

        tf = np.matrix([[c1 , -s1 , 0 , 0],\
                        [s1 , c1 , 0 , 0],\
                        [0 , 0 , 1 , l1],\
                        [0 , 0 , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):

        q2 = r_joints_array[1]
        c2 = np.cos(q2)
        s2 = np.sin(q2)

        tf_A12 = np.matrix([[c2 , -s2 , 0 , 0],\
                            [0 , 0 , 1 , 0],\
                            [-s2 , -c2 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):

        l2 = self.l2
        q3 = r_joints_array[2]
        c3 = np.cos(q3)
        s3 = np.sin(q3)

        tf_A23 = np.matrix([[c3 , -s3 , 0 , 0],\
                            [0 , 0 , -1 , -l2],\
                            [s3 , c3 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):

        l3 = self.l3
        q4 = r_joints_array[3]
        c4 = np.cos(q4)
        s4 = np.sin(q4)

        tf_A34 = np.matrix([[c4 , -s4 , 0 , l3],\
                            [0 , 0 , -1 , 0],\
                            [s4 , c4 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):

        l4 = self.l4
        theta1 = self.theta1
        ctheta1 = np.cos(theta1)
        stheta1 = np.sin(theta1)
        q5 = r_joints_array[4]
        c5 = np.cos(q5)
        s5 = np.sin(q5)

        tf_A45 = np.matrix([[c5 , -s5 , 0 , l4*stheta1],\
                            [0 , 0 , -1 , -l4*ctheta1],\
                            [s5 , c5 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):

        q6= r_joints_array[5]
        c6 = np.cos(q6)
        s6 = np.sin(q6)

        tf_A56 = np.matrix([[c6 , -s6 , 0 , 0],\
                            [0 , 0 , -1 , 0],\
                            [s6 , c6 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):

        l5 = self.l5
        theta2 = self.theta2
        ctheta2 = np.cos(theta2)
        stheta2 = np.sin(theta2)
        q7 = r_joints_array[6]
        c7 = np.cos(q7)
        s7 = np.sin(q7)

        tf_A67 = np.matrix([[c7 , -s7 , 0 , l5*stheta2],\
                            [0 , 0 , 1 , l5*ctheta2],\
                            [-s7 , -c7 , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
