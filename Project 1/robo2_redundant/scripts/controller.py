#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        self.angpos_data = []
        self.angvel_data = []
        self.end_effector_position = []
        self.outputFile = 'output.txt'

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg

        self.angpos_data.append(msg.position)
        self.angvel_data.append(msg.velocity)
        end_effector_position_str = ', '.join(map(str, self.A07[:3,3]))
        self.end_effector_position.append(self.A07[:3,3])

        with open(self.outputFile, 'a') as f:
                #Write joint angles, velocities and end effector position to the file
                f.write(f"Joint Angles: {msg.position}\n")
                f.write(f"Joint Velocities: {msg.velocity}\n")
                f.write(f"End Effector Position: [{end_effector_position_str}]\n")
                f.write("\n")

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        # set configuration
        # total pitch: j2-j4+j6+pi (upwards: 0rad)
        j2 = 0.7 ; j4 = np.pi/2
        j6 = - (j2-j4)
        self.joint_angpos = [0, j2, 0, j4, 0, j6, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        A07 = self.kinematics.tf_A07(self.joint_angpos)
        print("The End Effector Coordinates are: ", A07[:3, 3])

        #Trajectory definition

        T = 2
        steps = 3001
        Time = np.linspace(0, 5, steps)

        # #Coefficients

        px = np.ones(steps)*0.6127
        py = 0.2*np.sin(2*pi/T*Time)
        pz = np.ones(steps)*0.199

        vx = np.zeros(steps)
        vy = 0.2*pi/2*np.cos(2*pi/T*Time)
        vz = np.zeros(steps)

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        while not rospy.is_shutdown():

            for i in range(steps):

                # Compute each transformation matrix wrt the base frame from joints' angular positions
                self.A01 = self.kinematics.tf_A01(self.joint_angpos)
                self.A02 = self.kinematics.tf_A02(self.joint_angpos)
                self.A03 = self.kinematics.tf_A03(self.joint_angpos)
                self.A04 = self.kinematics.tf_A04(self.joint_angpos)
                self.A05 = self.kinematics.tf_A05(self.joint_angpos)
                self.A06 = self.kinematics.tf_A06(self.joint_angpos)
                self.A07 = self.kinematics.tf_A07(self.joint_angpos)

                # Compute jacobian matrix
                J = self.kinematics.compute_jacobian(self.joint_angpos)
                # pseudoinverse jacobian
                pinvJ = pinv(J)

                #Task 1 - Calculate joint speeds

                #for end effector
                desired_velocity = np.matrix([[vx[i]],[vy[i]], [vz[i]]])
                desired_position = np.matrix([[px[i]], [py[i]], [pz[i]]])
                real_position = self.A07[:3, 3]

                K = 100

                q1_dot = pinvJ @ (desired_velocity + K*(desired_position - real_position))

                #Task 2 - Avoid obstacles

                greenObstacle = self.model_states.pose[1].position.y
                redObstacle = self.model_states.pose[2].position.y

                center = (greenObstacle + redObstacle)/2

                q1 = self.joint_angpos[0]
                q2 = self.joint_angpos[1]
                q3 = self.joint_angpos[2]
                q4 = self.joint_angpos[3]

                l2 = self.kinematics.l2
                l3 = self.kinematics.l3
                l4 = self.kinematics.l4

                theta1 = self.kinematics.theta1

                c1 = np.cos(q1)
                c2 = np.cos(q2)
                c3 = np.cos(q3)
                c4 = np.cos(q4)

                s1 = np.sin(q1)
                s2 = np.sin(q2)
                s3 = np.sin(q3)
                s4 = np.sin(q4)

                ctheta1 = np.cos(theta1)
                stheta1 = np.sin(theta1)

                Kc = 250

                dist3 = np.zeros((7,1))
                dist3[0] = -Kc*(self.A03[1,3] - center)*l2*s2*c1
                dist3[1] = -Kc*(self.A03[1,3] - center)*l2*s1*c2

                dist4 = np.zeros((7,1))
                dist4[0] = -Kc*(self.A04[1,3] - center)*l2*s2*c1 + l3*(-s1*s3 + c1*c2*c3)
                dist4[1] = -Kc*(self.A04[1,3] - center)*(l2*s1*c2 - l3*s1*s2*c3)
                dist4[2] = -Kc*(self.A04[1,3] - center)*(l3*(-s1*s3*c2 + c1*c3))

                dist5 = np.zeros((7,1))


                dist5[0] = -Kc*(self.A05[1,3] - center)*(l2*s2*c1 + l3*(-s1*s3 + c1*c2*c3) + l4*((-s1*s3 + c1*c2*c3)*c4 + s2*s4*c1)*stheta1 - l4*((s1*s3 - c1*c2*c3)*s4 + s2*c1*c4)*ctheta1)
                dist5[1] = -Kc*(self.A05[1,3] - center)*(l2*s1*c2 - l3*s1*s2*c3 - l4*(s1*s2*s4*c3 + s1*c2*c4)*ctheta1 + l4*(-s1*s2*c3*c4 + s1*s4*c2)*stheta1)
                dist5[2] = -Kc*(self.A05[1,3] - center)*(l3*(-s1*s3*c2 + c1*c3) + l4*(-s1*s3*c2 + c1*c3)*stheta1*c4 - l4*(s1*s3*c2 - c1*c3)*s4*ctheta1)
                dist5[3] = -Kc*(self.A05[1,3] - center)*(-l4*((-s1*c2*c3 - s3*c1)*c4 - s1*s2*s4)*ctheta1 + l4*(-(s1*c2*c3 + s3*c1)*s4 + s1*s2*c4)*stheta1)

                weights = np.array([15,30,15])

                dist = weights[0]*dist3 + weights[1]*dist4 + weights[2]*dist5

                q2_dot = (np.eye(7) - np.dot(pinvJ, J)) @ dist

                for i in range(7):
                    self.joint_angvel[i] = q1_dot[i,0] + q2_dot[i,0] #Velocity from both tasks
                
                # Append data to lists
                self.angpos_data.append(self.joint_states.position)
                self.angvel_data.append(self.joint_states.velocity)
                self.end_effector_position.append(self.A07[:3,3])

                # Convertion to angular position after integrating the angular speed in time
                # Calculate time interval
                time_prev = time_now
                rostime_now = rospy.get_rostime()
                time_now = rostime_now.to_nsec()
                dt = (time_now - time_prev)/1e9
                # Integration
                self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )

                # Publish the new joint's angular positions
                self.joint1_pos_pub.publish(self.joint_angpos[0])
                self.joint2_pos_pub.publish(self.joint_angpos[1])
                self.joint3_pos_pub.publish(self.joint_angpos[2])
                self.joint4_pos_pub.publish(self.joint_angpos[3])
                self.joint5_pos_pub.publish(self.joint_angpos[4])
                self.joint6_pos_pub.publish(self.joint_angpos[5])
                self.joint7_pos_pub.publish(self.joint_angpos[6])

                self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
