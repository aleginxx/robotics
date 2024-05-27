#!/usr/bin/env python3
# Robotics II - Project 2A - Team26 - Iliopoulos - Serlis
"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
# Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self, rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        # Publishing rate
        self.period = 1.0 / rate
        self.pub_rate = rospy.Rate(rate)

        # Publishers to plot
        self.r_error = rospy.Publisher("/R_sonar_error", Float64, queue_size=100)
        self.fr_error = rospy.Publisher("/FR_sonar_error", Float64, queue_size=100)
        self.linear_velocity_x = rospy.Publisher("/linear_velocity_x", Float64, queue_size=100)
        self.angular_velocity_z = rospy.Publisher("/angular_velocity_z", Float64, queue_size=100)

        # Open the output file and write headers
        self.output_file = open('output.txt', 'w')
        self.output_file.write("Time, Sonar_F, Linear_Velocity_X, Angular_Velocity_Z, Front_Right_Error, Right_Error\n")

        self.publish()

    # SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states
        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu
        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frame is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frame is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frame is stored in :: self.imu.linear_acceleration)

        # quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # (roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F
        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL
        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR
        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L
        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R
        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        # Gains of PD Controller
        Kp = 30
        Kd = 10

        # Define desired distances from walls
        front_right_margin = 0.27  
        right_margin = 0.25  
        front_right_error = 0.0
        right_error = 0.0
        proportional_error = 0.0
        derivative_error = 0.0
        
        phase = 0  # steps of algorithm

        while not rospy.is_shutdown():

            if phase == 0:
                # Set velocities to move towards wall
                self.velocity.linear.x = 0.6
                self.velocity.angular.z = 0.0

                # Came too close to wall - need to turn
                if self.sonar_F.range < 0.4:
                    phase = 1

            elif phase == 1:
                # Set velocities to turn counterclockwise
                self.velocity.linear.x = 0.025
                self.velocity.angular.z = -np.pi  # Negative for counterclockwise

                # Turned enough -> need to start PD controller
                if (self.sonar_R.range + 0.25 < self.sonar_F.range) and (self.sonar_FR.range + 0.25 < self.sonar_F.range):
                    # front_right_error = 0
                    # right_error = 0
                    phase = 2

            elif phase == 2:
                # Proportional Errors
                front_right_error = front_right_margin - self.sonar_FR.range
                right_error = right_margin - self.sonar_R.range

                proportional_error = front_right_error + right_error

                # Derivative Errors
                derivative_front_right = front_right_error - previous_P_FR_error
                derivative_right = right_error - previous_P_R_error

                derivative_error = (derivative_front_right + derivative_right) / dt

                # Set velocities for wall following
                self.velocity.linear.x = 0.6  # constant
                self.velocity.angular.z = -(Kp * proportional_error + Kd * derivative_error)  # Negative for counterclockwise

                # Front-end came too close - need to turn counterclockwise
                if (self.sonar_R.range + 0.25 >= self.sonar_F.range) or (self.sonar_FR.range + 0.25 >= self.sonar_F.range):
                    phase = 1

            if phase == 2:
                # Update error values for next iteration -> to find derivative errors
                previous_P_FR_error = front_right_error
                previous_P_R_error = right_error

            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev) / 1e9 + 10 ** (-12)  # problem with zero division!

            # Log data to the output file
            self.output_file.write(f"{rostime_now.to_sec()}, {self.sonar_F.range}, {self.velocity.linear.x}, {self.velocity.angular.z}, {front_right_error}, {right_error}\n")

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            # Publish values to be plotted
            self.fr_error.publish(abs(front_right_margin - self.sonar_FR.range))
            self.r_error.publish(abs(right_margin - self.sonar_R.range))
            self.linear_velocity_x.publish(self.velocity.linear.x)
            self.angular_velocity_z.publish(self.velocity.angular.z)

            self.pub_rate.sleep()

    def turn_off(self):
        self.output_file.close()
        print("Output file closed.")

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
