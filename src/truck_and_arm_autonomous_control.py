#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# Allows switch of vacuum gripper on or off
from std_srvs.srv import SetBool

from geometry_msgs.msg import PoseStamped
import numpy as np
from tf_transformations import euler_from_quaternion

import sympy as sp
from sympy import *
import math

# test for multiple callbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class AutonomousControl(Node):

    def __init__(self, x_loc, y_loc):
        super().__init__('autonomous_control_node')

        # test multiple callbacks
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        ## Create a publisher node to publish joint positions to the arm
        self.arm_position_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10, callback_group=timer_cb_group)

        # initializing service client
        self.cli = self.create_client(SetBool, '/demo/custom_switch', callback_group=client_cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

        ## Truck Control
        # Subscriber to retrieve localisation information of the Truck "dummy_link" (Topic: /odom)
        self.odom_sub = self.create_subscription(PoseStamped,
                                                '/odom',
                                                self.listener_callback,
                                                10, callback_group=timer_cb_group)
        self.odom_sub # Prevent unused variable warning

        # Publisher to publish velocity data the Truck Rear Wheels(Topic: /velocity_controller/commands)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10, callback_group=timer_cb_group)

        # Publisher to publish position data the Truck Front Joints(Topic: /position_controller/commands)
        self.pos_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10, callback_group=timer_cb_group)

        # Wheel Velocity, Joint Position, and Destination
        self.velocity = 0.0
        self.position = 0.0
        self.destination = [float(x_loc), float(y_loc)]

        # # Proportional Gains
        self.vel_gain = 3.0
        self.pos_gain = 1.0

        # Define Max. velocities and Joint angle position
        self.max_vel = 20.0
        self.max_pos = 0.00

        self.i = 1


        self.initial = 1

        # home position arm
        self.global_t1 = 0.0
        self.global_t2 = 0.0
        self.global_t3 = 0.0
        self.global_t4 = 0.0

    # Function to publish wheel velocity and joint position whenever the subscriber hears from '/odom'
    def vel_pos_publisher(self, vel, pos):
        # Velocity message to publish to the Truck
        vel_msg = Float64MultiArray()
        # Constant velocity v = 15
        vel_msg.data = [-vel, vel] # Rear-left and Rear-right Wheels
        vel_msg.layout.dim = []
        vel_msg.layout.data_offset = 1

        # Position message to publish to the Truck
        pos_msg = Float64MultiArray()
        # Constant angle a = 1.0  Range: {-3.0, 3.0}
        pos_msg.data = [-pos, -pos, float(self.global_t1), float(self.global_t2), float(self.global_t3), float(self.global_t4)] # Front-left and Front-right joints
        pos_msg.layout.dim = []
        pos_msg.layout.data_offset = 1

        # Publish message
        self.vel_pub.publish(vel_msg)
        self.pos_pub.publish(pos_msg)

        self.i += 1

    # Call back to msg retrieved from '/odom' topic
    def listener_callback(self, msg):

        # Obtain the Coordinates (X, Y) of the Truck "dummy_link"
        X = msg.pose.position.x
        Y = msg.pose.position.y

        # Distance of the Truck from the Destination
        dist = np.sqrt((self.destination[0] - X)**2 + (self.destination[1] - Y)**2)

        # Slope between the Truck and the Destination
        if dist == 0.0:
            theta = 0.0
        else:
            theta = np.degrees(np.arcsin((self.destination[1] - Y) / dist))
            if  X > self.destination[0]:
                if Y <= self.destination[1]:
                    theta += 90.0
                else:
                    theta += -90.0
            theta = np.radians(theta)


        # Obtain the Quaternion Orientation (X, Y, Z, W) of the Truck "dummy_link"
        X_quat = msg.pose.orientation.x
        Y_quat = msg.pose.orientation.y
        Z_quat = msg.pose.orientation.z
        W_quat = msg.pose.orientation.w

        # Euler Orientation (in radians) from the Quaternions
        [roll, pitch, yaw] = euler_from_quaternion([X_quat, Y_quat, Z_quat, W_quat])

        ## Define a proportional Controller for Wheel Velocity and Joint Position ##

        # Wheel Velocity
        self.velocity = self.vel_gain * dist
        if self.velocity > 0.0:
            self.velocity = min(self.velocity, self.max_vel)

        # Joint Position
        self.position = self.pos_gain * (yaw - theta) # yaw: orientation of the truck (in radians)
        if self.position > 0.0:
            self.position = min(self.position, self.max_pos)
        elif self.position < 0.0:
            self.position = max(self.position, -1*self.max_pos)

        # Now, publish the velocity and position to the truck
        if dist > 0.2:
            self.vel_pos_publisher(self.velocity, self.position)
            self.get_logger().info(f'Driving towards destination! Publishing[{self.i}] Velocity: {self.velocity}, Joint Position: {self.position}')
        # If destination arrived
        else:
            self.vel_pos_publisher(0.0, 0.0)
            self.get_logger().info(f'Destination Arrived Successfully!!')

            # destroy node if second destination has been reached. 
            if self.destination == [5.3, 0.25]:
                # move arm to apple
                self.publish_arm_position(-13, 0, 14)
                self.destroy_node()
            else:
                # move arm to apple
                self.publish_arm_position(-12, 0, 15)
            # set second arm destination
            self.destination = [5.3, 0.25]


    # send true or false to turn switch on or off
    def send_request(self, status):
        self.req.data = status
        self.future = self.cli.call_async(self.req)
        
        return self.future.result()

    ##---------------------------------------------------------------------------------------------------------------------------------------------------------##

    ### Arm Control ###

    def publish_arm_position(self, x_pos, y_pos, z_pos):
        joint_positions = Float64MultiArray()

        ## Define the joint anlges which are function of time
        θ1, θ2, θ3, θ4, t = sp.symbols('θ1 θ2 θ3 θ4 t')

        ## Home position of end-effector coordinates (in inches)
        x_end = 8.0
        y_end = 0.0
        z_end = 15.15

        ## Define the DH Table (Matrix: [a_i, alpha_i, d_i, theta_i])
        DH = sp.Matrix([[0,  sp.rad(-90), 9.15, θ1],
                        [-6, sp.rad(180), 0,    sp.rad(90)+θ2],
                        [6,  0,           0,    sp.rad(90)+θ3],
                        [2,  0,           0,    θ4]])


        ## Initiate a list to store all the homogeneous transofrmation matrices
        H = []

        for i in range(DH.shape[0]):
            ## Calculate Transformation Matrix at each frame using the DH Table
            T = sp.Matrix([[sp.cos(DH[i, 3]), -sp.sin(DH[i, 3])*sp.cos(DH[i, 1]),sp.sin(DH[i, 3])*sp.sin(DH[i, 1]), sp.cos(DH[i, 3])*DH[i, 0]],
                            [sp.sin(DH[i, 3]), sp.cos(DH[i, 3])*sp.cos(DH[i, 1]), -sp.cos(DH[i, 3])*sp.sin(DH[i, 1]), sp.sin(DH[i, 3])*DH[i, 0]],
                            [0, sp.sin(DH[i, 1]), sp.cos(DH[i, 1]), DH[i, 2]],
                            [0, 0, 0, 1]])

            ## Store the matrix in 'H'
            H.append(T)

        ## Initiate Identity matrix to start the matrix multiplication to obtain final Homo. Trans. Mat.
        Final = identity_matrix = np.identity(4, dtype="int")

        ## Inittiate list to store Trans. Matrices for frame 'n' w.r.t frame '0', n=1,2,3,4..
        T0_n = []

        for i in range(DH.shape[0]):
            Final = Final*H[i]
            T0_n.append(Final)

        ## End-effector position coordinates (x, y, z) wrt base frame
        p = []
        p.append(Final[0, 3])
        p.append(Final[1, 3])
        p.append(Final[2, 3])

        ## Now, compute the LINEAR Velocity Jacobian Matrix (Jv) using method 2 (using z-components and partial derivatives): First 3 rows (Upper-half) of Overall Jacobian Matrix (J)
        # j'th column of Jv = d(O_0n)/dqj, for revolute joint qj = theta_j and O_0n => Position of enf-effector(n) w.r.t base frame(0)
        # Jv[i][j] = d(O_0n[i])/dθj
        Jv = sp.Matrix([[sp.diff(p[0], θ1), sp.diff(p[0], θ2), sp.diff(p[0], θ3),sp.diff(p[0], θ4)],
                            [sp.diff(p[1], θ1), sp.diff(p[1], θ2), sp.diff(p[1], θ3),sp.diff(p[1], θ4)],
                            [sp.diff(p[2], θ1), sp.diff(p[2], θ2), sp.diff(p[2], θ3),sp.diff(p[2], θ4)]])


        ## Obtain target coordinates (x_target, y_target, z_target) w.r.t arm base frame (0) and traverse end-effector to target location
        x_target = float(x_pos)
        y_target = float(y_pos)
        z_target = float(z_pos)

        # Check if target location is reachable:
        if (x_target)**2 + (y_target)**2 + (z_target-9.15)**2 <= 14**2:
            self.get_logger().info(f"Target Coordinates: ({x_target}, {y_target}, {z_target}) is Reachable!!")
        else:
            self.get_logger().info("Target location not reachable!!")
            return 0

        ## Initial Joint variable values
        t1= self.global_t1
        t2= self.global_t2
        t3= self.global_t3
        t4= self.global_t4
    

        t=0
        dt = 0.1

        ## Total time extimated to complete the trajectory (Less than 20 seconds)
        T = 5

        ## Initiate boolean to denote successful target reaching
        target_reached = False
        home_reached = False
        basket_reached = False
        final_reached = False

        ### Home Position to Target Position

        ## loop for itereating over the time elapsed
        while True:

            ## Substitute angle values into the Jacobian (Jv)
            Jv_subs = Jv.subs([(θ1,t1),(θ2,t2),(θ3,t3),(θ4,t4)])

            ## Since Jv is (3*4), not n*n, Jv Inverse can't be directly computed from Jv inversion
            ## Have to compute the 'Right Pseudoinverse' of Jv

            ## Check if (Jv*Jv_T) Inverse exists
            # Condition: (Jv*Jv_T) Inverse exists for Jv (m*n), if m < n and rank Jv = m
            # Fomr the previous cell, we already know that Rank of Jv (3*4) = 3
            # Therfore, (Jv*Jv_T) Inverse exists

            ## Compute Right Pseudoinverse of Jv_subs
            # J_plus = J_T(JJ_T)-1
            Jv_T = Jv_subs.T
            Jv_right_pseudo = Jv_T * (Jv_subs * Jv_T).inv()

            ## Therefore, q_dot = Jv_right_pseudo * X_dot, where X_dot = (x_dot, y_dot, z_dot), NOTE: w_x, w_y, w_z not considered in this case


            ## Now, Compute Velocity Trajectory (X_dot)
            ## Line equation to connect the home-position of end-effector and target coordinates

            ### Home Position to Target Position ###
            if target_reached == False:
                X_dot = Matrix([[(x_target - x_end)/T],
                                [(y_target - y_end)/T],
                                [(z_target - z_end)/T]])

            ### Target Position to Home Position ###
            if (target_reached == True) and (home_reached == False):
                X_dot = Matrix([[(8.0 - x_end)/T],
                            [(0.0 - y_end)/T],
                            [(15.15 - z_end)/T]])

            ### Home Position to Basket Position ###
            if (home_reached == True) and (basket_reached == False):
                X_dot = Matrix([[-((16/20)*math.pi) * sin((((2*math.pi)/20)*t))],
                                [0],
                                [((16/20)*math.pi) * cos(((2*math.pi)/20)*t)]])

            ### Basket Position to final Home Position ###
            if (basket_reached == True) and (final_reached == False):
                X_dot = Matrix([[((16/20)*math.pi) * sin((((2*math.pi)/20)*t))],
                                [0],
                                [((16/20)*math.pi) * cos(((2*math.pi)/20)*t)]])

            # self.get_logger().info(f"End-effector velocity: ({(x_target - x_end)/T}, {(y_target - y_end)/T}, {(z_target - z_end)/T})")

            ## Compute Joint Velocities
            q_dot = Jv_right_pseudo * X_dot

            ## Extract the joint velocities
            q1_dot = q_dot[0,0]
            q2_dot = q_dot[1,0]
            q3_dot = q_dot[2,0]
            q4_dot = q_dot[3,0]

            ####### home position

            ## Compute Joint Angles
            q1 = t1 + (q1_dot * dt)
            q2 = t2 + (q2_dot * dt)
            q3 = t3 + (q3_dot * dt)
            q4 = t4 + (q4_dot * dt)

            ## Substitude the new joint angles in the Final Transformation Matrix
            T_end = Final.subs([(θ1,q1),(θ2,q2),(θ3,q3),(θ4,q4)])

            ## Extracting Position values of the end effector wrt base
            x = float(T_end[0,3])
            y = float(T_end[1,3])
            z = float(T_end[2,3])

            # ## Confine joint angles to it's limits
            # if ((q1 >= -3.14) and (q1 <= 3.14)) and ((q2 >= -1.57) and (q2 <= 1.57)) and ((q3 >= -1.0) and (q3 <= 4.57)) and ((q4 >= -2.5) and (q1 <= 2.5)):

            ## Update values for next iteration
            t1 = q1
            t2 = q2
            t3 = q3
            t4 = q4

            self.global_t1 = t1
            self.global_t2 = t2
            self.global_t3 = t3
            self.global_t4 = t4

            # Publish the joint message
            # self.get_logger().info(f"{float(t1)}, {float(t2)}, {float(t3)}, {float(t4)}")
            joint_positions.data = [0.0, 0.0, float(t1), float(t2), float(t3), float(t4)]
            self.arm_position_publisher.publish(joint_positions)

            ## Update time (t)
            t = t + dt

            ## Update End-effector position
            x_end = x
            y_end = y
            z_end = z
            self.get_logger().info(f"End-effector posiotion w.r.t base frame(0): ({round(x_end, 2)}, {round(y_end, 2)}, {round(z_end, 2)})")

            ## Check if Target Reached!!
            if target_reached == False:
                if (round(x, 1) == round(x_target, 1)) and (round(y, 1) == round(y_target, 1)) and (round(z, 1) == round(z_target, 1)):
                    self.get_logger().info("Target Reached Successfully!")
                    ## Set target_reached
                    target_reached = True
                    # turn on vacuum griper
                    self.send_request(True)

                    ## Update time for next execution
                    t = 0

                    self.get_logger().info("Returning back to home position")


            ## Check if Home Position Reached!!
            if (target_reached == True) and (home_reached == False):
                if [round(x_end), round(y_end), round(z_end)] == [8, 0, 15]:
                    self.get_logger().info("Home Position Reached Successfully!")
                    ## Set home_reached
                    home_reached = True

                    ## Update time for next execution
                    t = 0

                    self.get_logger().info("Driving arm to basket")

            # Check if Basket Position Reached!!
            if (home_reached == True) and (basket_reached == False):
                if [round(x_end), round(y_end), round(z_end)] == [-6, 0, round(9.95)]:
                    self.get_logger().info("Home Position Reached Successfully!")
                    ## Set basket_reached
                    basket_reached = True
                    # turn off vacuum griper
                    self.send_request(False)

                    ## Update time for next execution
                    t = 0

                    self.get_logger().info("Driving arm to back to Home Position")

            ## Check if Final Home Position Reached!!
            if (basket_reached == True) and (final_reached == False):
                if [round(x_end), round(y_end), round(z_end)] == [8, 0, 15]:
                    self.get_logger().info("Home Position Reached Successfully!")
                    ## Set final_reached
                    final_reached = True

                    ## Update time for next execution
                    t = 0

                    self.get_logger().info("Final Position Reached!!!")

            ### Break loop if final home position reached
            if final_reached == True:
                break

    

def main(args=None):
    rclpy.init(args=args)

    x_loc = 3.25
    y_loc = 0.25

    # test multiple callbacks
    node = AutonomousControl(x_loc, y_loc)
    rclpy.spin(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()