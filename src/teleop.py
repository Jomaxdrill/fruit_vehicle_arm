#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Control its arm!

        Keys arm: 1,5 link1
        Keys arm: 2,6 link2
        Keys arm: 3,7 link3
        Keys arm: 4,8 link4

        Esc to quit

        """


        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0

        # angles for manipulator arm link
        arm_1_angle = 0.0
        arm_2_angle = 0.0
        arm_3_angle = 0.0
        arm_4_angle = 0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                    arm_1_angle = 0.0
                    arm_2_angle = 0.0
                    arm_3_angle = 0.0
                    arm_4_angle = 0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE
                elif key == '1':    # arm_link1 increase
                    arm_1_angle += ANG_VEL_STEP_SIZE
                elif key == '5':    # arm_link1 decrease
                    arm_1_angle -= ANG_VEL_STEP_SIZE
                elif key == '2':    # arm_link2 increase
                    arm_2_angle += ANG_VEL_STEP_SIZE
                elif key == '6':    # arm_link2 decrease
                    arm_2_angle -= ANG_VEL_STEP_SIZE
                elif key == '3':    # arm_link3 increase
                    arm_3_angle += ANG_VEL_STEP_SIZE
                elif key == '7':    # arm_link3 decrease
                    arm_3_angle -= ANG_VEL_STEP_SIZE
                elif key == '4':    # arm_link4 increase
                    arm_4_angle += ANG_VEL_STEP_SIZE
                elif key == '8':    # arm_link4 decrease
                    arm_4_angle -= ANG_VEL_STEP_SIZE


                if steer_angle>1.0:
                        steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                print("Steer Angle",steer_angle)
                print("Linear Velocity",linear_vel)
                print("arm_1_angle", arm_1_angle)
                print("arm_2_angle", arm_2_angle)
                print("arm_3_angle", arm_3_angle)
                print("arm_4_angle", arm_4_angle)


                # Publish the twist message
                wheel_velocities.data = [-linear_vel,linear_vel]
                joint_positions.data = [-steer_angle,-steer_angle, arm_1_angle, arm_2_angle, arm_3_angle, arm_4_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()