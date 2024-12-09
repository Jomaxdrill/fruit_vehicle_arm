#!/usr/bin/env python3
### Project 1: Subscriber ###

##-----------------------------------------------------------------------------##
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np

class Subscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')

        # Subscriber to retrieve localisation information of the Truck "dummy_link" (Topic: /odom)
        self.odom_sub = self.create_subscription(PoseStamped,
                                                '/odom',
                                                self.listener_callback,
                                                10)

        self.i = 1

        # Destination
        self.destination = [10.0, 10.0]

    # Call back to msg retrieved from '/odom' topic 
    def listener_callback(self, msg):
        # Obtain the Coordinates (X, Y) of the Truck "dummy_link"
        X = msg.pose.position.x
        Y = msg.pose.position.y

        # Distance of the Truck from the Destination
        dist = np.sqrt((self.destination[0] - X)**2 + (self.destination[1] - Y)**2)
        
        # Now, publish the velocity and position to the truck
        if dist > 0.1:
            self.get_logger().info(f'Subscription[{self.i}] Truck Location X: {X}, Y: {Y}')
        # If destination arrived
        else:
            self.get_logger().info(f'Destination Arrived Successfully!!')

        self.i += 1
        

def main():
    rclpy.init()

    odom_subscriber = Subscriber()

    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
##-----------------------------------------------------------------------------##