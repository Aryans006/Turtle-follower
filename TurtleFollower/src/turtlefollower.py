#!/usr/bin/env python3
import rclpy
import math 
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class MyNode(Node):
    def __init__(self):
        super().__init__('TurtleFollower')

#calling the spawn service to spawn the second turtle    

        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn not available')
        print("SPAWNING")
        request = Spawn.Request()
        request.x = 3.5
        request.y = 3.5
        request.theta = 0.0
        request.name = "Follower"
        self.future = self.cli.call_async(request)
        
#setting up pubs and subs

        self.publisher_turtle= self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_follower= self.create_publisher(Twist, '/Follower/cmd_vel', 10)
        self.subscription_turtle = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscription_follower = self.create_subscription(Pose, '/Follower/pose', self.pose2_callback, 10)

#other general intialization
        self.current_posex = 0.0
        self.current_posey = 0.0
        self.count = 0        
        self.radius = 0.2
        self.kp = 0.5

            
    def pose2_callback(self, msg):

        self.current_posex = msg.x
        self.current_posey = msg.y

    def pose_callback(self, msg):
        
#each point on the circumference of the circle can have a radius vector pointing to the centre, which can be decomposed into x and y components
#which would be Rcos(theta) and Rsin(theta)
#using a counter to keep track of the iterations, incrementing the angle at every iteration

        if self.count < 361:
                x_vel =  self.radius * math.sin(math.radians(self.count))
                y_vel = self.radius * math.cos(math.radians(self.count))
                self.count = self.count + 1 
                print (self.count)
                
                vel = Twist()
                vel.linear.x = x_vel
                vel.linear.y = y_vel
                self.publisher_turtle.publish(vel)
                time.sleep(0.1)

#using a p controller for following 
        posex = msg.x
        posey = msg.y

#calculating error (position of turtle1 - position of follower turtle)
        errorx = (posex - self.current_posex) * self.kp
        errory = (posey - self.current_posey) * self.kp
        
        # print("YAY", errorx)
        # print("YOOOOY", errory)
        
#adding error to fixed velocity to calculate final velocity
        vel2 = Twist()
        vel2.linear.x = 0.01 + errorx
        vel2.linear.y = 0.01 + errory
        self.publisher_follower.publish(vel2)
                


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
