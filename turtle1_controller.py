#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
import math

class Turtle1ControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle1_controller")
        self.cmd_vel_publisher1_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber2_ = self.create_subscription(
            Pose, "/turtle2/pose", self.pose_callback2, 10)
        self.pose_subscriber1_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback1, 10)
        self.get_logger().info("Turtle1 controller has been started.")
        self.pose1_ = None
        self.pose2_ = None
        self.turtle2_spawned_ = True

    def pose_callback1(self, pose: Pose):
        self.pose1_ = pose
        self.control_turtle1()

    def pose_callback2(self, pose: Pose):
        self.pose2_ = pose
        self.control_turtle1()

    def control_turtle1(self):
        if self.pose1_ is None or self.pose2_ is None:
            return
        cmd = Twist()
        target_angle = math.atan2(self.pose2_.y - self.pose1_.y, self.pose2_.x - self.pose1_.x)
        if not self.turtle2_spawned_:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        if(target_angle >= self.pose1_.theta) and self.turtle2_spawned_:
            angle_diff = target_angle - self.pose1_.theta
            if angle_diff > 0.0348888:
                cmd.linear.x = 1.0
                cmd.angular.z = 5.0 
            else:
                cmd.linear.x = 10.0
                cmd.angular.z = 0.0
        elif(target_angle < self.pose1_.theta) and self.turtle2_spawned_:
            angle_diff = self.pose1_.theta - target_angle
            if angle_diff > 0.0348888:
                cmd.linear.x = 1.0
                cmd.angular.z = -5.0 
            else:
                cmd.linear.x = 10.0
                cmd.angular.z = 0.0

        self.cmd_vel_publisher1_.publish(cmd)

        if self.is_close(self.pose1_, self.pose2_) and self.turtle2_spawned_:
            self.get_logger().info(
                "Turtle1 reached Turtle2. Killing and respawning Turtle2.")
            self.turtle2_spawned_ = False
            self.kill_and_respawn_turtle2()
    
    def kill_and_respawn_turtle2(self):
        client_kill = self.create_client(Kill, '/kill')
        while not client_kill.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service...")
        kill_request = Kill.Request()
        kill_request.name = "turtle2"

        kill_future = client_kill.call_async(kill_request)
        kill_future.add_done_callback(partial(self.callback_kill))

    def callback_kill(self, kill_future):
        try:
            kill_future.result()
            self.get_logger().info(
                "Successfully killed Turtle2. Spawning at new location.")
            self.spawn_turtle2()

        except Exception as e:
            self.get_logger().error("Service kill call failed: %r" % (e,))

    def spawn_turtle2(self):
        client_spawn = self.create_client(Spawn, '/spawn')
        while not client_spawn.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn service...")
        spawn_request = Spawn.Request()
        new_x = random.uniform(0, 11)
        new_y = random.uniform(0, 11)
        spawn_request.x = new_x
        spawn_request.y = new_y
        spawn_request.theta = 0.0
        spawn_request.name = "turtle2"

        spawn_future = client_spawn.call_async(spawn_request)
        spawn_future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self, spawn_future):
        try:
            spawn_future.result()
            self.get_logger().info("Successfully spawned Turtle2.")
            self.turtle2_spawned_ = True
        except Exception as e:
            self.get_logger().error("Service spawn call failed: %r" % (e,))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def is_close(self, pose1, pose2, threshold=0.5):
        distance = math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)
        return distance < threshold
    
def main(args=None):
    rclpy.init(args=args)
    node = Turtle1ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
