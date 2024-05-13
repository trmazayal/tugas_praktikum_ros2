import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Point, Twist
import numpy as np


class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscriber_goal_point = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/hokuyo',
            self.listener_scan,
            10)

        self.timer = self.create_timer(0.1, self.navigate)

        self.robot_pose = Pose2D()
        self.robot_pose_received = False

        self.goal_point = Point()
        self.goal_point_received = False

        self.laser_scan = LaserScan()
        self.laser_scan_received = False

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True

    def goal_point_callback(self, msg):
        self.goal_point = msg
        self.goal_point_received = True

    def listener_scan(self, msg):
        self.laser_scan = msg
        self.laser_scan_received = True

    def navigate(self):
        short_distance = float('inf')
        predicted_goal_angle = 0
        x_mod = 0
        y_mod = 0
        print(f"Robot pose theta: {self.robot_pose.theta}")
        if self.laser_scan_received:
            distances = np.array(self.laser_scan.ranges)
            shortest = distances.argmin()
            angles = np.linspace(self.laser_scan.angle_min, self.laser_scan.angle_max, len(self.laser_scan.ranges))
            short_angle = angles[shortest]
            short_distance = distances[shortest]
            print(f"Closest distance: {short_distance}")
            predicted_goal_angle = (self.robot_pose.theta + short_angle) % 6.28
            if predicted_goal_angle > 3.14:
                predicted_goal_angle += -6.28

        dx = self.goal_point.x - self.robot_pose.x + x_mod
        dy = self.goal_point.y - self.robot_pose.y + y_mod
        distance = np.sqrt(dx ** 2 + dy ** 2)
        goal_angle = np.arctan2(dy, dx)

        avoid_distance = 0.33
        very_close = False
        if short_distance < avoid_distance:
            distance = avoid_distance - short_distance
            goal_angle = predicted_goal_angle
            print(f"Predicted goal angle: {predicted_goal_angle}")
            very_close = True

        theta = goal_angle - self.robot_pose.theta

        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi

        cmd_vel = Twist()

        if distance > 0.1 and not very_close:
            cmd_vel.linear.y = np.min([0.2 * distance, 0.2])
            cmd_vel.angular.z = 2.0 * theta
        elif very_close:
            cmd_vel.linear.y = -0.2
            cmd_vel.angular.z = 2.0 * theta
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        self.publisher_cmd_vel.publish(cmd_vel)

        return True


def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()