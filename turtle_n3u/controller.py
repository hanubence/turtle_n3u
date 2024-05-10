import math
import sys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
import argparse


class PTController(Node):

    def __init__(self):
        super().__init__("turtlesim_controller")
        self.twist_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.cb_pose, 10
        )

        self.Kp_distance = 20  # Proportional gain of distance
        self.distance_threshold = 0.01

        self.Kp_angle = 10  # Proportional gain of angle
        self.angle_threshold = 0.001

    def cb_pose(self, msg):
        self.pose = msg
        self.get_logger().info(
            f"POSE: ({round(self.pose.x, 2)}; {round(self.pose.y, 2)}, theta={round(math.degrees(self.pose.theta), 4)}°)"
        )

    def move_forward(self, distance):
        if self.pose:

            # get the end points of the line
            end_x = self.pose.x + distance * math.cos(self.pose.theta)
            end_y = self.pose.y + distance * math.sin(self.pose.theta)

            # set loop rate
            loop_rate = self.create_rate(100)

            # create new Twist message
            vel_msg = Twist()
            while rclpy.ok():

                # calculate the error distance
                dx = end_x - self.pose.x
                dy = end_y - self.pose.y
                err_distance = math.sqrt(dx**2 + dy**2)

                # if the error distance is less than the threshold, break the loop
                if err_distance < self.distance_threshold:
                    break

                # cap the speed to 1.0
                speed = min(self.Kp_distance * err_distance, 1.0)

                # create and publish Twist message
                vel_msg.linear.x = speed if err_distance > 0 else -speed
                self.twist_pub.publish(vel_msg)

                rclpy.spin_once(self)

            # stop once the loop is broken
            vel_msg.linear.x = 0.0
            self.twist_pub.publish(vel_msg)

    def rotate(self, angle):
        if self.pose:

            # convert the angle to radians
            angle = math.radians(angle)

            # calculate the target angle
            target_angle = self.pose.theta + angle
            target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

            # calculate the error angle
            error_angle = target_angle - self.pose.theta
            error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

            # set loop rate
            loop_rate = self.create_rate(100)

            while rclpy.ok():
                # calculate the error angle
                error_angle = target_angle - self.pose.theta
                error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

                # break the loop if the error angle is less than the threshold
                if abs(error_angle) < self.angle_threshold:
                    break

                # calculate the angular speed and account for the direction
                angular_speed = self.Kp_angle * abs(error_angle)
                if error_angle < 0:
                    angular_speed = -angular_speed

                # create and publish Twist message
                vel_msg = Twist()
                vel_msg.angular.z = angular_speed
                self.twist_pub.publish(vel_msg)

                rclpy.spin_once(self)

            # stop once the loop is broken
            vel_msg.angular.z = 0.0
            self.twist_pub.publish(vel_msg)

    def edge(self, length, depth):
        if depth > 0:
            self.edge(length / 3, depth - 1)
            self.rotate(-60)
            self.edge(length / 3, depth - 1)
            self.rotate(120)
            self.edge(length / 3, depth - 1)
            self.rotate(-60)
            self.edge(length / 3, depth - 1)
        else:
            self.move_forward(length)

    def snowflake(self, length, depth):
        for i in range(3):
            self.edge(length, depth)
            self.rotate(120)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--triangle", action="store_true", help="Boolean CLI argument")
    args = parser.parse_args()

    controller = PTController()
    while controller.pose is None:
        rclpy.spin_once(controller)

    if args.triangle:
        controller.move_forward(3)
        controller.rotate(120)
        controller.move_forward(3)
        controller.rotate(120)
        controller.move_forward(3)
        controller.rotate(120)
    else:
        controller.snowflake(3, 3)

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
