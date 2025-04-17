import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odometry')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot params
        self.TICKS_PER_REV = 500
        self.GEAR_RATIO = 2.5
        self.TRACK_RADIUS = 0.065
        self.TRACK_WIDTH = 0.4

        self.TICKS_PER_METER = (self.TICKS_PER_REV * self.GEAR_RATIO) / (2 * math.pi * self.TRACK_RADIUS)

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Simulate encoder reading (replace with real data later)
        self.create_timer(0.05, self.update_odometry)

    def get_encoder_ticks(self):
        # REPLACE THIS FUNCTION with your real encoder interface
        # Return current_left_ticks, current_right_ticks
        # These should be cumulative (like a counter)
        return self.last_left_ticks + 2, self.last_right_ticks + 2  # Example: 2 ticks per update

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        left_ticks, right_ticks = self.get_encoder_ticks()

        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Convert to distance
        d_left = delta_left / self.TICKS_PER_METER
        d_right = delta_right / self.TICKS_PER_METER
        d_center = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.TRACK_WIDTH

        # Update pose
        self.x += d_center * math.cos(self.theta + delta_theta / 2)
        self.y += d_center * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta

        # Normalize theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Prepare messages
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
