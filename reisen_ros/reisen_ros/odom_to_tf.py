import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from math import cos, sin,pi
from geometry_msgs.msg import TransformStamped
import tf2_ros

class Odom_to_tf(Node):
    def __init__(self):
        node_name = "odom_to_tf"
        super().__init__(node_name)
        self.declare_parameter("d_time",'0.05')
        self.subscription = self.create_subscription(
            Twist,
            'diff_odom',
            self.odom_to_tf,
            10)
        self.current_pose = [0.0,0.0,0.0]
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def odom_to_tf(self,msg):
        d_time = float(self.get_parameter("d_time").get_parameter_value().string_value)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        self.current_pose[0] += linear_vel * cos(self.current_pose[2]) * d_time
        self.current_pose[1] += linear_vel * sin(self.current_pose[2]) * d_time
        self.current_pose[2] += angular_vel * d_time
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = self.current_pose[0]
        transform_stamped.transform.translation.y = self.current_pose[1]
        transform_stamped.transform.rotation.w = cos(self.current_pose[2] / 2)
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = sin(self.current_pose[2] / 2)

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    pass
if __name__ == '__main__':
    main()
