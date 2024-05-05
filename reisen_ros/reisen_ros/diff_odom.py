import rclpy
from rclpy.node import Node
from reisen_msgs.msg import Wheel
from geometry_msgs.msg import Twist, Vector3
from math import cos, sin,pi
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomCalculator(Node):
    def __init__(self):
        super().__init__('odom_calculator')
        self.declare_parameter("car_width",'0.253')
        self.declare_parameter("wheel_radius",'0.068')
        self.declare_parameter("d_time",'0.05')
        self.subscription = self.create_subscription(
            Wheel,
            'Wheel_speed',
            self.wheel_speed_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            'diff_odom',
            10)
        self.current_pose = [0.0,0.0,0.0]
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
    def wheel_speed_callback(self,msg):
        r =float(self.get_parameter("wheel_radius").get_parameter_value().string_value)
        self.get_logger().info(msg)
        left_speed = msg.left_wheel_rps*2*pi*r
        right_speed= msg.right_wheel_rps*2*pi*r
        wheel_base = float(self.get_parameter("car_width").get_parameter_value().string_value)
        linear_vel = (left_speed + right_speed) / 2
        angular_vel = (right_speed - left_speed) / wheel_base
        time_step = float(self.get_parameter("d_time").get_parameter_value().string_value)
        self.current_pose[0] += linear_vel * cos(self.current_pose[2]) * time_step
        self.current_pose[1] += linear_vel * sin(self.current_pose[2]) * time_step
        self.current_pose[2] += angular_vel * time_step
        twist_msg = Twist(
            linear=Vector3(x=self.current_pose[0], y=self.current_pose[1], z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.current_pose[2]))
        self.publisher.publish(twist_msg)
#        self.update_odometry(linear_vel, angular_vel)

'''
    def update_odometry(self, linear_vel, angular_vel):
        time_step = float(self.get_parameter("d_time").get_parameter_value().string_value)
        self.current_pose[0] += linear_vel * cos(self.current_pose[2]) * time_step
        self.current_pose[1] += linear_vel * sin(self.current_pose[2]) * time_step
        self.current_pose[2] += angular_vel * time_step
        twist_msg = Twist(
            linear=Vector3(x=self.current_pose[0], y=self.current_pose[1], z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.current_pose[2]))
        self.publisher.publish(twist_msg)
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
'''
      

def main(args=None):
    rclpy.init(args=args)
    odom_calculator = OdomCalculator()
    rclpy.spin(odom_calculator)
    odom_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

