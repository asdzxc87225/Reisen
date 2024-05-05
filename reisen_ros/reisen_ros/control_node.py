import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from reisen_msgs.msg import Wheel
import math 

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info('control node ....')
        self.declare_parameter("car_width",'0.26')
        self.declare_parameter("wheel_radius",'0.035')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.Wheel = self.create_publisher(
            Wheel,
            'Wheel',
            10)
        self.get_logger().info('control node start')

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        speed = Wheel()
        L = float(self.get_parameter('car_width').get_parameter_value().string_value)
        r = float(self.get_parameter('wheel_radius').get_parameter_value().string_value)
        speed.right_wheel_rps = (linear_vel+angular_vel*L/2)/(r*2*math.pi)
        speed.left_wheel_rps  = (linear_vel-angular_vel*L/2)/(r*2*math.pi)
        print("speed=",speed)
        self.get_logger().info('whell:\t%f'%speed.right_wheel_rps)
        self.get_logger().info('left:\t%f'%speed.left_wheel_rps)
        self.Wheel.publish(speed)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

