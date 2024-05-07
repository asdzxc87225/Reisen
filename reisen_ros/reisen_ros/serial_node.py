import serial
import rclpy
from rclpy.node import Node
from reisen_msgs.msg import Wheel

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(Wheel, 'Wheel_speed', 10)
        self.subscription = self.create_subscription(
            Wheel,
            'Wheel',
            self.wheel_callback,
            10)
        self.get_logger().info('serial node start')
        COM_PORT = '/dev/ttyUSB0'  # 請自行修改序列埠名稱
        BAUD_RATES = 115200
        self.ser = serial.Serial(COM_PORT, BAUD_RATES)
        self.timer = self.create_timer(0.1, self.clear)
        self.timer1 = self.create_timer(0.05, self.serial_get)
        
    def  wheel_callback(self,msg):#給arduino目標轉速資料
        try:
            to_arduino_data = "{:.5f} {:.5f}\n".format(msg.left_wheel_rps, msg.right_wheel_rps)
            self.ser.write(to_arduino_data.encode('utf-8'))
            self.get_logger().info('get data:'+to_arduino_data)
        except:                   # 如果 try 的內容發生錯誤，就執行 except 裡的內容
            self.get_logger().info("發身錯誤")
    def serial_get(self):#讀取arduino給的當前轉速
        speed = Wheel()
        if self.ser.in_waiting > 0:
            data_raw = self.ser.readline()  # 讀取一行
            print(data_raw)
            data = data_raw.decode().split()
            print(data)
            try:
                if len(data) >= 2:
                    speed.left_wheel_rps = float(data[0])
                    speed.right_wheel_rps =float(data[1])
            except:                   # 如果 try 的內容發生錯誤，就執行 except 裡的內容
                self.get_logger().info("發身錯誤")
        else:
            speed.left_wheel_rps = 0.0
            speed.right_wheel_rps= 0.0
        self.publisher_.publish(speed)
    def clear(self):
        self.ser.reset_input_buffer( )
        self.ser.reset_output_buffer( )

def main(args=None):
    rclpy.init(args=args)
    Serial_Node = SerialNode()
    rclpy.spin(Serial_Node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

