import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial

class Lidar(Node):
    def __init__(self):
        super().__init__('Lidar')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser.write(b'ST\n') # Send start measurement command

        self.laser_scan = self.create_publisher(LaserScan, 'scan', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = self.ser.readline()
        if data.startswith(b'ME'):
            ranges = [float(x) / 1000.0 for x in data[2:].split(b',')]
            header = self.get_msg_header()
            scan = LaserScan(header=header, angle_min=-1.5708, angle_max=1.5708, angle_increment=0.00613592, range_min=0.02, range_max=60.0, ranges=ranges)
            self.laser_scan.publish(scan)

    def get_msg_header(self):
        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = now
        header.frame_id = "laser_frame"
        return header

def main(args=None):
    rclpy.init(args=args)

    lidar = Lidar()

    rclpy.spin(lidar)

    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
