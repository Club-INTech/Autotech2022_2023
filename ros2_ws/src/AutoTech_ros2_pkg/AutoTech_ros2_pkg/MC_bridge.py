import sys

sys.path.append("~/Autotech2022_2023/build_tool/com")

from processus_MC import *

import rcply
from rcply.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import serial
import math

#Opening the USB port
arduino = serial.Serial(port = '/dev/ttyACM0', timeout=0)

class MC_bridge(Node):
    def __init__(self):
        super().__init__('MC_bridge')
        self.safety = self.create_subscription(Bool, 'safety/bool', 10 )
        self.ai = self.create_subscription(Twist, 'ia_node/topic/twist', 10)
        self.teleop = self.create_subscription(Twist, 'teleop/cmd_vel', 10)
        self.subsbsription  #Prevent unused variable warning

    def callback(self):
        emergency_stop(arduino, self.safety) #Send info to the MC if an emergency stop is needed
        if self.teleop == None: #Test to know if we are in the ai mode or in the teleop mode
            speed_target = math.sqrt((self.ai.linear.x)**2 + (self.ai.linear.y)**2) #Speed that we want to reach
            if self.ai.linear.x >= 0: #Test to know if we want to go forward or backward
                forward_speed(arduino, speed_target)
            else : 
                backward_speed(arduino, speed_target)
            angular_variation = self.ai.angular.z #Correspond to the angular variation asked
        else:
            speed_target = math.sqrt((self.teleop.linear.x)**2 + (self.teleop.linear.y)**2) #Speed that we want to reach
            if self.teleop.linear.x >= 0: #Test to kno if we want to go forward or backward
                forward_speed(arduino, speed_target)
            else : 
                backward_speed(arduino, speed_target)
            angular_variation = self.teleop.angular.z #Correspond to the angular variation asked     
        rot(arduino, angular_variation) 

        
def main(args=None):
    rclpy.init(args=args)
    mc_bridge = MC_bridge()
    rclpy.spin(mc_bridge)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mc_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


