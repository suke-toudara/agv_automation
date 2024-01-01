#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import sin, cos, tan, atan2, sqrt
import serial
import time

class Wheel_Controller(Node):
    WHEEL_BASE = 0.4  # ロボットのホイールベース（車輪の間の距離）[m]
    WHEEL_RADIUS = 0.01  # ホイールの半径 [m]

    def __init__(self,**args):
        super().__init__('differential_drive_controller')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.twist_callback,10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
         
    def twist_callback(self, msg):
        linear_velocity = msg.linear.x  # 線形速度
        angular_velocity = msg.angular.z  # 角速度
        #velocity
        left_wheel_velocity = (2 * linear_velocity - angular_velocity * self.WHEEL_BASE) #/ (2 * self.WHEEL_RADIUS)
        right_wheel_velocity = (2 * linear_velocity + angular_velocity * self.WHEEL_BASE) #/ (2 * self.WHEEL_RADIUS)
        #angular_velocity
        left_wheel_angular_velocity = left_wheel_velocity / self.WHEEL_RADIUS
        right_wheel_angular_velocity = right_wheel_velocity / self.WHEEL_RADIUS
        #send_commannd_to_arduino
        self.send_command(left_wheel_angular_velocity,right_wheel_angular_velocity)

    def send_command(self, left_val,right_val):
        send_data = 'S'+str(int(left_val))+','+str(int(right_val))+'E'
        print(send_data)
        # self.ser.write(send_data.encode(encoding='utf-8'))
        # try:
        #     self.ser.timeout = 1 #(s)
        #     line = self.ser.readline()
        #     receive_data = line.strip().decode('UTF-8')    
        #     if receive_data !="":
        #         return receive_data
        #     else :
        #         return None
        # except serial.serialutil.SerialTimeoutException:
        #     print("タイムアウトエラー: データの受信がタイムアウトしました")
        #     return None
        
def main():
    try:
        rclpy.init()
        node = Wheel_Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+Cが入力されました")  
        print("プログラム終了") 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 