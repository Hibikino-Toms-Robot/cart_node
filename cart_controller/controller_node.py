#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry   
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from std_msgs.msg import Int16

import numpy as np
import math
import serial
import tf2_msgs.msg
import numpy as np
from command_service.srv import CartCommand

import sys
sys.path.append("/home/suke/toms_ws/src/cart_controller/cart_controller/")
from cart_module import Control_Cart

class Cart_Controller(Node):
    def __init__(self,**args):
        super().__init__('cart_node')
        #publisher
        self.odem_publisher_ = self.create_publisher(Int16, '/odom', 10)

        #timer
        timer_period=0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)    

        #service
        self.srv = self.create_service(CartCommand, "/cart", self.controller)
        self.cart_control = Control_Cart()
    
    
    def controller(self,request, response):
        mode=request.data
        if mode=="forword":
            distanse=self.cart_control.move_cart(10.0)
        elif mode=="back":
            distanse=self.cart_control.move_cart(-10.0)        
        if distanse :
            self.odem_publisher_.publish(distanse)  
            print("----------------------------")
            self.get_logger().info(f'現在位置{distanse}')  
            response.task_comp = True
        else :
            response.task_comp = False
        return response
       
def main():
    try:
        rclpy.init()
        node = Cart_Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+Cが入力されました")  
        print("プログラム終了") 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
 