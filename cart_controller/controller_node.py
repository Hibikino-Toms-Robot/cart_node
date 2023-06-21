#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry   
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Int16

import serial
import numpy as np
import math
import serial
import tf2_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ptick


class Cart_Controller(Node):
    def __init__(self,**args):
        super().__init__('cart_node')
        self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=None)
        self.x=0
        self.y=0
        self.odem_publisher_ = self.create_publisher(Int16, '/odom', 10)
        self.mode_sub_ = self.create_subscription(String,'/cart_mode',self.controller,10)
        self.ekf_module=EKF()
        timer_period=0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)    

    def timer_callback(self):
        try :
            data_ = self.ser.readline().decode() 
            data = data_.split('\r')[0]
            if data:
                distanse=Int16()
                distanse.data=int(data)

                self.ekf(distanse)
                self.odem_publisher_.publish(distanse)
        except:
            pass    
    

    def controller(self,data):
        mode=data.data
        if mode=="forword":
            cmd_vel = 15
        elif mode=="back":
            cmd_vel = -15
        else :
            cmd_vel = 0
        message = bytes(str(cmd_vel) + "\n",'utf-8')
        self.ser.write(message.encode())
            
    
    def ekf(self,distanse):
        pass


class EKF():
    G = np.array([[1]])
    F = np.array([[1]])
    W = np.array([[1]]) # 恣意的に与える必要がある
    V = np.array([[10]]) # 上に同じ
    T = 100

    def __init__(self,**args):
        self.x=np.array([0,0,0])
        self.sita=np.array([0,0,0])
        self.delta=np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.q=0 #除隊遷移ノイズ
        self.r=0 #観測ノイズ
        self.m0 = np.array([[0]])
        self.C0 = np.array([[1e7]])
        # 結果を格納するarray
        self.m = np.zeros((self.T, 1))
        self.C = np.zeros((self.T, 1, 1))
        self.s = np.zeros((self.T, 1))
        self.S = np.zeros((self.T, 1, 1))

    def kalman_filter(m, C, y, G=G, F=F, W=W, V=V):
        """
        Kalman Filter
        m: 時点t-1のフィルタリング分布の平均
        C: 時点t-1のフィルタリング分布の分散共分散行列
        y: 時点tの観測値
        """
        a = G @ m
        R = G @ C @ G.T + W
        f = F @ a
        Q = F @ R @ F.T + V
        # 逆行列と何かの積を取る場合は、invよりsolveを使った方がいいらしい
        K = (np.linalg.solve(Q.T, F @ R.T)).T
        # K = R @ F.T @ np.linalg.inv(Q)
        m = a + K @ (y - f)
        C = R - K @ F @ R
        return m, C

    def kalman_smoothing(s, S, m, C, G=G, W=W):
        """
        Kalman smoothing
        """
        # 1時点先予測分布のパラメータ計算
        a = G @ m
        R = G @ C @ G.T + W
        # 平滑化利得の計算
        # solveを使った方が約30%速くなる
        A = np.linalg.solve(R.T, G @ C.T).T
        # A = C @ G.T @ np.linalg.inv(R)
        # 状態の更新
        s = m + A @ (s - a)
        S = C + A @ (S - R) @ A.T
        return s, S
    

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
 