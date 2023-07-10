#! /usr/bin/env python
# coding:utf-8

#USFQ Leader Motion
import numpy as np
from common import *
from time import sleep

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from arm_transbot import Transbot_ARM
from dynamic_reconfigure.server import Server
from transbot_laser.cfg import laserAvoidPIDConfig

class leaderMotion:
    def __init__(self):
        rospy.on_shutdown(self.cancel)

        self.Leader = True
        self.servo_id = [7,8,9]
        self.color = [255,0,0]
        self.pub_TargetAngle = rospy.Publisher("/TargetAngle", Arm, queue_size=10)

        self.r = rospy.Rate(20)
        self.linear = 0.1
        self.angular = 0.5
        self.ResponseDist = 0.55 #m de distancia
        self.LaserAngle = 30  # 10~180 Angulo de data 
        self.Moving = False
        self.switch = False
        self.running = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.Front_warning = 0
        self.ros_ctrl = ROSCtrl()
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)

    def cancel(self):
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config
        
    def registerScan(self,scan_data):
        if self.running == True:
            return
        ranges = np.array(scan_data.ranges)

        #Retorna los indices del arreglo ordenado
        sortedIndices = np.argsort(ranges)

        #Usamos estos valores para detectar colisiones
        self.Right_warning = 0
        self.Left_warning = 0
        self.Front_warning = 0

        for i in sortedIndices:
            #Lidar que den 360 scan -> arreglo de 720 

            #Filtro los objetos a una distancia menor a 0.55 m y activo 
            #Warning en su respectivo lado
            if len(np.array(scan_data.ranges)) == 720:
                #Izquierda
                if 20 < i < self.LaserAngle*2:
                    if ranges[i] < self.ResponseDist:
                        self.Left_warning += 1
                elif (720 - self.LaserAngle*2) < i < 700:
                    if ranges[i] < self.ResponseDist:
                        self.Right_warning += 1
                elif (700 <= i) or (i <= 20):
                    if ranges[i] <= self.ResponseDist:
                        self.Front_warning += 1
        
            #Lidar de 180 scan -> arreglo de 360
            elif len(np.array(scan_data.ranges)) == 360:
                #Todo dividido para 2
                #Izquierda
                if 10 < i < self.LaserAngle:
                    if ranges[i] < self.ResponseDist:
                        self.Left_warning += 1
                #Derecha
                elif (350 - self.LaserAngle) < i < 350:
                    if ranges[i] < self.ResponseDist: 
                        self.Right_warning += 1
                #Adelante
                elif (350 <= i <= 360) or (0 <= i <= 10):
                    if ranges[i] < self.ResponseDist:
                        self.Front_warning += 1
    
    
    def circle_move(self):
        while not rospy.is_shutdown():
            if self.ros_ctrl.Joy_active or self.switch == True:
                if self.Moving == True:
                    self.ros_ctrl.pub_vel.publish(Twist())
                    self.Moving = not self.Moving
                continue
            self.Moving = True
            twist = Twist()
        if self.Front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            twist.linear.x = self.linear
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
        self.r.sleep()
if __name__ == '__main__':
    rospy.init_node('leader_motion', anonymous=False)
    robot = leaderMotion()
    robot.circle_move()
    rospy.spin
    robot.cancel()



        