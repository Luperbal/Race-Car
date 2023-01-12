#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import sys

Pose = []
path_info = []
Goal = []
reach_goal = False
MAX_VELOCITY = 0.5
MIN_VELOCITY = 0
max_angle = 1
steering_velocity = 1
jerk = 0.0
acceleration = 0.0
LOOKAHEAD_DISTANCE = 0.4
Low_Speed_Mode = False
        
def callback_read_current_position(data):

    global path_info
    global Pose
    global Low_Speed_Mode
    global reach_goal

    # Si se ha llegado al final, paramos de actualizar la información
    if reach_goal: 
        path_info = []
        Pose = []
        ackermann_control = AckermannDriveStamped()
        ackermann_control.drive.speed = 0.0
        ackermann_control.drive.steering_angle = 0.0
        ackermann_control.drive.steering_angle_velocity = 0.0

    if not len(path_info) == 0:
        # Se guardan los puntos en x,y,w
        path_points_x = [float(point[0]) for point in path_info]
        path_points_y = [float(point[1]) for point in path_info]
        path_points_w = [float(point[2]) for point in path_info]

        # Lee la posicion alctual del coche
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        # calculamos el angulo yaw
        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        Pose = [float(x), float(y), float(yaw)]
        
        goal_aux = Goal # Variable auxiliar para guardarnos la variable Goal y no tener después problemas con las variables gloables

        if dist(goal_aux,Pose) < 1.0:
            Low_Speed_Mode = True
            if dist(goal_aux, Pose) < 0.3:
                reach_goal = True
                print('OBJETIVO CONSEGUIDO!')
            else:
                print('Acercandose al objetivo')
        else:
            Low_Speed_Mode = False

        # Buscamos el punto del camino más cercano al vehículo
        dist_array = np.zeros(len(path_points_x))
        for i in range(len(path_points_x)):
            dist_array[i] = dist((path_points_x[i], path_points_y[i]), (x, y))
        
        goal = np.argmin(dist_array) # Devuelve el indice del valor mas pequeño del vector dist_array

#--------------------------------------------------------------------------------------------------------------------------------------       
        goal_array = np.where((dist_array < (LOOKAHEAD_DISTANCE + 0.3)) & (dist_array > (LOOKAHEAD_DISTANCE - 0.3)))[0]
        for id in goal_array:
            v1 = [path_points_x[id] - x, path_points_y[id] - y]
            v2 = [math.cos(yaw), math.sin(yaw)]
            diff_angle = find_angle(v1,v2)
            if abs(diff_angle) < np.pi/4: 
                goal = id
                break
#--------------------------------------------------------------------------------------------------------------------------------------       
        
        L = dist_array[goal]

        # Se calcula la curvatura y se transforma al angulo de giro de las ruedas
        diff_angle = path_points_w[goal] - yaw # Se calcula el angulo de giro
        r = L/(2*math.sin(diff_angle)) # Se calcula el radio de giro
        angle = 2 * math.atan(0.4/r) # Se calcula el angulo finañ

        #Si algún ángulo calculado supera los limtes por defecto, se modican esos valores
        if angle < -max_angle :
            angle = -max_angle

        if angle > max_angle :
            angle = max_angle

        if abs(angle) < 0.1 :
            angle = 0

        VELOCITY = speed_control(angle)

        # Se escribe la velocidad y el angulo como un mensaje ackermann
        ackermann_control = AckermannDriveStamped()
        ackermann_control.drive.speed = VELOCITY
        ackermann_control.drive.steering_angle = angle
        ackermann_control.drive.steering_angle_velocity = steering_velocity   
    else:
        ackermann_control = AckermannDriveStamped()
        ackermann_control.drive.speed = 0.0
        ackermann_control.drive.steering_angle = 0.0
        ackermann_control.drive.steering_angle_velocity = 0.0
    
    navigation_input.publish(ackermann_control)

def callback_read_path(data):
    # Lee en tiempo real la pose (solo se queda con x,y,yaw) y lo carga a path_info
    global path_info 
    global Goal

    path_info = []
    path_array = data.poses

    for path_pose in path_array:
        path_x = path_pose.pose.position.x
        path_y = path_pose.pose.position.y
        path_qx = path_pose.pose.orientation.x
        path_qy = path_pose.pose.orientation.y
        path_qz = path_pose.pose.orientation.z
        path_qw = path_pose.pose.orientation.w
        path_quaternion = (path_qx, path_qy, path_qz, path_qw)
        path_euler = euler_from_quaternion(path_quaternion)
        path_yaw = path_euler[2]
        path_info.append([float(path_x), float(path_y), float(path_yaw)])
    
    Goal = list(path_info[-1]) # Guardamos el ultimo valor del camino como el punto objetivo

# Calcula la distancia euclidea entre 2 puntos
def dist(p1, p2):
    if(len(p1) > 0 and len(p2) > 0):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    else:
        return 0.5

# Devulve el angulo que se forma entre la dirección del coche y la dirección objetivo
def find_angle(v1, v2):
    cos_ang = np.dot(v1, v2)
    sin_ang = LA.norm(np.cross(v1, v2))
    return np.arctan2(sin_ang, cos_ang) 


# Se asume que la velocidad es proporcional al angulo de giro
def speed_control(angle):

    if Low_Speed_Mode:
        Velocity = 0.5
    else:
        k = (MIN_VELOCITY - MAX_VELOCITY)/max_angle + 0.5
        Velocity = k * abs(angle) + MAX_VELOCITY

    return Velocity

    
if __name__ == "__main__":

    global current_pose
    global path_pose
    global navigation_input

    rospy.init_node("pursuit_path")

    current_pose = rospy.Subscriber('/pf/pose/odom', Odometry, callback_read_current_position, queue_size=1)
    path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, callback_read_path, queue_size=1)
        
    navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
    
    rospy.spin()
