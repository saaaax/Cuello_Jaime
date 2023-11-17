#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
import time 

home=None
pos=[0,0,0,0,0]
#Función calback lectura posicion inidical
def callback_read(data):
    # rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)
    if home==None : 
     pos[0]=(data.dynamixel_state[0].present_position)
     pos[1]=(data.dynamixel_state[1].present_position)
     pos[2]=(data.dynamixel_state[4].present_position)
        # for i in range(3):
        #     self.pos[i]=(data.dynamixel_state[i].present_position)
     global home
     home=pos
            
     print(home)
     print("offset is : ")
     print(9000-home[0])
     print(-1000-home[1])

# Inicializa el nodo ROS
rospy.init_node('IK_publisher', anonymous=True)
#rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('IK_node', Float64MultiArray, queue_size=10)
# if home==None:
#     rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

#     print(home)
while not rospy.is_shutdown():
        
        
    # Crea un mensaje Float64MultiArray y publícalo
    #rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

    data = Float64MultiArray()
    data.data = [input("motor 1 :"),input("motor 2 :"), input("motor 3 :"), input("motor 4:"),input("motor 5 :")]
    rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)
    pub.publish(data)
        
        
