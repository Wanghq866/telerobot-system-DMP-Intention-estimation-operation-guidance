#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy import io as scio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent


class TouchPose():
    def __init__(self):
        self.pose = [.0,.0,.0]
    
    def Callback(self,msg):
        self.pose = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]

class ButtonFlag():
    def __init__(self):
        self.grayFlag = False
        #self.whiteFlag = False
        
    def Callback(self,msg):
        if msg.grey_button == 1:
            self.grayFlag = True
        else:
            self.grayFlag = False

class TouchForce():
    def __init__(self):
        self.force = .0
    
    def Callback(self,msg):
        self.force = ((msg.force.y)**2+(msg.force.z)**2)**0.5

class OperatCreditablity():
    def __init__(self):
        self.credit = [.25,.25,.25,.25,.25]
        self.M = [0]
        
    def Callback(self,msg):
        self.credit = [msg.creditability_1,msg.creditability_2,msg.creditability_3,msg.creditability_4,msg.creditabilityMax]
        self.M = msg.M
 
class ObtCanonical():
    def __init__(self):
        self.canonival = .0
    
    def Callback(self,msg):
        self.canonival = msg.canonical

class ObtDevia():
    def __init__(self):
        self.deviaVal = .0
    
    def Callback(self,msg):
        self.deviaVal = msg.floatval

class ObtTimeItem():
    def __init__(self):
        self.timeval = .0
    
    def Callback(self,msg):
        self.timeval = msg.floatval


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('Touch_executedTraj_plot', anonymous = True)
    
    data = scio.loadmat('/home/sxm/project_touch_mra7a/src/omni_common/Replay_lead_trajectory_fuzzy_fusion/Two_Subspace/Etouch_traj_2.mat')  # 读出来的数据是字典dict型

    y_plan1 = data['Etouch_traj1'] 
    y_plan2 = data['Etouch_traj2']
    y_plan3 = data['Etouch_traj3']
    y_plan4 = data['Etouch_traj4']
    y_plan5 = data['Etouch_traj5']
    y_plan6 = data['Etouch_traj6']

    # 获取Touch位姿
    curpose = TouchPose()
    rospy.Subscriber("/phantom/pose", PoseStamped, curpose.Callback)
    
    # 获取按钮标志
    buttonflag = ButtonFlag()
    rospy.Subscriber("/phantom/button", OmniButtonEvent, buttonflag.Callback)
    
    ExecutedTraj = np.array([.0,.0,.0])
    ExecutedTraj = ExecutedTraj[np.newaxis, :]
    
    
    fig = plt.figure(figsize=(5,4))
    axa = fig.gca(projection='3d')
        
    while not rospy.is_shutdown():
         
        # --------- 按下按钮，开始绘图 ---------
        if buttonflag.grayFlag == 1:
            ExecutedTraj = np.vstack((ExecutedTraj,curpose.pose)) # print(ExecutedTraj) # 3列
            
            
        plt.cla()

        axa.plot(y_plan1[0,:],y_plan1[1,:],y_plan1[2,:],'--')
        axa.plot(y_plan2[0,:],y_plan2[1,:],y_plan2[2,:],'--')
        axa.plot(y_plan3[0,:],y_plan3[1,:],y_plan3[2,:],'--')
        axa.plot(y_plan4[0,:],y_plan4[1,:],y_plan4[2,:],'--')
        axa.plot(y_plan5[0,:],y_plan5[1,:],y_plan5[2,:],'--')
        axa.plot(y_plan6[0,:],y_plan6[1,:],y_plan6[2,:],'--')
        # axa.plot(curpose.pose[0],curpose.pose[1],curpose.pose[2],'o')
        axa.plot(ExecutedTraj[1:,0],ExecutedTraj[1:,1],ExecutedTraj[1:,2],'k')
        plt.xlim(-0.025,-0.01)
        axa.set_xlabel('x (meter)')
        axa.set_ylabel('y (meter)')
        axa.set_zlabel('z (meter)')
        axa.set_title("Pre-defined trajectories and following: 3D")
        
        plt.pause(0.005)
