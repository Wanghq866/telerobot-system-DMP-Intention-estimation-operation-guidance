#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy import io as scio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent
from omni_msgs.msg import ForceforTouch
from omni_msgs.msg import Creditability
from omni_msgs.msg import CanonicalSyatemVal
from omni_msgs.msg import SingleValWithTypes


class TouchPose():
    def __init__(self):
        self.pose = [.0,.0,.0]
    
    def Callback(self,msg):
        self.pose = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]

class DesirPose():
    def __init__(self):
        self.pose = [.0,.0,.0]
    
    def Callback(self,msg):
        self.pose = [msg.position.x,msg.position.y,msg.position.z]

class ButtonFlag():
    def __init__(self):
        self.grayFlag = False
        
    def Callback(self,msg):
        if msg.grey_button == 1:
            self.grayFlag = True
        else:
            self.grayFlag = False
        
class TouchForce():
    def __init__(self):
        self.force = [.0,.0,.0,.0]
    
    def Callback(self,msg):
        self.force = [msg.force.x,msg.force.y,msg.force.z,((msg.force.y)**2+(msg.force.z)**2)**0.5]

class OperatCreditablity():
    def __init__(self):
        self.credit = [.0,.0,.0,.0]
        self.M = [0]
        
    def Callback(self,msg):
        self.credit = [msg.creditability_1,msg.creditability_2,msg.creditability_3,msg.creditabilityMax]
        self.M = msg.M
 
class ObtCanonical():
    def __init__(self):
        self.canonival = .0
    
    def Callback(self,msg):
        self.canonival = msg.canonical

class ObtDevia():
    def __init__(self):
        self.deviaVal = [.0,.0,.0,.0]
    
    def Callback(self,msg):
        self.deviaVal = [msg.position.x,msg.position.y,msg.position.z,(msg.position.x**2+msg.position.y**2+msg.position.z**2)**0.5]

class ObtTimeItem():
    def __init__(self):
        self.timeval = .0
    
    def Callback(self,msg):
        self.timeval = msg.floatval*2.5

class ObtCreditParam():
    def __init__(self):
        self.paramval = .0
    
    def Callback(self,msg):
        self.paramval = msg.floatval


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('Touch_executedTraj_plot', anonymous=True)
    
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
    
    # 获取DMP演化的期望位置
    desirpos = DesirPose()
    rospy.Subscriber("/phantom/force_feedback", OmniFeedback, desirpos.Callback)
    
    # 获取次优DMP演化的期望位置
    willdesirpos = DesirPose()
    rospy.Subscriber("willtraj", OmniFeedback, willdesirpos.Callback)
    
    # 获取按钮标志
    buttonflag = ButtonFlag()
    rospy.Subscriber("/phantom/button", OmniButtonEvent, buttonflag.Callback)

    # 获取Touch的力
    forceCase = TouchForce()
    rospy.Subscriber("/force_in_touch", ForceforTouch, forceCase.Callback)
    
    # 获取所有信任度
    creditall = OperatCreditablity()
    rospy.Subscriber("/creditability", Creditability, creditall.Callback)
    
    # 获取正则系统的值
    canoni = ObtCanonical()
    rospy.Subscriber("/canonicalVal", CanonicalSyatemVal, canoni.Callback)
    
    # 获取偏差值
    devia = ObtDevia()
    rospy.Subscriber("/deviationVal", OmniFeedback, devia.Callback)
    
    # 获取时间缩放系数值
    timeItem = ObtTimeItem()
    rospy.Subscriber("/tau_a_Val", SingleValWithTypes, timeItem.Callback)
    
    # 获取信任度系数值
    Credparamval = ObtCreditParam()
    rospy.Subscriber("/nu", SingleValWithTypes, Credparamval.Callback)
    
    ExecutedTraj = np.array([.0,.0,.0])
    ExecutedTraj = ExecutedTraj[np.newaxis, :]
    ExecutedForce = np.array([.0,.0,.0,.0])
    ExecutedForce = ExecutedForce[np.newaxis, :]
    ExecutedCredit = np.array([.0,.0,.0,.0])
    ExecutedCredit = ExecutedCredit[np.newaxis, :]
    ExecutedCanoni = np.array([.0])
    ExecutedCanoni = ExecutedCanoni[np.newaxis, :]
    ExecutedM = np.array([0])
    ExecutedM = ExecutedM[np.newaxis, :]
    ExecutedDevia = np.array([.0,.0,.0,.0])
    ExecutedDevia = ExecutedDevia[np.newaxis, :]
    ExecutedTimeItem = np.array([.0])
    ExecutedTimeItem = ExecutedTimeItem[np.newaxis, :]
    ExecutedCredparam = np.array([.0])
    ExecutedCredparam = ExecutedCredparam[np.newaxis, :]
    
    plt.figure(figsize=(6,10)) #10 4
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
                
        # --------- 按下按钮，开始绘图 ---------
        if buttonflag.grayFlag == 1:
            ExecutedTraj = np.vstack((ExecutedTraj,curpose.pose)) # print(ExecutedTraj) # 3列
            ExecutedForce = np.vstack((ExecutedForce,forceCase.force))
            ExecutedCredit = np.vstack((ExecutedCredit,creditall.credit))
            ExecutedCanoni = np.vstack((ExecutedCanoni,canoni.canonival))
            ExecutedM = np.vstack((ExecutedM,creditall.M))
            ExecutedDevia = np.vstack((ExecutedDevia,devia.deviaVal))
            ExecutedTimeItem = np.vstack((ExecutedTimeItem,timeItem.timeval))
            ExecutedCredparam = np.vstack((ExecutedCredparam,Credparamval.paramval))
        
        plt.cla()
        
        plt.axes().get_xaxis().set_visible(False) # 隐藏x坐标轴
        plt.axes().get_yaxis().set_visible(False) # 隐藏y坐标轴
        plt.axis('off')
        
        ax1 = plt.axes([0.15,0.85,0.8,0.14])   # [0.15,0.83,0.8,0.16]    # [左, 下, 宽, 高] 规定的矩形区域 （全部是0~1之间的数，表示比例）
        ax1.plot(y_plan1[1,:],y_plan1[2,:],'--')
        ax1.plot(y_plan2[1,:],y_plan2[2,:],'--')
        ax1.plot(y_plan3[1,:],y_plan3[2,:],'--')
        ax1.plot(y_plan4[1,:],y_plan4[2,:],'--')
        ax1.plot(y_plan5[1,:],y_plan5[2,:],'--')
        ax1.plot(y_plan6[1,:],y_plan6[2,:],'--')
        ax1.plot(desirpos.pose[1]/1000,desirpos.pose[2]/1000,'or',label = 'DMP')
        ax1.plot(curpose.pose[1],curpose.pose[2],'ob',label = 'user')
        ax1.plot(willdesirpos.pose[1]/1000,willdesirpos.pose[2]/1000,'og',label = 'will DMP')
        ax1.plot(ExecutedTraj[1:,1],ExecutedTraj[1:,2],'k')
        ax1.legend(fontsize = 4)
        ax1.set_xlabel('y (meter)')
        ax1.set_ylabel('z (meter)')
        #ax1.set_title("Pre-defined trajectories and following: 2D")
        
        hight = 0.085
        
        ax2 = plt.axes([0.15,0.715,0.8,hight]) # 0.1,0.23,0.8,0.18
        ax2.plot(ExecutedCredit[1:,0],'-', label = 'traj 1')
        ax2.plot(ExecutedCredit[1:,1],'-', label = 'traj 2')
        ax2.plot(ExecutedCredit[1:,2],'-', label = 'traj 3')
        #ax2.plot(ExecutedCredit[1:,3],'b--', label = 'max')
        ax2.legend(fontsize = 8)
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('Credit')
        # ax2.set_title("Credit")
        
        ax3 = plt.axes([0.15,0.585,0.8,hight])
        ax3.plot(ExecutedM[1:],'-')
        ax3.set_xlabel('time (s)')
        ax3.set_ylabel('M')
        # ax5.set_title("M")
        
        ax4 = plt.axes([0.15,0.455,0.8,hight])
        ax4.plot(ExecutedDevia[1:,0],'-',label='x')
        ax4.plot(ExecutedDevia[1:,1],'-',label='y')
        ax4.plot(ExecutedDevia[1:,2],'-',label='z')
        #ax4.plot(ExecutedDevia[1:,3],'-',label='e_p')
        ax4.legend(fontsize = 8)
        ax4.set_xlabel('time (s)')
        ax4.set_ylabel('Deviation')
        
        ax5 = plt.axes([0.15,0.325,0.8,hight])
        ax5.plot(ExecutedForce[1:,0],label='x')
        ax5.plot(ExecutedForce[1:,1],label='y')
        ax5.plot(ExecutedForce[1:,2],label='z')
        #ax5.plot(ExecutedForce[1:,3],label='F')
        ax5.set_xlabel('time (s)')
        ax5.set_ylabel('Force')
        ax5.legend(fontsize = 8)
        # ax2.set_title("Force")
        
        ax6 = plt.axes([0.15,0.19,0.8,hight])
        ax6.plot(ExecutedTimeItem[1:])
        ax6.set_xlabel('time (s)')
        ax6.set_ylabel('TimeItem')
        
        # ax7 = plt.axes([0.15,0.05,0.8,hight])
        # ax7.plot(ExecutedCanoni[1:],'-')
        # ax7.set_xlabel('time (s)')
        # ax7.set_ylabel('Canonical Syatem')
        
        ax8 = plt.axes([0.15,0.05,0.8,hight])
        ax8.plot(ExecutedCredparam[1:],'-')
        
        plt.pause(0.005)
        
        rate.sleep()