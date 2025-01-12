#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import pandas as pd
from scipy import io as scio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent

# Stefan Schaal 博士（2002）论文中描述的正则动态系统
class CanonicalSystem:
    def __init__(self, run_time, dt, ax = 0.1):
        # dt float: the timestep; ax float: 增益项
        self.ax = ax
        self.dt = dt
        self.run_time = run_time
        self.timesteps = int(self.run_time / self.dt)
        self.reset_state()

    # 重置系统状态
    def reset_state(self):
        self.x = 1.0

    # 生成一个 x 的单步, x 按照 dx = -ax*x 衰减.
    # tau float: 时间增益，增加tau，系统执行速度更快
    # error_coupling (误差耦合) float: 如果误差>1，放慢速度
    def step(self, tau = 1.0, error_coupling = 1.0):
        # x = x + (-ax*x*error_coupling)*tau*dt
        self.x += (-self.ax * self.x * error_coupling) * tau * self.dt
        return self.x

    # 为开环运动生成x
    def rollout(self, **kwargs):
        if "tau" in kwargs:
            timesteps = int(self.timesteps / kwargs["tau"])
        else:
            timesteps = self.timesteps
            
        self.x_track = np.zeros(timesteps)

        self.reset_state()
        
        for t in range(timesteps):
            self.x_track[t] = self.x
            self.step(**kwargs)

        return self.x_track

# Dynamic Movement Primitives
class DMPs(object):
    # n_dmps int: DMP的数量；n_bfs int: 每个DMP中基函数的数量；y0 list: DMPs的初始状态；
    # goal list: DMPs的目标状态；w list: 可调参数，基函数的权重；ay int: 增益；by int: 增益
    def __init__(self, n_dmps, run_time, dt, n_bfs, ay=None, by=None, **kwargs):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt
        self.timesteps = int(run_time / dt)
        self.y0 = np.ones(self.n_dmps)
        self.goal = np.ones(self.n_dmps)
        self.w = np.zeros((self.n_dmps, self.n_bfs))
        self.ay = np.ones(n_dmps) * 25.0 if ay is None else ay  # Schaal 2012
        self.by = self.ay / 4.0 if by is None else by  # Schaal 2012
        self.cs = CanonicalSystem(run_time = run_time, dt = self.dt, **kwargs) # set Canonical System
        
        self.reset_state()

        self.gen_centers()

        # set variance of Gaussian basis functions 高斯基函数的集方差
        # trial and error to find this spacing 反复试验以找到此间距 (径向基函数中的 h = 1/(2*sigma^2))
        self.h = np.ones(self.n_bfs) * self.n_bfs ** 1.5 / self.c / self.cs.ax

    # 检查初始位置和目标是否相同，如果是，稍微偏移，使强制项不为0
    def check_offset(self):
        for d in range(self.n_dmps):
            if abs(self.y0[d] - self.goal[d]) < 1e-4:
                self.goal[d] += 1e-4

    def reset_state(self):
        # 重置系统状态
        self.y = self.y0.copy() # 一个DMP对应一个单维度目标
        self.dy = np.zeros(self.n_dmps)
        self.ddy = np.zeros(self.n_dmps)
        self.cs.reset_state() # CS: self.x = 1.0

    # 将高斯基函数的中心设置为在整个运行时间内均匀分布
    def gen_centers(self):
        # n_bfs个基函数的中心在整个时间内均匀分布
        des_c = np.linspace(0, self.cs.run_time, self.n_bfs)

        self.c = np.ones(len(des_c))

        for n in range(len(des_c)):
            # 用时间上均匀布置的点t1,t2,...对应的x变量x1,x2,...来作为径向基函数的中心点
            self.c[n] = np.exp(-self.cs.ax * des_c[n]) # x = exp(-ax*t)

    # 在强迫项上生成递减前沿项。x float:规范cs系统的当前值; dmp_num int:当前dmp的索引
    def gen_front_term(self, x, dmp_num):
        return x * (self.goal[dmp_num] - self.y0[dmp_num]) # x(g-y0)

    # 为给定的规范系统rollout生成基函数的激活。x float, array: 规范系统状态或路径
    def gen_psi(self, x):
        if isinstance(x, np.ndarray): # 判断一个对象是否是一个已知的类型
            x = x[:, None] # None 相当于 np.newaxis，给数组增加维度，原先(n,)-->增加维度(n,1)
        # print(x.shape) # (3827, 1) | print(self.h.shape) # (200,) | print(self.c.shape) # (200,)
        return np.exp(-self.h * (x - self.c) ** 2) # 径向基函数，exp(-h(x-ci)^2) # (3827, 200)

    # 在基函数上生成一组权重 w，以便匹配目标强迫项轨迹。f_target np.array:所需的强迫项轨迹
    def gen_weights(self, f_target):
        # 计算 x 和 psi
        x_track = self.cs.rollout()  # x=x+(-ax*x)*tau*dt, # print(x_track.shape) # (3827,), 后边tau改变，不影响这里x_track的维度
        # plt.figure(50)
        # plt.plot(x_track)
        psi_track = self.gen_psi(x_track)  # exp(-h(x-ci)^2), h=1/(2*sigma^2)
        # print(psi_track.shape) # (3827, 200)

        # 使用加权线性回归有效地计算BF权重
        self.w = np.zeros((self.n_dmps, self.n_bfs)) # 每个DMP的每个径向基函数bfs对应一个权重，(3, 200)
        for d in range(self.n_dmps):
            # 每个DMP
            k = self.goal[d] - self.y0[d]  # g-y0
            for b in range(self.n_bfs):
                # sum(x*exp(-h(x-ci)^2)*f_target)
                numer = np.sum(x_track * psi_track[:, b] * f_target[:, d]) # psi_track[:, b]: (3827,)
                # sum(x^2*exp(-h(x-ci)^2))
                denom = np.sum(x_track ** 2 * psi_track[:, b])
                self.w[d, b] = numer / denom # 求第d个DMP对应的第b个rbf的系数
                if abs(k) > 1e-5:
                    self.w[d, b] /= k

        self.w = np.nan_to_num(self.w) # 用零替换NaN
        #print(self.w.shape) # (3, 200)

    # 模仿路径，获取所需的轨迹，并生成最能实现该路径的系统参数集
    # y_des list/array: 每个DMP的期望轨迹应该格式化为[n_dmps，run_time]
    def imitate_path(self, y_des, plot=False):
        # set initial state and goal
        # print(y_des.shape) # (3, 3827)
        # 当学习一个维度的DMP时，即输入的模仿轨迹是一维的，这里增加维度。例(1000,)-->(1,1000)
        if y_des.ndim == 1: 
            y_des = y_des.reshape(1, len(y_des))
        self.y_des = y_des.copy()
        self.y0 = y_des[:, 0].copy()
        self.goal = y_des[:, -1].copy()
        
        self.check_offset()
        
        # 生成插值所需轨迹的函数
        # 采集的数据是离散的点，只有在采样点上才有具体的值，在其他区域没有值数据。
        # 插值分析，将采样点的数值根据一定的算法，推算出其他未采样区域的数值。
        import scipy.interpolate # 插值函数

        path = np.zeros((self.n_dmps, self.timesteps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        for d in range(self.n_dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[d])
            for t in range(self.timesteps):
                path[d, t] = path_gen(t * self.dt)
        y_des = path

        # 用中心差分法计算y_des的速度与加速度
        dy_des = np.gradient(y_des, axis=1) / self.dt
        # print(dy_des.shape) # (3, 3827)
        ddy_des = np.gradient(dy_des, axis=1) / self.dt
        # print(ddy_des.shape) # (3, 3827)

        f_target = np.zeros((y_des.shape[1], self.n_dmps)) # (3827, 3)
        # 找到沿着这个轨迹移动所需的力
        for d in range(self.n_dmps):
            # f_target = ddy-ay(by(g-y)-dy)
            f_target[:, d] = ddy_des[d] - self.ay[d] * (self.by[d] * (self.goal[d] - y_des[d]) - dy_des[d])
        # print(f_target.shape) # (3827, 3)

        # 计算权重值
        self.gen_weights(f_target)

        if plot is True:
            # 绘制基函数激活
            plt.figure(0)
            plt.subplot(211)
            # 计算基函数 x = x+(-ax*x)*tau*dt, psi_track = exp(-h(x-ci)^2), (3827,200)
            psi_track = self.gen_psi(self.cs.rollout()) 
            plt.plot(psi_track)
            plt.title("basis functions")

            # 绘制所需的强制函数与近似值
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii) # (2,3,4),(2,3,5),(2,3,6)
                plt.plot(f_target[:, ii], "--", label = "f_target %i" % (ii+1))
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii)
                # self.w[ii] #(200,), psi_track * self.w[ii] # (3827, 200), np.sum(psi_track * self.w[ii], axis = 1) # (3827,) 按行相加
                # sum(psi_track*w)*x*(g-y0)/sum(psi_track)
                plt.plot(
                    np.sum(psi_track * self.w[ii], axis = 1) * self.cs.rollout() * (self.goal[ii] - self.y0[ii]) / np.sum(psi_track, axis = 1), 
                    label="w*psi %i" % (ii+1)) # self.w.shape (3,200)
                plt.legend()
            plt.title("DMP forcing function")
            plt.tight_layout()
            plt.show()

        self.reset_state()
        return y_des

    # 调用一次step,计算一个时刻的轨迹的位置、速度、加速度
    # 运行一个时间步长的DMP系统；tau float: 缩放时间，增加tau系统执行得会更快；error float: 可选系统反馈
    def step(self, tau = 1.0, error = 0.0, external_force = None):
        error_coupling = 1.0 / (1.0 + error)
        x = self.cs.step(tau = tau, error_coupling=error_coupling) # x = x + (-ax*x*error_coupling)*tau*dt
        psi = self.gen_psi(x) # exp(-h(x-ci)^2)
        
        for d in range(self.n_dmps):
            # 生成强制项
            f = self.gen_front_term(x, d) * (np.dot(psi, self.w[d]))/np.sum(psi) # x(g-y0)*exp(-h(x-ci)^2)*w/sum(exp(-h(x-ci)^2))
            # DMP 加速度
            self.ddy[d] = self.ay[d] * (self.by[d] * (self.goal[d] - self.y[d]) - self.dy[d]) + f # ddy = ay(by(g-y)-y)+f
            if external_force is not None:
                self.ddy[d] += external_force[d]
            self.dy[d] += self.ddy[d] * tau * self.dt * error_coupling # dy = ddy*tau*dt*error_coupling
            self.y[d] += self.dy[d] * tau * self.dt * error_coupling # y = dy*tau*dt*error_coupling
        #print(self.y)
        #print("A")
        return self.y, self.dy, self.ddy

    # 生成学习到的轨迹，不包含任何反馈
    def rollout(self, timesteps=None, **kwargs):
        self.reset_state()

        if timesteps is None:
            if "tau" in kwargs:
                timesteps = int(self.timesteps / kwargs["tau"])
            else:
                timesteps = self.timesteps
                
        # 初始化
        y_track = np.zeros((timesteps, self.n_dmps))
        dy_track = np.zeros((timesteps, self.n_dmps))
        ddy_track = np.zeros((timesteps, self.n_dmps))

        for t in range(timesteps):
            # 运行并记录时间步长
            y_track[t], dy_track[t], ddy_track[t] = self.step(**kwargs)
       
        return y_track, dy_track, ddy_track

class TouchPose():
    def __init__(self):
        self.pose = [.0,.0,.0]
    
    def Callback(self,msg):
        self.pose = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]

class ButtonFlag():
    def __init__(self):
        self.Flag = False
        
    def Callback(self,msg):
        if msg.grey_button == 1:
            self.Flag = True
        else:
            self.Flag = False

def DMPLearn(y_des, tau):
    Time = y_des.shape[1]/100
    dmp = DMPs(n_dmps = 3, run_time = Time, dt = 0.001, n_bfs = 120, ay = np.ones(3) * 80.0)
    dmp.imitate_path(y_des = y_des)
    y_plan, _, _ = dmp.rollout(tau = tau)
    
    return y_plan

if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('Tdmp_trajectory', anonymous=True)
    
    #------- 导入数据，用DMP学习得到y_plan，执行时间根据弧长确定，采样点数在matlab相关程序已经确定 ---------------------------------------------------
    data = scio.loadmat('/home/sxm/project_touch_mra7a/src/omni_common/Replay_lead_trajectory_fuzzy_fusion/Three_Subspace/Stouch_traj.mat')  # 读出来的数据是字典dict型
    y_plan1 = DMPLearn(data['Stouch_traj1'], tau = 1.0)
    rospy.loginfo("y_plan1 learned by DMP with Tau = 1.0")
    y_plan2 = DMPLearn(data['Stouch_traj2'], tau = 1.0)
    rospy.loginfo("y_plan2 learned by DMP with Tau = 1.0")
    y_plan3 = DMPLearn(data['Stouch_traj3'], tau = 1.0)
    rospy.loginfo("y_plan3 learned by DMP with Tau = 1.0")
    y_plan4 = DMPLearn(data['Stouch_traj4'], tau = 1.0)
    rospy.loginfo("y_plan4 learned by DMP with Tau = 1.0")
    y_plan5 = DMPLearn(data['Stouch_traj5'], tau = 1.0)
    rospy.loginfo("y_plan5 learned by DMP with Tau = 1.0")
    y_plan6 = DMPLearn(data['Stouch_traj6'], tau = 1.0)
    rospy.loginfo("y_plan6 learned by DMP with Tau = 1.0")
    y_plan7 = DMPLearn(data['Stouch_traj7'], tau = 1.0)
    rospy.loginfo("y_plan7 learned by DMP with Tau = 1.0")
    y_plan8 = DMPLearn(data['Stouch_traj8'], tau = 1.0)
    rospy.loginfo("y_plan8 learned by DMP with Tau = 1.0")
    y_plan9 = DMPLearn(data['Stouch_traj9'], tau = 1.0)
    rospy.loginfo("y_plan9 learned by DMP with Tau = 1.0")
    y_plan10 = DMPLearn(data['Stouch_traj10'], tau = 1.0)
    rospy.loginfo("y_plan10 learned by DMP with Tau = 1.0")
    y_plan11 = DMPLearn(data['Stouch_traj11'], tau = 1.0)
    rospy.loginfo("y_plan11 learned by DMP with Tau = 1.0")
    
    #------- 记录已经执行的轨迹，并计算已执行轨迹与y_plan的相似度-----------------------------------------------------------------------------------
    # 获取按钮标志
    buttonflag = ButtonFlag()
    rospy.Subscriber("/phantom/button", OmniButtonEvent, buttonflag.Callback)
    
    # 获取Touch位姿
    curpose = TouchPose()
    rospy.Subscriber("/phantom/pose", PoseStamped, curpose.Callback)
    
    # 定义一个期望位置的发布话题
    pub = rospy.Publisher("/phantom/force_feedback",OmniFeedback,queue_size = 10)
    PlannedTraj = OmniFeedback()
    
    #-------------------------------------------------------------------------------------------------
    rate = rospy.Rate(1000)
    
    # ****** 运动到起点位置 ****************
    COUNT1 = 0
    StartPoint = y_plan1[0,:] # 起点位置相同
    while not rospy.is_shutdown():
        if COUNT1 < 5*1000:
            PlannedTraj.header.stamp = rospy.Time.now()
            PlannedTraj.position.x = curpose.pose[0]*1000+(StartPoint[0]*1000-curpose.pose[0]*1000)/5000*COUNT1 # m --> mm
            PlannedTraj.position.y = curpose.pose[1]*1000+(StartPoint[1]*1000-curpose.pose[1]*1000)/5000*COUNT1
            PlannedTraj.position.z = curpose.pose[2]*1000+(StartPoint[2]*1000-curpose.pose[2]*1000)/5000*COUNT1
            
            pub.publish(PlannedTraj)
            
            rospy.loginfo("Count:%d --> %d, [%.3f,%.3f,%.3f], move to start point......",COUNT1, 5000, PlannedTraj.position.x,PlannedTraj.position.y,PlannedTraj.position.z)
            
            COUNT1 = COUNT1+1
            
            rate.sleep()
        else:
            break
    
    # 人为设置通过路径，验证提出方法的引导能力和效率，最佳3,6,10
    SetTraj1 = 'y_plan1' # 1,2,3,4
    SetTraj2 = 'y_plan7' # 5,6,7
    SetTraj3 = 'y_plan8' # 8,9,10,11
    # ***** 按下按钮，开始DMP引导 *******
    CountTraj1 = 0
    CountTraj2 = 0
    CountTraj3 = 0
    while not rospy.is_shutdown():
        if buttonflag.Flag == 0:
            continue
        
        # ----- 按下按钮，开始DMP引导 ----
        else:
            if CountTraj1 < len(eval(SetTraj1))-1:
                PlannedTraj.header.stamp = rospy.Time.now()
                PlannedTraj.position.x = eval(SetTraj1)[CountTraj1][0]*1000 # unit: from m to mm
                PlannedTraj.position.y = eval(SetTraj1)[CountTraj1][1]*1000
                PlannedTraj.position.z = eval(SetTraj1)[CountTraj1][2]*1000
                
                rospy.loginfo("CountTraj1:%d --> %d, [%.3f,%.3f,%.3f]",CountTraj1, len(eval(SetTraj1))-1, PlannedTraj.position.x,PlannedTraj.position.y,PlannedTraj.position.z)
                
                pub.publish(PlannedTraj)
            
                CountTraj1 = CountTraj1+1
                
                rate.sleep()
                
            elif CountTraj1 >= len(eval(SetTraj1))-1 and CountTraj2  < len(eval(SetTraj2))-1:
                PlannedTraj.header.stamp = rospy.Time.now()
                PlannedTraj.position.x = eval(SetTraj2)[CountTraj2][0]*1000 # unit: from m to mm
                PlannedTraj.position.y = eval(SetTraj2)[CountTraj2][1]*1000
                PlannedTraj.position.z = eval(SetTraj2)[CountTraj2][2]*1000
                
                rospy.loginfo("CountTraj2:%d --> %d, [%.3f,%.3f,%.3f]",CountTraj2, len(eval(SetTraj2))-1, PlannedTraj.position.x,PlannedTraj.position.y,PlannedTraj.position.z)
                
                pub.publish(PlannedTraj)
            
                CountTraj2 = CountTraj2+1
                
                rate.sleep()
                
            elif CountTraj2  >= len(eval(SetTraj2))-1 and CountTraj3 < len(eval(SetTraj3))-1:
                PlannedTraj.header.stamp = rospy.Time.now()
                PlannedTraj.position.x = eval(SetTraj3)[CountTraj3][0]*1000 # unit: from m to mm
                PlannedTraj.position.y = eval(SetTraj3)[CountTraj3][1]*1000
                PlannedTraj.position.z = eval(SetTraj3)[CountTraj3][2]*1000
                
                rospy.loginfo("CountTraj3:%d --> %d, [%.3f,%.3f,%.3f]",CountTraj3, len(eval(SetTraj3))-1, PlannedTraj.position.x,PlannedTraj.position.y,PlannedTraj.position.z)
                
                pub.publish(PlannedTraj)
            
                CountTraj3 = CountTraj3+1
                
                rate.sleep()
                
                if CountTraj3 == len(eval(SetTraj3))-2:
                    rospy.logwarn("\nThe time of justment is FIVE second!\n")
                    rospy.logwarn("Close the data subscription node! The image will be displayed soon!\n")
                    rospy.logwarn("Button again for go to start point!")
                    rospy.sleep(5.)
                
            else:
                break
    
    # 运动引导完毕，返回预设起点    
    COUNT3 = 0 
    while not rospy.is_shutdown():
        if COUNT3 < 5*1000:
            # 与MRA7A的RVIZ中Touch状态匹配的touch初始点，state->lock_pos = {-16.56,0,-12.51}; 
            PlannedTraj.header.stamp = rospy.Time.now()
            PlannedTraj.position.x = curpose.pose[0]*1000+(-16.56-curpose.pose[0]*1000)/5000*COUNT3
            PlannedTraj.position.y = curpose.pose[1]*1000+(0-curpose.pose[1]*1000)/5000*COUNT3
            PlannedTraj.position.z = curpose.pose[2]*1000+(-12.51-curpose.pose[2]*1000)/5000*COUNT3
            
            rospy.loginfo("Count:%d --> %d, Go to start point...", COUNT3, 5000)
            
            pub.publish(PlannedTraj)
        
            COUNT3 = COUNT3+1
            
            rate.sleep()

        else:
            rospy.sleep(6.)
            path = '/home/sxm/project_touch_mra7a/src/omni_common/Replay_lead_trajectory_fuzzy_fusion/Three_Subspace/Touch_tracked_traj/Touch_tracked_traj.txt'
            y_tracked = np.loadtxt(path, delimiter=',')
            
            plt.figure(1)
            for i in range(1,12):
                str1 = 'y_plan#'
                y_plan = str1.replace('#',str(i))
                plt.plot(eval(y_plan)[:, 1]*1000, eval(y_plan)[:, 2]*1000, "k", lw=2)

            plt.plot(y_tracked[1:,1]*1000, y_tracked[1:,2]*1000, "b", lw=2)
            plt.show()
            rospy.signal_shutdown("the node of planned trajectory is shutdown.")