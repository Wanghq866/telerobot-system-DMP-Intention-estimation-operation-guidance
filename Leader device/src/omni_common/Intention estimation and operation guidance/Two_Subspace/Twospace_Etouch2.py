#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy import io as scio
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniFeedback
from omni_msgs.msg import OmniButtonEvent
from omni_msgs.msg import Creditability
from omni_msgs.msg import CanonicalSyatemVal
from omni_msgs.msg import SingleValWithTypes
from dtaidistance import dtw


def softmax(z, min_d, pubnu, Nu_val):
    # nu = 21
    nu = -40 * min_d + 21
    exp_z = np.exp(-nu*z)
    
    Nu_val.floatval = nu
    pubnu.publish(Nu_val)
    
    return exp_z / np.sum(exp_z)

def tanh(x, a = 130):
    x = a*x
    return (np.exp(x) - np.exp(-x)) / (np.exp(x) + np.exp(-x))

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

# Stefan Schaal 博士（2002）论文中描述的正则动态系统
class CanonicalSystem:
    def __init__(self, run_time, dt, ax):
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
    def __init__(self, n_dmps, run_time, dt, n_bfs, ax, ay=None, by=None, **kwargs):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt
        self.timesteps = int(run_time / dt)
        self.y0 = np.ones(self.n_dmps)
        self.goal = np.ones(self.n_dmps)
        self.w = np.zeros((self.n_dmps, self.n_bfs))
        self.ay = np.ones(n_dmps) * 25.0 if ay is None else ay  # Schaal 2012
        self.by = self.ay / 4.0 if by is None else by  # Schaal 2012
        self.ax = ax
        self.cs = CanonicalSystem(run_time = run_time, dt = self.dt, ax = self.ax, **kwargs) # set Canonical System
        
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
    def imitate_path(self, y_des):
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

        self.reset_state()
        
        return self.w

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
        
        return self.y, self.dy, self.ddy, x

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
            y_track[t], dy_track[t], ddy_track[t], _ = self.step(**kwargs)
       
        return y_track, dy_track, ddy_track

class DMPLearn:
    def __init__(self,y_des,ax):
        self.time = np.shape(y_des)[1]/100
        self.dmp = DMPs(n_dmps = 3, run_time = self.time, dt = 0.01, n_bfs = 120, ax = ax, ay = np.ones(3) * 80.0)
        self.weight = self.dmp.imitate_path(y_des = y_des) # 找到沿轨迹的力，计算权重
        
    def completeTraj(self,tau):
        y_plan, _, _ = self.dmp.rollout(tau = tau) # DMP计算轨迹
        
        return y_plan
    
    def oneCompute(self,tau,x):
        y_track, _, _, _ = self.dmp.step(tau,x)
        phase = self.dmp.cs.step(tau = tau)
        
        return y_track, phase

def LoadData(tau_d):
    data = scio.loadmat('/home/sxm/project_touch_mra7a/src/omni_common/Replay_lead_trajectory_fuzzy_fusion/Two_Subspace/Etouch_traj_2.mat')  # 读出来的数据是字典dict型
    
    DMP1 = DMPLearn(data['Etouch_traj1'], ax = 0.6) # (1000*Time)*3列
    y_plan1 = DMP1.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan1 learned by DMP with Tau = %.1f",tau_d)
    
    DMP2 = DMPLearn(data['Etouch_traj2'], ax = 0.8)
    y_plan2 = DMP2.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan2 learned by DMP with Tau = %.1f",tau_d)
    
    DMP3 = DMPLearn(data['Etouch_traj3'], ax = 0.7)
    y_plan3 = DMP3.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan3 learned by DMP with Tau = %.1f",tau_d)
    
    DMP4 = DMPLearn(data['Etouch_traj4'], ax = 0.5)
    y_plan4 = DMP4.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan4 learned by DMP with Tau = %.1f",tau_d)
    
    DMP5 = DMPLearn(data['Etouch_traj5'], ax = 0.7)
    y_plan5 = DMP5.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan5 learned by DMP with Tau = %.1f",tau_d)
    
    DMP6 = DMPLearn(data['Etouch_traj6'], ax = 0.6)
    y_plan6 = DMP6.completeTraj(tau = tau_d)
    rospy.loginfo("y_plan6 learned by DMP with Tau = %.1f",tau_d)

    return y_plan1, y_plan2, y_plan3, y_plan4, y_plan5, y_plan6, DMP1, DMP2, DMP3, DMP4, DMP5, DMP6

def Cutoff(y_plan,ExecutedTraj):
    # 起点对齐
    idx2 = None
    if ExecutedTraj[0,1] < y_plan[0,1]:
        idx1 = abs(ExecutedTraj[:,1]-y_plan[0,1]).argmin()
        ExecutedTraj = ExecutedTraj[idx1:,:]
    else:
        idx2 = abs(y_plan[:,1]-ExecutedTraj[0,1]).argmin() # y_plan1中与已执行路径的y的最近值
        
    # 终点对齐
    idx3 = abs(y_plan[:,1]-max(ExecutedTraj[:,1])).argmin()
    
    if idx2 is None:
        idx2 = 0
    y_exec = y_plan[idx2:idx3+1,:]

    return y_exec, ExecutedTraj, idx3

# ----------------- evolution of DMP according to phase ---------------------------------
# 根据操作轨迹与DMP演示轨迹的偏差调节系数tau，降低DMP的演示速度
class DmpEvolu():
    def __init__(self,tau_d):
        self.tau_d = tau_d
        
    def evolution(self,y_DMP,curpose,x,x_track,DesirTraj,pubpos,Devia,pubdevi,Tau_a_v,pubtau_a, ifpub = True): # 正常DMP演化：ifpub = True; willtraj: ifpub = False
        Exedis = ((curpose.pose[1]-y_DMP.dmp.y[1])**2+(curpose.pose[2]-y_DMP.dmp.y[2])**2)**0.5
        
        vector_e = np.array([curpose.pose[1]-y_DMP.dmp.y[1],curpose.pose[2]-y_DMP.dmp.y[2]]) # 操作偏差向量
        vector_e /= np.linalg.norm(vector_e) # 单位向量
        vector_dmpspeed = np.array([y_DMP.dmp.dy[1],y_DMP.dmp.dy[2]]) # DMP速度向量
        vector_dmpspeed /= np.linalg.norm(vector_dmpspeed) # 单位向量
        
        
        if ifpub == True:
            tau_a = self.tau_d*(1+np.sign(np.inner(vector_e,vector_dmpspeed))*tanh(Exedis))
            
            # 发布偏差到话题
            Devia.header.stamp = rospy.Time.now()
            Devia.position.x = curpose.pose[0]-y_DMP.dmp.y[0]
            Devia.position.y = curpose.pose[1]-y_DMP.dmp.y[1]
            Devia.position.z = curpose.pose[2]-y_DMP.dmp.y[2]
            pubdevi.publish(Devia)
            
            # 发布时间缩放项到话题
            Tau_a_v.floatval = tau_a
            pubtau_a.publish(Tau_a_v)
        else:
            tau_a = self.tau_d*(1+np.sign(curpose.pose[1]-y_DMP.dmp.y[1])*tanh(Exedis))
            
        if x == float('nan') or tau_a == float('nan'):
            tau_a == 1.0
            
            rospy.logwarn("DANGEROUS!!!, x or tau_a == float('nan')!!!")
            rospy.signal_shutdown()
        
        if x >= x_track[-1]:
            y_track, x = y_DMP.oneCompute(tau = tau_a, x = x)
            # 发布期望轨迹到话题
            DesirTraj.header.stamp = rospy.Time.now()
            DesirTraj.position.x = y_track[0]*1000 # unit: from m to mm
            DesirTraj.position.y = y_track[1]*1000
            DesirTraj.position.z = y_track[2]*1000
            
            pubpos.publish(DesirTraj)
            
        return x, tau_a, Exedis

if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('Tdmp_trajectory', anonymous = True)
    
    # ------------------------------------------ 定义话题节点 ------------------------------------------------------
    # 获取按钮标志
    buttonflag = ButtonFlag()
    rospy.Subscriber("/phantom/button", OmniButtonEvent, buttonflag.Callback)
    
    # 获取Touch位姿
    curpose = TouchPose()
    rospy.Subscriber("/phantom/pose", PoseStamped, curpose.Callback)
    
    # 定义一个[期望]位置的发布话题
    pubforce = rospy.Publisher("/phantom/force_feedback",OmniFeedback,queue_size = 10)
    PlannedTraj = OmniFeedback()
    
    # 定义一个[概率次大]位置的发布话题
    pubwilltraj = rospy.Publisher("/willtraj",OmniFeedback,queue_size = 10)
    PlannedTrajWill = OmniFeedback()
        
    # 定义一个信任度的发布话题(/creditability 是自定义的话题名)
    pubcredit = rospy.Publisher("/creditability",Creditability,queue_size = 10)
    Credit = Creditability()
    
    # 定义一个正则系统的发布话题(/canonicalVal 是自定义的话题名)
    pubcanonical = rospy.Publisher("/canonicalVal",CanonicalSyatemVal,queue_size = 10)
    Canoni = CanonicalSyatemVal()
    
    # 定义一个偏差的发布话题
    pubdevia = rospy.Publisher("/deviationVal",OmniFeedback,queue_size = 10)
    Devia = OmniFeedback()
    
    # 定义一个时间缩放系数的发布话题
    pubtau_a = rospy.Publisher("/tau_a_Val",SingleValWithTypes,queue_size = 10)
    Tau_a_v = SingleValWithTypes()
    
    # 定义一个概率调节系数的发布话题
    pubnu = rospy.Publisher("/nu",SingleValWithTypes,queue_size = 10)
    Nu_val = SingleValWithTypes()

    # -------------------------------- 预定义参数 ----------------------------------------------------------------
    tau_d = 0.65
    WINDOWS = 180
    Lamba_f = 0.98
    Lamba_t = 0.83
    
    #-----------------------------------------------------------------------------------------------------------
    #---- 导入数据，用DMP学习得到y_plan，执行时间根据弧长确定，采样点数在matlab相关程序已经确定 ----
    y_plan1, y_plan2, y_plan3, y_plan4, y_plan5, y_plan6, DMP1, DMP2, DMP3, DMP4, DMP5, DMP6 = LoadData(tau_d)
    rate = rospy.Rate(100)
    
    # ************************************************************* 运动到起点位置 ***************************************************************
    COUNT1 = 0
    StartPoint = y_plan1[0,:] # 起点位置相同
    while not rospy.is_shutdown():
        if COUNT1 < 5*100:
            PlannedTraj.header.stamp = rospy.Time.now()
            PlannedTraj.position.x = curpose.pose[0]*1000+(StartPoint[0]-curpose.pose[0])*1000/500*COUNT1 # m --> mm
            PlannedTraj.position.y = curpose.pose[1]*1000+(StartPoint[1]-curpose.pose[1])*1000/500*COUNT1
            PlannedTraj.position.z = curpose.pose[2]*1000+(StartPoint[2]-curpose.pose[2])*1000/500*COUNT1
            
            pubforce.publish(PlannedTraj)
            
            rospy.loginfo("| Move to Start Point | Count:%d --> %d |",COUNT1+1, 500)
            
            COUNT1 = COUNT1+1
            
            rate.sleep()
        else:
            break
    
    # ************************************************************ 按下按钮，开始DMP引导 *************************************************************
    M1 = 0
    Cal_Corr1 = True
    Cal_Cannio1 = True
    saveflag1 = True
    willflag1 = True
    ExecutedTraj = np.array([.0,.0,.0])
    
    while not rospy.is_shutdown():
        if buttonflag.Flag == 0:
            continue
        
        # --------- 按下按钮，开始DMP引导 ---------
        else:
            # ------------- （子空间1中一直保持记录）将已执行轨迹记录到一个数组中 ---------------
            ExecutedTraj = np.vstack((ExecutedTraj,curpose.pose)) # print(ExecutedTraj) # 3列
            
            # -----------------------------------------------------------------
            if len(ExecutedTraj) <= WINDOWS:
                continue
                
            else:
                cutExeTraj = ExecutedTraj[-WINDOWS:,:]
                
                # 计算信任度
                y_exec1, ExecutedTraj1, cut_off1 = Cutoff(y_plan1,cutExeTraj)
                y_exec2, ExecutedTraj2, cut_off2 = Cutoff(y_plan2,cutExeTraj)
                y_exec3, ExecutedTraj3, cut_off3 = Cutoff(y_plan3,cutExeTraj)
                if cut_off1 == 0: cut_off1 = 1
                if cut_off2 == 0: cut_off2 = 1
                if cut_off3 == 0: cut_off3 = 1

                # dtw.distance_fast; cosine_similarity
                distance1 = dtw.distance_fast(y_exec1[:,2], ExecutedTraj1[:,2])
                distance2 = dtw.distance_fast(y_exec2[:,2], ExecutedTraj2[:,2])
                distance3 = dtw.distance_fast(y_exec3[:,2], ExecutedTraj3[:,2])
                
                # 选中轨迹的概率
                probs = softmax(np.array([distance1,distance2,distance3]), np.min([distance1,distance2,distance3]),pubnu, Nu_val)
                #rospy.loginfo("distance 1-2-3: [%.4f - %.4f - %.4f]", distance1,distance2,distance3)
                #rospy.loginfo("probs 1-2-3: [%.4f - %.4f - %.4f]", probs[0], probs[1], probs[2])
                
                # 发布信任度
                Credit.header.stamp = rospy.Time.now()
                Credit.creditability_1 = probs[0]
                Credit.creditability_2 = probs[1]
                Credit.creditability_3 = probs[2]
                Credit.creditabilityMax = max(probs)
                
                if M1 == 0 and max(probs) < Lamba_f: M1 = 0
                elif (M1 == 0 and max(probs) >= Lamba_f) or (M1 == 1 and max(probs) >= Lamba_t):
                    M1 = 1
                    saveargmax1 = np.argmax(probs)
                elif M1 == 1 and max(probs) < Lamba_t and saveargmax1 == np.argmax(probs): M1 = 2
                elif M1 == 2 and max(probs) >= Lamba_t and saveargmax1 == np.argmax(probs): M1 = 1
                elif M1 == 2 and max(probs) < Lamba_t and saveargmax1 == np.argmax(probs): M1 = 2
                elif M1 == 2 and max(probs) < Lamba_t and saveargmax1 != np.argmax(probs): M1 = 3
                elif M1 == 3 and max(probs) < Lamba_t: M1 = 3
                elif M1 == 3 and max(probs) >= Lamba_f: M1 = 1
                Credit.M = M1
                pubcredit.publish(Credit)
                
                # 选择信任度最大的轨迹
                dic1 = {'y_plan1':probs[0], 'y_plan2':probs[1], 'y_plan3':probs[2]}
                y_max_prob_traj = max(dic1,key = lambda x:dic1[x]) # 找到概率最大的轨迹-字符串
                dic2 = {'DMP1':probs[0], 'DMP2':probs[1], 'DMP3':probs[2]}
                y_further_DMP = eval(sorted(dic2, key=dic2.get, reverse=True)[0]) # 找到概率最大的轨迹，获得其DMP对象
                
                # 输出概率次大的轨迹的数据到PlannedTrajWill的"/willtraj"话题
                will_traj = sorted(dic1, key=dic1.get, reverse=True)[1]
                y_will_DMP = eval(sorted(dic2, key=dic2.get, reverse=True)[1]) # 找到概率次大的轨迹，获得其DMP对象
                
                if willflag1 == True:
                    willfurther = will_traj
                    willflag1 = False
                
                if will_traj == willfurther:
                    if Cal_Cannio1 == True:
                        y_will_DMP.dmp.reset_state()
                        x_track_w1 = y_will_DMP.dmp.cs.rollout()
                        y_will_DMP.dmp.cs.reset_state()
                        
                        dic3 = {'y_plan1':cut_off1, 'y_plan2':cut_off2, 'y_plan3':cut_off3}
                        for i in range(0,dic3[will_traj]):
                            _, _, _, x_w1 = y_will_DMP.dmp.step(tau_d)
                        
                        dmpevol_will1 = DmpEvolu(tau_d)
                        x_w1, _, _ = dmpevol_will1.evolution(y_will_DMP,curpose,x_w1,x_track_w1,PlannedTrajWill,pubwilltraj,Devia,pubdevia,Tau_a_v,pubtau_a, ifpub = False)
                    
                        Cal_Cannio1 = False
                    else:
                        if x_w1 >= x_track_w1[-1]:
                            x_w1, _, _ = dmpevol_will1.evolution(y_will_DMP,curpose,x_w1,x_track_w1,PlannedTrajWill,pubwilltraj,Devia,pubdevia,Tau_a_v,pubtau_a, ifpub = False)
                else:
                    willflag1 = True
                    Cal_Cannio1 = True
                    
                if saveflag1 == True:
                    savefuther = y_max_prob_traj
                    saveflag1 = False
                
                if y_max_prob_traj == savefuther:
                    if Cal_Corr1 == True:
                        y_further_DMP.dmp.reset_state()
                        x_track = y_further_DMP.dmp.cs.rollout()
                        y_further_DMP.dmp.cs.reset_state()
                        
                        dic3 = {'y_plan1':cut_off1, 'y_plan2':cut_off2, 'y_plan3':cut_off3}
                        for i in range(0,dic3[y_max_prob_traj]):
                            # 为后边从cutoff点的DMP重放初始化（1）相位x,（2）self.y（y_further_DMP.dmp.y）和（3）self.dy（y_further_DMP.dmp.dy）
                            _, _, _, x1 = y_further_DMP.dmp.step(tau_d)
                        
                        dmpevol = DmpEvolu(tau_d)
                        x1, tau_a, Exedis1 = dmpevol.evolution(y_further_DMP,curpose,x1,x_track,PlannedTraj,pubforce,Devia,pubdevia,Tau_a_v,pubtau_a)
                        
                        rospy.loginfo("| probs 1-2-3: [%.3f - %.3f - %.3f] | Select traj [%s] | tau_a = [%.3f] | Phase(x1): [%.3f] --> %.3f |",
                           probs[0], probs[1], probs[2], y_max_prob_traj, tau_a, x1, x_track[-1])
                        
                        Cal_Corr1 = False
                        
                        rate.sleep()
                                        
                    else:
                        if x1 >= x_track[-1]:
                            x1, tau_a, Exedis1 = dmpevol.evolution(y_further_DMP,curpose,x1,x_track,PlannedTraj,pubforce,Devia,pubdevia,Tau_a_v,pubtau_a)
                            
                            # 发布正则系统的值
                            Canoni.header.stamp = rospy.Time.now()
                            Canoni.canonical = x1
                            pubcanonical.publish(Canoni)
                            
                            rospy.loginfo("| probs 1-2-3: [%.3f - %.3f - %.3f] | Select traj [%s] | tau_a = [%.3f] | Phase(x1): [%.3f] --> %.3f |",
                                probs[0], probs[1], probs[2], y_max_prob_traj, tau_a, x1, x_track[-1])
                            
                            rate.sleep()
                        else:
                            break
                else:
                    saveflag1 = True
                    Cal_Corr1 = True
                    

    # -------------------------------------- 开始子空间2的引导 --------------------------------------------------
    M2 = 0
    Cal_Corr2 = True
    Cal_Cannio2 = True
    saveflag2 = True
    willflag2 = True
    Flag_F = 0.05
    x2 = 1.0
    ExecutedTraj2 = np.array([.0,.0,.0])
    
    while not rospy.is_shutdown():
        if buttonflag.Flag == 0:
            continue
        
        # --------- 按下按钮，开始DMP引导 ---------
        else:
            # ------------- （子空间2中一直保持记录）将已执行轨迹记录到一个数组中 ---------------
            ExecutedTraj2 = np.vstack((ExecutedTraj2,curpose.pose)) # print(ExecutedTraj) # 3列
            
            # -----------------------------------------------------------------
            if len(ExecutedTraj2) <= WINDOWS:
                continue
                
            else:
                cutExeTraj2 = ExecutedTraj2[-WINDOWS:,:]
                
                # 计算信任度
                y_exec4, ExecutedTraj4, cut_off4 = Cutoff(y_plan4,cutExeTraj2)
                y_exec5, ExecutedTraj5, cut_off5 = Cutoff(y_plan5,cutExeTraj2)
                y_exec6, ExecutedTraj6, cut_off6 = Cutoff(y_plan6,cutExeTraj2)
                if cut_off4 == 0: cut_off4 = 1
                if cut_off5 == 0: cut_off5 = 1
                if cut_off6 == 0: cut_off6 = 1

                # dtw.distance_fast; cosine_similarity
                distance4 = dtw.distance_fast(y_exec4[:,2], ExecutedTraj4[:,2])
                distance5 = dtw.distance_fast(y_exec5[:,2], ExecutedTraj5[:,2])
                distance6 = dtw.distance_fast(y_exec6[:,2], ExecutedTraj6[:,2])
                
                # 选中轨迹的概率
                probs = softmax(np.array([distance4,distance5,distance6]), np.min([distance4,distance5,distance6]), pubnu, Nu_val)
                #rospy.loginfo("distance 4-5-6: [%.4f - %.4f - %.4f]", distance4, distance5, distance6)
                #rospy.loginfo("probs 4-5-6: [%.4f - %.4f - %.4f]", probs[0], probs[1], probs[2])
                
                # 发布信任度
                Credit.header.stamp = rospy.Time.now()
                Credit.creditability_1 = probs[0]
                Credit.creditability_2 = probs[1]
                Credit.creditability_3 = probs[2]
                Credit.creditabilityMax = max(probs)
                
                if x2 >= Flag_F:
                    print("True")
                    if M2 == 0 and max(probs) < Lamba_f: M2 = 0
                    elif (M2 == 0 and max(probs) >= Lamba_f) or (M2 == 1 and max(probs) >= Lamba_t):
                        M2 = 1
                        saveargmax1 = np.argmax(probs)
                    elif M2 == 1 and max(probs) < Lamba_t and saveargmax1 == np.argmax(probs): M2 = 2
                    elif M2 == 2 and max(probs) >= Lamba_t and saveargmax1 == np.argmax(probs): M2 = 1
                    elif M2 == 2 and max(probs) < Lamba_t and saveargmax1 == np.argmax(probs): M2 = 2
                    elif M2 == 2 and max(probs) < Lamba_t and saveargmax1 != np.argmax(probs): M2 = 3
                    elif M2 == 3 and max(probs) < Lamba_t: M2 = 3
                    elif M2 == 3 and max(probs) >= Lamba_f: M2 = 1
                    Credit.M = M2
                    pubcredit.publish(Credit)
                
                # 选择信任度最大的轨迹
                dic1 = {'y_plan4':probs[0], 'y_plan5':probs[1], 'y_plan6':probs[2]}
                y_max_prob_traj = max(dic1,key = lambda x:dic1[x]) # 找到概率最大的轨迹-字符串
                dic2 = {'DMP4':probs[0], 'DMP5':probs[1], 'DMP6':probs[2]}
                y_further_DMP = eval(sorted(dic2, key=dic2.get, reverse=True)[0]) # 找到概率最大的轨迹，获得其DMP对象
                
                # 输出概率次大的轨迹的数据到PlannedTrajWill的"/willtraj"话题
                will_traj = sorted(dic1, key=dic1.get, reverse=True)[1]
                y_will_DMP = eval(sorted(dic2, key=dic2.get, reverse=True)[1]) # 找到概率次大的轨迹，获得其DMP对象
                
                if willflag2 == True:
                    willfurther = will_traj
                    willflag2 = False
                
                if will_traj == willfurther:
                    if Cal_Cannio2 == True:
                        y_will_DMP.dmp.reset_state()
                        x_track_w2 = y_will_DMP.dmp.cs.rollout()
                        y_will_DMP.dmp.cs.reset_state()
                        
                        dic3 = {'y_plan4':cut_off4, 'y_plan5':cut_off5, 'y_plan6':cut_off6}
                        for i in range(0,dic3[will_traj]):
                            _, _, _, x_w2 = y_will_DMP.dmp.step(tau_d)
                        
                        dmpevol_will2 = DmpEvolu(tau_d)
                        x_w2, _, _ = dmpevol_will2.evolution(y_will_DMP,curpose,x_w2,x_track_w2,PlannedTrajWill,pubwilltraj,Devia,pubdevia,Tau_a_v,pubtau_a, ifpub = False)
                    
                        Cal_Cannio2 = False
                    else:
                        if x_w2 >= x_track_w2[-1]:
                            x_w2, _, _ = dmpevol_will2.evolution(y_will_DMP,curpose,x_w2,x_track_w2,PlannedTrajWill,pubwilltraj,Devia,pubdevia,Tau_a_v,pubtau_a, ifpub = False)
                else:
                    willflag2 = True
                    Cal_Cannio2 = True
                    
                if saveflag2 == True:
                    savefuther = y_max_prob_traj
                    saveflag2 = False
                
                if y_max_prob_traj == savefuther:
                    if Cal_Corr2 == True:
                        y_further_DMP.dmp.reset_state()
                        x_track = y_further_DMP.dmp.cs.rollout()
                        y_further_DMP.dmp.cs.reset_state()
                        
                        dic3 = {'y_plan4':cut_off4, 'y_plan5':cut_off5, 'y_plan6':cut_off6}
                        for i in range(0,dic3[y_max_prob_traj]):
                            # 为后边从cutoff点的DMP重放初始化（1）相位x,（2）self.y（y_further_DMP.dmp.y）和（3）self.dy（y_further_DMP.dmp.dy）
                            _, _, _, x2 = y_further_DMP.dmp.step(tau_d)
                        
                        dmpevol2 = DmpEvolu(tau_d)
                        x2, tau_a, Exedis2 = dmpevol.evolution(y_further_DMP,curpose,x2,x_track,PlannedTraj,pubforce,Devia,pubdevia,Tau_a_v,pubtau_a)
                        
                        rospy.loginfo("| probs 4-5-6: [%.3f - %.3f - %.3f] | Select traj [%s] | tau_a = [%.3f] | Phase(x2): [%.3f] --> %.3f |",
                           probs[0], probs[1], probs[2], y_max_prob_traj, tau_a, x2, x_track[-1])
                        
                        Cal_Corr2 = False
                        
                        rate.sleep()
                                        
                    else:
                        if x2 >= x_track[-1]:
                            x2, tau_a, Exedis2 = dmpevol2.evolution(y_further_DMP,curpose,x2,x_track,PlannedTraj,pubforce,Devia,pubdevia,Tau_a_v,pubtau_a)
                            
                            # 发布正则系统的值
                            Canoni.header.stamp = rospy.Time.now()
                            Canoni.canonical = x2
                            pubcanonical.publish(Canoni)
                            
                            rospy.loginfo("| probs 4-5-6: [%.3f - %.3f - %.3f] | Select traj [%s] | tau_a = [%.3f] | Phase(x2): [%.3f] --> %.3f |",
                               probs[0], probs[1], probs[2], y_max_prob_traj, tau_a, x2, x_track[-1])
                            
                            rate.sleep()
                        else:
                            rospy.logwarn("\nThe time of justment is THREE second!\n")
                            rospy.logwarn("Close the data subscription node! The image will be displayed soon!\n")
                            rospy.logwarn("Button again for go to start point!")
                            rospy.sleep(3.)
                            break
                else:
                    saveflag2 = True
                    Cal_Corr2 = True
                               

    # --------------------------------------- 回到起始位置 -----------------------------------------------------    
    COUNT3 = 0 
    rospy.loginfo("Go To Start Point...")
    
    Credit.header.stamp = rospy.Time.now()
    Credit.creditability_1 = 0
    Credit.creditability_2 = 0
    Credit.creditability_3 = 0
    Credit.creditabilityMax = 0
    Credit.M = 0
    pubcredit.publish(Credit)
    
    while not rospy.is_shutdown():
        if COUNT3 < 5*100:
            # 与MRA7A的RVIZ中Touch状态匹配的touch初始点，state->lock_pos = {-16.56,0,-12.51}; 
            PlannedTraj.header.stamp = rospy.Time.now()
            PlannedTraj.position.x = curpose.pose[0]*1000+(-16.56-curpose.pose[0]*1000)/500*COUNT3
            PlannedTraj.position.y = curpose.pose[1]*1000+(0-curpose.pose[1]*1000)/500*COUNT3
            PlannedTraj.position.z = curpose.pose[2]*1000+(-12.51-curpose.pose[2]*1000)/500*COUNT3
            
            pubforce.publish(PlannedTraj)
                        
            #rospy.loginfo("Go To Start Point... | Count:%d --> %d |", COUNT3+1, 500)
        
            COUNT3 = COUNT3+1
            
            rate.sleep()

        else:
            rospy.sleep(2.)
            rospy.signal_shutdown("the node of planned trajectory is shutdown.")