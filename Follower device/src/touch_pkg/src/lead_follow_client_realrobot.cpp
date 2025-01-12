#include <ros/ros.h>
#include <Eigen/Core> 
#include "mra_kinematics/GetDirectInverse.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include "mr_msgs/JointCommand.h"
#include "mr_msgs/GripperCommand.h"
#include "omni_msgs/OmniButtonEvent.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <fstream>
#include<cmath>

/* 主从操作机械臂步骤： 1.订阅/phantom/pose话题 2.空间映射 3.client求逆解 4.将逆解发布出去 */
/* 主从操作机械臂夹爪步骤： 1.订阅/phantom/button话题 2.将/phantom/button标志位决定的手爪抓或放的话题/mra7a/gripper_cmd发布出去 */


class Processor
{
    public:
    Eigen::Matrix<double,1,7> pose_follow; // 机械臂的位姿


    public:
    void Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        double pose_lead[7] = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w}; // 传进来的主手的位姿
        
        // 位置空间映射，姿态不变
        double Cartesian_x = 6*pose_lead[0] - 0.24;//整体空间12*pose_lead[0]+0.12;
        double Cartesian_y = 3.4*pose_lead[1];//6*pose_lead[1];
        double Cartesian_z = 3.4*pose_lead[2] + 0.34;//4*pose_lead[2]+0.4;
        //double Cartesian_ori_x = pose_lead[3]; double Cartesian_ori_y = pose_lead[4]; double Cartesian_ori_z = pose_lead[5]; double Cartesian_ori_w = pose_lead[6];
        double Cartesian_ori_x = 0; double Cartesian_ori_y = -0.706; double Cartesian_ori_z = 0; double Cartesian_ori_w = 0.707;
        pose_follow << Cartesian_x, Cartesian_y, Cartesian_z, Cartesian_ori_x, Cartesian_ori_y, Cartesian_ori_z, Cartesian_ori_w;
    };
};


class ButtonGrasp
{
    public:
    bool buttonFlag;

    public:
    void doButton(const omni_msgs::OmniButtonEvent::ConstPtr& buttonmsg)
    {
        if (buttonmsg->grey_button == 1)
        {   buttonFlag = 1;  }
        else
        {   buttonFlag = 0;  }
    };
};


class RecordData
{
    public:
    Eigen::Matrix<double,1,24> datalist;

    public:
    void Callback(const geometry_msgs::PoseStamped::ConstPtr& sub_touch, const sensor_msgs::JointState::ConstPtr& sub_mra7a)
    {
        datalist << sub_touch->pose.position.x, sub_touch->pose.position.y,sub_touch->pose.position.z,
                sub_mra7a->position[0], sub_mra7a->position[1], sub_mra7a->position[2], sub_mra7a->position[3], sub_mra7a->position[4], sub_mra7a->position[5], sub_mra7a->position[6], 
                sub_mra7a->velocity[0], sub_mra7a->velocity[1], sub_mra7a->velocity[2], sub_mra7a->velocity[3], sub_mra7a->velocity[4], sub_mra7a->velocity[5], sub_mra7a->velocity[6], 
                sub_mra7a->effort[0], sub_mra7a->effort[1], sub_mra7a->effort[2], sub_mra7a->effort[3], sub_mra7a->effort[4], sub_mra7a->effort[5], sub_mra7a->effort[6];
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lead_follow_client_realrobot");
    ros::NodeHandle node;

    /*------------ 控制夹爪 -------------*/
    ButtonGrasp buttongrasp;
    ros::Subscriber sub_button = node.subscribe<omni_msgs::OmniButtonEvent>("/phantom/button", 10, &ButtonGrasp::doButton, &buttongrasp);

    ros::Publisher pub_grasp = node.advertise<mr_msgs::GripperCommand>("/mra7a/gripper_cmd", 10);
    mr_msgs::GripperCommand grasp_inf; //消息声明
    grasp_inf.name = "Gripper1";
    
    /*----------- 控制机械臂 ------------*/
    Processor processor;
    ros::Subscriber sub = node.subscribe<geometry_msgs::PoseStamped>("/phantom/pose", 10, &Processor::Callback, &processor);

    // 逆解
    ros::ServiceClient client = node.serviceClient<mra_kinematics::GetDirectInverse>("direct_inverse");
    ros::service::waitForService("direct_inverse"); // 等待服务启动
    mra_kinematics::GetDirectInverse Endposatt; //定义一个服务消息
    
    Endposatt.request.init_joints.resize(7);
    Endposatt.request.pose.resize(7);

    // 发布新的轨迹，发布到/robot_joint_states话题
    ros::Publisher joint_state_pub = node.advertise<mr_msgs::JointCommand> ("mra7a/joint_cmd", 10);
    mr_msgs::JointCommand joint_cmd; //消息声明
    joint_cmd.names.resize(7); //定义7个运动节点
    joint_cmd.cmdPos.resize(7);
    joint_cmd.command = 3;
	joint_cmd.names[0] = "Joint1"; joint_cmd.names[1] = "Joint2"; joint_cmd.names[2] = "Joint3";
    joint_cmd.names[3] = "Joint4"; joint_cmd.names[4] = "Joint5"; joint_cmd.names[5] = "Joint6"; joint_cmd.names[6] = "Joint7";

    double sol_joint[7] = {0, 0, 0, 0, 0, 0, 0};
    float near_joints[7] = {0, 0, 0, 0, 0, 0, 0};

    /*------------ 记录Touch与Mra7a数据 -------------*/
    RecordData recordData;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_touch(node,"/phantom/pose", 1);
    message_filters::Subscriber<sensor_msgs::JointState> sub_mra7a(node,"/mra_joint_states", 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,sensor_msgs::JointState> mysync;
    message_filters::Synchronizer<mysync> sync(mysync(10),sub_touch,sub_mra7a);
    sync.registerCallback(boost::bind(&RecordData::Callback,&recordData,_1,_2));

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        /*-------- 发布夹爪控制话题 ----------*/
        if (buttongrasp.buttonFlag == 1)
        {
            grasp_inf.positionL = 30.0;
            grasp_inf.positionR = 30.0;
            

            /*------ 将数据写入文件 -------*/
            /*--- https://blog.csdn.net/qq_39400324/article/details/130488253 ---*/
            std::ofstream file; // 创建输出文件流
            file.open("./Touch_Mra7a_data_subscri.txt", std::ios::app);
            // 将矩阵以txt格式写入文件
            file << recordData.datalist.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", ""));
            file << '\n';
            file.close(); // 关闭文件流
            ROS_INFO("**********Writing*********Writing************Writing**********Writing");
        }
        else
        {
            grasp_inf.positionL = 0.0;
            grasp_inf.positionR = 0.0;
        }
        pub_grasp.publish(grasp_inf);

        /*--------- 发布机械臂控制话题 -------*/
        // ***客户端请求时发送的数据1：机械臂初始关节角***
        for(int i = 0; i < 7; i++){Endposatt.request.init_joints[i] = near_joints[i]/57.3; } 
        // ***客户端请求时发送的数据2：机械臂末端位姿***
        for(int i = 0; i < 7; i++){Endposatt.request.pose[i] = processor.pose_follow[i]; } 
        bool flag = client.call(Endposatt); // 发送请求，返回标记是否成功的bool值

        if (flag)
        {
            //ROS_INFO("Find the inverse solution");
            for(int i = 0; i < 7; i++){
                // ***服务器响应的数据***
                sol_joint[i] = Endposatt.response.targetJoints[i];

                //if(每个关节，两个距离判断，超过，arm.go)
                // 
                if (fabs(sol_joint[0])<=1.57*57.3 && sol_joint[1]<=0.2*57.3){ //fabs(sol_joint[i]-near_joints[i]) < 0.5

                    joint_cmd.cmdPos[i] = sol_joint[i]*57.3;
                    near_joints[i] = sol_joint[i]*57.3;
                    }
                else{
                    ROS_WARN_STREAM ("WARN: No suitable inverse solution");
                    joint_cmd.cmdPos[i] = near_joints[i]*57.3;
                }
             }
        }
        else
        {
            ROS_ERROR_STREAM ("WARN: Not Find Inverse Solution");
        }

        //ROS_INFO("[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",joint_cmd.cmdPos[0], joint_cmd.cmdPos[1],joint_cmd.cmdPos[2],joint_cmd.cmdPos[3],joint_cmd.cmdPos[4],joint_cmd.cmdPos[5],joint_cmd.cmdPos[6]);
        
        joint_state_pub.publish(joint_cmd);
        
        /*loop_rate default 100 Hz */
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
