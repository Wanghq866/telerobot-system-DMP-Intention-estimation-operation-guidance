#include <stdio.h>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "mra_kinematics/mra_kin.h"
#include <iostream>
//ros
#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>
using namespace std;
using namespace::mra_kinematics;
const double Limits[7][2] = { {-3.05, 3.05}, {-2.18, 2.18}, {-3.05, 3.05}, {-2.30, 2.30},
                            {-3.05, 3.05}, {-2.05, 2.05}, {-3.05, 3.05}};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse");
    ros::NodeHandle node;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("/Link0", "/grasp_frame", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/Link0", "/grasp_frame", ros::Time(0), transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return -1;
    };
    double q_x = transform.getRotation().getX();
    double q_y = transform.getRotation().getY();
    double q_z = transform.getRotation().getZ();
    double q_w = transform.getRotation().getW();
    KDL::Frame goal;
    KDL::Rotation rot;
    Eigen::Matrix4d tm1 = Eigen::Matrix4d::Identity();
    goal.M = rot.Quaternion(q_x, q_y, q_z, q_w);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            tm1(i, j) = goal.M.operator()(i,j);
    }
    tm1(0, 3) = transform.getOrigin().getX();
    tm1(1, 3) = transform.getOrigin().getY();
    tm1(2, 3) = transform.getOrigin().getZ();
    double R, P, Y;
    goal.M.GetRPY(R,P,Y);
    cout<<"X: "<<tm1(0, 3)<<"Y: "<<tm1(1, 3)<<"Z: "<<tm1(2, 3)<<endl;
    cout<<"R :"<<R<<"P: "<<P<<"Y: "<<Y<<endl;
    cout<<"tm1 is "<<endl;
    cout<<tm1<<endl;
    //获取Link7相对于Grasp的齐次变换矩阵
    Eigen::Matrix4d tm2;
    tm2 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -0.125,
            0, 0, 0, 1;

    Eigen::Matrix4d tm = tm1*tm2;  //获取Link7相对于Link0的相对刚体变换矩阵
    //tm1是Grasp相对于Link0的位姿，tm2是Link7相对于Link0的刚体变换矩阵
    cout<<"tm is "<<tm<<endl;
    //获取Grasp相对于Link0的刚体变换矩阵
double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
double Jnt[7] = {0, 0, 0, 0, 0, 0, 0};   //Jnt为初始关节角
double sols[8][7];  //最终求解的关节角度
double M_T[16];

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
            T[i][j] = tm(i,j);
    }

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
            cout<<T[i][j]<<" ";
        cout<<endl;
    }
    mra_kinematics::inverse(T, Jnt, (double *)sols);
    printf("以下为解:\n");
    for(int i = 0; i < 8; i++){
		for(int j = 0; j < 7; j++)
        	printf("%5.2f ", sols[i][j]);
    	printf("\n");
	}
    vector<vector<double>> valid_sols;
    for(int i = 0; i < 8; i++)
    {
        bool IsValid = true;
        vector<double> temp;
        for(int j = 0; j < 7; j++)
        {
            temp.push_back(sols[i][j]);
            if(sols[i][j] < Limits[j][0] || sols[i][j] > Limits[j][1])
            {
                IsValid = false;
                break;
            }
        }
        if(IsValid == true)
        {
            valid_sols.push_back(temp);
        }
    }
    printf("以下为可行解:\n");
    for(int i = 0; i < valid_sols.size(); i++){
		for(int j = 0; j < 7; j++)
        	cout<<valid_sols[i][j]<<", ";
    	cout<<endl;
	}
    for(int i = 0; i < 8; i++)
    {
        forward(sols[i], M_T);
        printf("以下为根据逆解回算的末端位置:\n");
        for(int i = 0; i < 16; i++)
        {
            cout<<M_T[i]<<" ";
            if((i+1) % 4 == 0)
                cout<<endl;
        }
    }

    return 0;
}
