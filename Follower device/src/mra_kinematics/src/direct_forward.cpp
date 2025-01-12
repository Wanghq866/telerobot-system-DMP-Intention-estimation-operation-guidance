#include <stdio.h>
#include <stdlib.h>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "mra_kinematics/mra_kin.h"
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <tf/transform_listener.h>
#include "mra_kinematics/GetInverseNamed.h"
#include "mra_kinematics/Getforward.h"
#include <moveit_msgs/GetPositionFK.h>
#include <sensor_msgs/JointState.h>
using namespace std;
using namespace mra_kinematics;
double startJoints[7];
const double Limits[7][2] = { {-3.05, 3.05}, {-2.18, 2.18}, {-3.05, 3.05}, {-2.30, 2.30}, {-3.05, 3.05}, {-2.05, 2.05}, {-3.05, 3.05}};
const double Weights[7] = {1.0, 0.3, 0.2, 0.2, 0.1, 0.1, 0.1};
bool isCalled = false;

void validJudge(double (*sols)[7], double (*T)[4], vector<vector<double>> & valid_sols){
    double M_T[4][4];  //由逆解计算出来的位姿
    double sol[7];
    for(int i = 0; i < 8; i++)
    {
        bool IsValid = true;
        for(int j = 0; j < 7; j++){
            sol[j] = sols[i][j];
        }
        for(int j = 0; j < 7; j++){
            if(sol[j] < Limits[j][0] || sol[j] > Limits[j][1]){
                IsValid = false;
                break;
            }
        }
        if(IsValid == true){
            forward(sol, (double *)M_T);
            cout<<"sol: "<<endl;
            for(int i = 0; i < 7; i++){
                cout<<sol[i]<<" ";
            }
            cout<<endl;
            cout<<"M_T: "<<endl;
            for(int i = 0; i < 4; i++){
                for(int j = 0; j < 4; j++){
                    cout<<M_T[i][j]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;
            if(fabs(M_T[0][3] - T[0][3]) > 0.001 || fabs(M_T[1][3] - T[1][3]) > 0.001 || fabs(M_T[2][3] - T[2][3]) > 0.001){
                IsValid = false;
            }

        }
        if(IsValid == true)
        {
            valid_sols.push_back(vector<double>(sol, sol+7));
        }
    }
}

void solsOptimize(vector<vector<double>>& valid_sols, double * Jnt){
    std::vector<pair<int, double>> weighted_diffs;	//idx_double是pair(关节i,权值)
    for(int i = 0; i < valid_sols.size(); i++) {
        double cur_weighted_diff = 0;
        for(int j = 0; j < 7; j++) {
            double abs_diff = std::fabs(Jnt[j] - valid_sols[i][j]);
            cur_weighted_diff += Weights[j] * abs_diff;
        }
        weighted_diffs.push_back(pair<int, double>(i, cur_weighted_diff));
    }
    std::sort(weighted_diffs.begin(), weighted_diffs.end(), 
            [](const pair<int, double>& l, const pair<int, double>& r){return l.second < r.second;});
    vector<vector<double>>  temp;
    for(auto iter = weighted_diffs.begin(); iter != weighted_diffs.end(); iter++){
        temp.push_back(valid_sols[iter->first]);
    }
    valid_sols = temp;
}

void showSolutions(double (*sols)[7]){
    printf("全部逆解：\n");
    for(int i = 0; i < 8; i++){
		for(int j = 0; j < 7; j++)
        	printf("%5.2f ", sols[i][j]);
    	printf("\n");
	}
}
void showSolutions(vector<vector<double>>& sols){
    printf("可行解:\n");
    for(int i = 0; i < sols.size(); i++){
		for(int j = 0; j < 7; j++)
        	cout<<sols[i][j]<<", ";
    	cout<<endl;
	}
}
void showPosition(double * T){
    for(int i = 0; i < 16; i++)
    {
        cout<<T[i]<<" ";
        if((i+1) % 4 == 0)
            cout<<endl;
    }
}

bool getTargetPosition(double (*T)[4], string frame_name){
    tf::TransformListener listener; 
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("/RightBase", frame_name, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/RightBase", frame_name, ros::Time(0), transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return -1;
    };
    KDL::Frame goal;
    KDL::Rotation rot;
    double q_x = transform.getRotation().getX();
    double q_y = transform.getRotation().getY();
    double q_z = transform.getRotation().getZ();
    double q_w = transform.getRotation().getW();
	goal.M = rot.Quaternion(q_x, q_y, q_z, q_w);
    Eigen::Matrix4d T08 = Eigen::Matrix4d::Identity(), T07;   //8表示grasp_frame, 7表示Link7,0表示Link0
    Eigen::Matrix4d T87 = Eigen::Matrix4d::Identity();
    T08(0, 3) = transform.getOrigin().getX();    
	T08(1, 3) = transform.getOrigin().getY();
	T08(2, 3) = transform.getOrigin().getZ();
    for(int i = 0; i < 3; i++)  // 获取vpServo_frame相对于Link0的位姿T08
    {
        for(int j = 0; j < 3; j++)
            T08(i, j) = goal.M.operator()(i,j);
    }
    T87 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, -0.125,
           0, 0, 0, 1;
    T07 = T08 * T87;  //目标是获取Link7相对于Link0的位姿T07
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            T[i][j] = T07(i, j);
        }
    }
    double R, P, Y;
    goal.M.GetRPY(R,P,Y);
    cout<<"目标位姿(xyz|rpy)："<<endl;
    cout<<"X: "<<T[0][3]<<" Y: "<<T[1][3]<<" Z: "<<T[2][3]<<endl;
    cout<<"R :"<<R<<" P: "<<P<<" Y: "<<Y<<endl;
    cout<<"目标位姿(matrix)："<<endl;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
            cout<<T[i][j]<<" ";
        cout<<endl;
    }
    return true;
}

bool get_forward(mra_kinematics::Getforward::Request & req, mra_kinematics::Getforward::Response & res){
    cout<<req.Joints.size()<<endl;
    double Jnt[7] = {0, 0, 0, 0, 0, 0, 0}; //Jnt为关节角
    double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    for(int i = 0; i < 7; i++){
       Jnt[i] = req.Joints[i];
    }
    forward(Jnt, (double *)T);
    showPosition((double *)T);
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            res.pose.push_back(T[i][j]);
        }
    }
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "m_forward");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("right_forward", get_forward);
    ROS_INFO("Ready to get right forward.");
    ros::spin();
    return 0;
}
