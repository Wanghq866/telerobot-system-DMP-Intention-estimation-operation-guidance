#include <stdio.h>
#include <stdlib.h>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "mra_kinematics/mra_kin.h"
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>
#include "mra_kinematics/GetDirectInverse.h"
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace mra_kinematics;

// Joint limit
const double Limits[7][2] = { {-3.05, 3.05}, {-2.18, 2.18}, {-3.05, 3.05}, {-2.30, 2.30}, {-3.05, 3.05}, {-2.05, 2.05}, {-3.05, 3.05}};
const double Weights[7] = {1.0, 1.0, 0.5, 0.5, 0.3, 0.1, 0.1};
bool isCalled = false;

void validJudge(double (*sols)[7], vector<vector<double>> & valid_sols){
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
}

void correctJudge(double (*T)[4], vector<vector<double>>& valid_sols){
    double M_T[4][4];
    double sol[7];
    vector<vector<double>> temp;
    for(int i = 0; i < valid_sols.size(); i++){
        for(int j = 0; j < 7; j++){
            sol[j] = valid_sols[i][j];
        }
        forward(sol, (double *)M_T);
        if(M_T[0][3] - T[0][3] <= 0.01 && M_T[1][3] - T[1][3] <= 0.01 && M_T[2][3] - T[2][3] <= 0.01){
            temp.push_back(valid_sols[i]);
        }
    }
    valid_sols = temp;
}

int solsOptimize(vector<vector<double>>& valid_sols, double * Jnt){
    std::vector<pair<int, double>> weighted_diffs;
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
    int select_idx = -1;
    if(!weighted_diffs.empty()){
        select_idx = weighted_diffs[0].first;
    }
    return select_idx;
}


void showSolutions(double (*sols)[7]){
    printf("All inverse solutions：\n");
    for(int i = 0; i < 8; i++){
		for(int j = 0; j < 7; j++)
        	printf("%5.2f ", sols[i][j]);
    	printf("\n");
	}
}
void showSolutions(vector<vector<double>>& sols){
    printf("Feasible solution:\n");
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


// The position of the seventh joint relative to the base coordinate system of the robot itself
bool getInverse(mra_kinematics::GetDirectInverse::Request & req, mra_kinematics::GetDirectInverse::Response & res){
    double Jnt[7] = {0, 0, 0, 0, 0, 0, 0}; // Jnt is the initial joint angle
    double sols[8][7];  // The final solved joint angle
    double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    vector<vector<double>> valid_sols;
    cout<<"init_joints: \n";
    for(int i = 0; i < 7; i++){
       Jnt[i] = req.init_joints[i]; // req.init_joints is the initial joint angle passed in
       cout<<Jnt[i]<<" ";
    }
    cout<<endl;

    // req.pose is the target position passed in,[0]-[2],x,y,z; [3]-[6] represents the quaternion of the attitude***
    KDL::Frame goal;
    KDL::Rotation rot;
	goal.M = rot.Quaternion(req.pose[3], req.pose[4], req.pose[5], req.pose[6]);
    Eigen::Matrix4d T07 = Eigen::Matrix4d::Identity(); // 7 represents Link7, 0 represents Link0

    T07(0, 3) = req.pose[0];    
	T07(1, 3) = req.pose[1];
	T07(2, 3) = req.pose[2];
    for(int i = 0; i < 3; i++)  // The goal is to obtain the pose T07 of Link7 relative to Link0
    {
        for(int j = 0; j < 3; j++)
            T07(i, j) = goal.M.operator()(i,j);
    } 

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            T[i][j] = T07(i, j);
        }
    }

    // Obtain all inverse solutions of the target pose, 8 sets of solutions
    mra_kinematics::inverse(T, Jnt, (double *)sols);
    showSolutions(sols);
    // Select feasible solutions from all inverse solutions and screen based on joint limits
    validJudge(sols, valid_sols);
    // Screen out all the correct solutions from the feasible ones, and then solve them again based on forward kinematics to verify
    correctJudge(T, valid_sols);
    if(valid_sols.empty()){
        ROS_ERROR("can't get valid IK solution!!!");
        return false;
    }
    showSolutions(valid_sols);
    // Select a target joint angle that is closest to the initial joint angle based on weighted distance
    int select_idx = solsOptimize(valid_sols, Jnt); 
    // Server response data
    res.targetJoints = valid_sols[select_idx]; 
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "direct_inverse");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("direct_inverse", getInverse);
    ROS_INFO("Ready to get inverse.");
    ros::spin();
    return 0;
}
