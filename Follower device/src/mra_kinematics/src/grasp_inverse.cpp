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
#include "mra_kinematics/GetInverse.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace mra_kinematics;
#define PI  3.1415926535
double startJoints[7];
const double Limits[7][2] = { {-3.05, 3.05}, {-2.18, 2.18}, {-3.05, 3.05}, 
                              {-2.30, 2.30},{-3.05, 3.05}, {-2.05, 2.05}, {-3.05, 3.05}};
const double Weights[7] = {1.0, 0.3, 0.2, 0.2, 0.1, 0.1, 0.1};

void Matrix4dToArray(Eigen::Matrix4d & Tm, double (*T)[4]){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            T[i][j] = Tm(i, j);
        }
    }
}
void ArrayToMatrix4d(double (*T)[4], Eigen::Matrix4d & Tm){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            Tm(i, j) = T[i][j];
        }
    }
}
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
void showValidSolutions(vector<vector<double>>& sols){
    printf("可行解:\n");
    for(int i = 0; i < sols.size(); i++){
		for(int j = 0; j < 7; j++)
        	cout<<sols[i][j]<<", ";
    	cout<<endl;
	}
}
bool getTargetPosition(Eigen::Matrix4d & T08){ 
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("/LeftBase", "/grasp_frame", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/LeftBase", "/grasp_frame", ros::Time(0), transform);
        cout<<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<<" "<<transform.getOrigin().getZ()<<endl;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    };
    KDL::Frame goal;
    KDL::Rotation rot;
    double q_x = transform.getRotation().getX();
    double q_y = transform.getRotation().getY();
    double q_z = transform.getRotation().getZ();
    double q_w = transform.getRotation().getW();
	goal.M = rot.Quaternion(q_x, q_y, q_z, q_w);
    T08(0, 3) = transform.getOrigin().getX();    
	T08(1, 3) = transform.getOrigin().getY();
	T08(2, 3) = transform.getOrigin().getZ();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            T08(i, j) = goal.M.operator()(i,j);
    }
    double R, P, Y;
    goal.M.GetRPY(R,P,Y);
    ROS_INFO("/grasp_frame -> Link0 (xyz|rpy):\n");
    cout<<"X: "<<T08(0, 3)<<" Y: "<<T08(1, 3)<<" Z: "<<T08(2, 3)<<endl;
    cout<<"R :"<<R<<" P: "<<P<<" Y: "<<Y<<endl;
    ROS_INFO("/grasp_frame -> Link0 (matrix):\n");
    cout<<T08<<endl;
    return true;
}

bool poseSample(Eigen::Matrix4d & T08, double * Jnt, vector<vector<double>> &valid_sols){
    double leftEdge = -PI/2, rightEdge = 0;
    int N = 100;
    double interval = (rightEdge - leftEdge) / (N - 1);
    vector<double> Angles(N, 0);
    for(int i = 0; i < N; i++){
        Angles[i] = leftEdge + i*interval;
    }
    for(int i = 0; i < N; i++){
        Eigen::AngleAxisd rotVec (Angles[i], Eigen::Vector3d(1, 0, 0)); // 绕x轴旋转弧度
        Eigen::Matrix4d T88_ = Eigen::Matrix4d::Identity();  //8_表示末端旋转之后的位姿
        T88_.block<3, 3>(0, 0) = rotVec.matrix();
        Eigen::Matrix4d T08_ = T08 * T88_;
        cout<<"T08_ "<<T08_<<endl;
        double sols[8][7];
        double T[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
        Eigen::Matrix4d T8_7;
        T8_7 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -0.125,
            0, 0, 0, 1;
        Eigen::Matrix4d T07 = T08_ * T8_7;
        cout<<"T07 "<<T07<<endl;
        Matrix4dToArray(T07, T);
        mra_kinematics::inverse(T, Jnt, (double *)sols);
        validJudge(sols, T, valid_sols);   //将所有符合关节角限制的逆解均加入到valid_sols中
    }
    if(valid_sols.empty()){
        ROS_ERROR("all IK solutions are over joint limits or not correct!!!");
        return false;
    }
    solsOptimize(valid_sols, Jnt);
    return true;
}

bool getInverse(mra_kinematics::GetInverse::Request & req, mra_kinematics::GetInverse::Response & res){
    double Jnt[7] = {0, 0, 0, 0, 0, 0, 0}; //Jnt为初始关节角
    double sols[8][7];  //最终求解的关节角度
    for(int i = 0; i < 7; i++){
       Jnt[i] = req.startJoints[i]; 
    }
    // 打印初始关节角：
    ROS_INFO("req.startJoints:\n");
    for(int i = 0; i < 7; i++){
        ROS_INFO("%lf ", Jnt[i]);  
    }
    ROS_INFO("\n");
    // 获取末端执行器相对于Link0的位姿T08:
    Eigen::Matrix4d T08 = Eigen::Matrix4d::Identity();
    if(getTargetPosition(T08) != true){
        ROS_ERROR("get target pose error!!!!");
        return false;
    }
    vector<vector<double>> valid_sols;
    if(poseSample(T08, Jnt, valid_sols) != true){
        ROS_ERROR("can't get valid IK solution!!!");
        return false;
    }
    showValidSolutions(valid_sols);
    res.targetJoints = valid_sols[0];
    
    double M_T[4][4];
    double sol[7];
    for(int i = 0; i < 7; i++){
        sol[i] = valid_sols[0][i];
    }
    forward(sol, (double *)M_T);
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            cout<<M_T[i][j]<<" ";
        }
        cout<<endl;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_inverse_left");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("get_inverse", getInverse);
    ROS_INFO("Ready to get inverse. ");
    ros::spin();
    return 0;
}
