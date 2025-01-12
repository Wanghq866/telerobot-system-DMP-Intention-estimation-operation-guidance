#include <stdio.h>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "mra_kinematics/mra_kin.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <string>
using namespace std;
using namespace::mra_kinematics;
const double Limits[7][2] = { {-3.05, 3.05}, {-2.18, 2.18}, {-3.05, 3.05}, {-2.30, 2.30},
                            {-3.05, 3.05}, {-2.05, 2.05}, {-3.05, 3.05}};
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

void showMat(double (*M_T)[4]){
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
        cout<<M_T[i][j]<<" ";
    }
    cout<<endl;
  }
}

void getMaxMin(double (*ws)[3], int N, double Max[3], double Min[3]){
    for(unsigned int i = 0; i < N; ++i){
        for(int j = 0; j < 3; ++j){
          if(ws[i][j] > Max[j]){
            Max[j] = ws[i][j];
          }
          if(ws[i][j] < Min[j]){
            Min[j] = ws[i][j];
          }
        }
    }
}

int getPws(double (*Lws)[3], double (*Rws)[3], int N, double (*Pws)[3], double Min[3]){
    double gridSize = 0.01;
    int Lws_ids[3], Rws_ids[3], size = 0;
    unordered_map<string, vector<int>> table;
    for(int i = 0; i < N; ++i){
      Lws_ids[0] = int((Lws[i][0]-Min[0])/gridSize );
      Lws_ids[1] = int((Lws[i][1]-Min[1])/gridSize );
      Lws_ids[2] = int((Lws[i][2]-Min[2])/gridSize );
      cout<<Lws_ids[0]<<" "<<Lws_ids[1]<<" "<<Lws_ids[2]<<endl;
      stringstream ss;
      ss << Lws_ids[0]<<"-"<<Lws_ids[1]<<"-"<<Lws_ids[2];
      cout<<ss.str()<<endl;
      table[ss.str()].push_back(i);
    }
    
    unordered_set<string> m_set;
    for(int i = 0; i < N; ++i){
      Rws_ids[0] = int((Rws[i][0]-Min[0])/gridSize );
      Rws_ids[1] = int((Rws[i][1]-Min[1])/gridSize );
      Rws_ids[2] = int((Rws[i][2]-Min[2])/gridSize );
      stringstream ss;
      ss << Rws_ids[0]<<"-"<<Rws_ids[1]<<"-"<<Rws_ids[2];
      if(table.find(ss.str()) != table.end()){
          if(m_set.find(ss.str()) == m_set.end()){
              m_set.insert(ss.str());
          }
          Pws[size][0] = Rws[i][0];
          Pws[size][1] = Rws[i][1];
          Pws[size][2] = Rws[i][2];
          size++;
      }
    }
    for(auto iter = m_set.begin(); iter != m_set.end(); iter++){
       auto Ids = table[*iter];
       for(int i = 0; i < Ids.size(); ++i){
          int idx = Ids[i];
          cout<<"size: "<<size<<endl;
          Pws[size][0] = Lws[idx][0];
          Pws[size][1] = Lws[idx][1];
          Pws[size][2] = Lws[idx][2];
          size++;
       }
    }
    return 0;
}

int main()
{

  const int N = 400000;
  double (*Lws)[3] =  new double[N][3], (*Rws)[3] = new double[N][3];
  // double Lws[N][3], Rws[N][3];
  double Max[3] = {-10.0, -10.0, -10.0}, Min[3] = {10.0, 10.0, 10.0};
  double joints[7];
  double M_T[4][4], T[4][4];
  Eigen::Matrix4d T0_7;
  Eigen::Matrix4d T7_e = Eigen::Matrix4d::Identity(); T7_e(2,3) = 0.125;
  Eigen::Matrix4d Tw_lb, Tw_rb;
  Tw_lb << -1, 0, 0, 0,0, -0.50854644, -0.86103456, -0.025,0, -0.86103456,  0.50854644,  0.94, 0, 0, 0, 1;
  cout<<Tw_lb<<endl;
  Tw_rb << 1, 0, 0, 0,0, 0.50854644, 0.86103456, 0.025,0, -0.86103456,  0.50854644,  0.94, 0, 0, 0, 1;
  cout<<Tw_rb<<endl;
  Eigen::Matrix4d T0_e, Tw_le, Tw_re;
  ofstream LwsFile, RwsFile;
  // LwsFile.open("left_workspace.txt");
  // RwsFile.open("right_workspace.txt");
  ofstream PwsFile;
  PwsFile.open("public_workspace.txt");

  //Link7相对于Base_link的位置
  for(int i = 0; i < N; i++){
    for(int i = 0; i < 7; i++){
      double rnd = rand() / (RAND_MAX+0.0001);
      joints[i] = Limits[i][0] + (Limits[i][1]-Limits[i][0])*rnd;
    }
    forward(joints, (double *)M_T);
    // showMat(M_T);
    ArrayToMatrix4d(M_T, T0_7);
    // cout<<T0_7<<endl;
    T0_e = T0_7 * T7_e;
    Tw_le = Tw_lb * T0_e;
    Tw_re = Tw_rb * T0_e;
    
    Lws[i][0] = Tw_le(0, 3);
    Lws[i][1] = Tw_le(1, 3);
    Lws[i][2] = Tw_le(2, 3);
    // LwsFile <<Lws[i][0]<<" "<<Lws[i][1]<<" "<<Lws[i][2]<<endl;

    Rws[i][0] = Tw_re(0, 3);
    Rws[i][1] = Tw_re(1, 3);
    Rws[i][2] = Tw_re(2, 3);
    // RwsFile <<Rws[i][0]<<" "<<Rws[i][1]<<" "<<Rws[i][2]<<endl;
  }

  getMaxMin(Lws, N, Max, Min);
  getMaxMin(Rws, N, Max, Min);
  cout<<"Max: "<<endl;
  for(int i = 0; i < 3; i++){
    cout<<Max[i]<<" ";
  }
  cout<<endl;
  cout<<"Min: "<<endl;
  for(int i = 0; i < 3; i++){
    cout<<Min[i]<<" ";
  }
  cout<<endl;

  // LwsFile.close();
  // RwsFile.close();

  double Pws[2*N][3];
  double gridSize = 0.02;
  int Lws_ids[3], Rws_ids[3], size = 0;
  unordered_map<string, vector<int>> table;

  for(int i = 0; i < N; ++i){
    Lws_ids[0] = int((Lws[i][0]-Min[0])/gridSize );
    Lws_ids[1] = int((Lws[i][1]-Min[1])/gridSize );
    Lws_ids[2] = int((Lws[i][2]-Min[2])/gridSize );
    stringstream ss;
    ss << Lws_ids[0]<<"-"<<Lws_ids[1]<<"-"<<Lws_ids[2];
    table[ss.str()].push_back(i);
  }

  unordered_set<string> m_set;
  for(int i = 0; i < N; ++i){
    Rws_ids[0] = int((Rws[i][0]-Min[0])/gridSize );
    Rws_ids[1] = int((Rws[i][1]-Min[1])/gridSize );
    Rws_ids[2] = int((Rws[i][2]-Min[2])/gridSize );
    stringstream ss;
    ss << Rws_ids[0]<<"-"<<Rws_ids[1]<<"-"<<Rws_ids[2];
    if(table.find(ss.str()) != table.end() && Rws[i][0] <= 0 && Rws[i][2] <= 1.0){
        if(m_set.find(ss.str()) == m_set.end()){
            m_set.insert(ss.str());
        }
        Pws[size][0] = Rws[i][0];
        Pws[size][1] = Rws[i][1];
        Pws[size][2] = Rws[i][2];
        PwsFile <<Rws[i][0]<<" "<<Rws[i][1]<<" "<<Rws[i][2]<<endl;
        size++;
    }
  }

  for(auto iter = m_set.begin(); iter != m_set.end(); iter++){
    auto Ids = table[*iter];
    for(int i = 0; i < Ids.size(); ++i){
      int idx = Ids[i];
      if (Lws[idx][0] <= 0 && Lws[idx][2] <= 1.0){
        Pws[size][0] = Lws[idx][0];
        Pws[size][1] = Lws[idx][1];
        Pws[size][2] = Lws[idx][2];
        PwsFile <<Lws[idx][0]<<" "<<Lws[idx][1]<<" "<<Lws[idx][2]<<endl;
        size++;
      }
    }
  }
  PwsFile.close();
  
  return 0;
}
