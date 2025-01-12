#ifndef MRA_KIN_H
#define MRA_KIN_H

// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

namespace mra_kinematics {
  // @param q       The 6 joint values //6个关节值
  // @param T       The 4x4 end effector pose in row-major ordering //行优先顺序，只给出最后一个坐标系
  void forward(const double* q, double* T);

/*   // @param q       The 6 joint values 
  // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
  //把每一个连杆坐标系都给出来
  void forward_all(const double* q, double* T1, double* T2, double* T3, 
                                    double* T4, double* T5, double* T6);
 */
  // @param T       The 4x4 end effector pose in row-major ordering
  // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
  // @param q6_des  An optional parameter which designates what the q6 value should take
  //                in case of an infinite solution on that joint.
  // @return        Number of solutions found (maximum of 8)
  int inverse(const double T[][4], double* joint, double* joint_sols);
};

#endif //MRA_KIN_H
