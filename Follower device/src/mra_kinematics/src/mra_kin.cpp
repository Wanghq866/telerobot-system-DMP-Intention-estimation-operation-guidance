#include <mra_kinematics/mra_kin.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

namespace mra_kinematics {

  namespace {
    /* const double ZERO_THRESH = 0.00000001;
    int SIGN(double x) {
      return (x > 0) - (x < 0);
    } */
    const double PI = M_PI;

	const double EPS = 1e-6;
	const double ZER = 1e-12;

	//DH parameters
	const double Dbs = 0.234;	//根据urdf模型求得的
	const double Dse = 0.292;
	const double Dew = 0.242;
	const double Dwt = 0.1071;

	//joint limits
	const double theta1_u = 180 / 180.0 * PI;
	const double theta1_l = -180 / 180.0 * PI;
	const double theta2_u = 135 / 180.0 * PI;
	const double theta2_l = -135 / 180.0 * PI;
	const double theta3_u = 180 / 180.0 * PI;
	const double theta3_l = -180 / 180.0 * PI;
	const double theta4_u = 135 / 180.0 * PI;
	const double theta4_l = -135 / 180.0 * PI;
	const double theta5_u = 180 / 180.0 * PI;
	const double theta5_l = -180 / 180.0 * PI;
	const double theta6_u = 135 / 180.0 * PI;
	const double theta6_l = -135 / 180.0 * PI;
	const double theta7_u = 180 / 180.0 * PI;
	const double theta7_l = -180 / 180.0 * PI;
	 
	//ψ region of joint_2 or joint_6
	void region_cos_type(double a, double b, double c,
						 double joint_u, double joint_l, double* region)
	{
		double joint_ul;
		if (joint_u > 0 && joint_l < 0)
		{
			joint_ul = fabs(joint_u) < fabs(joint_l) ? fabs(joint_u) : fabs(joint_l);
		}
		else
			return; //joint region error


		double cos_theta;
		cos_theta = cos(joint_ul);

		double psi_1, psi_2;
		double delta = pow(a, 2) + pow(b, 2) - pow(c - cos_theta, 2);

		if (delta < 0 || fabs(b - c + cos_theta) < ZER)
		{
			region[0] = 1;
			region[1] = -PI;
			region[2] = PI;
		}
		else
		{
			psi_1 = 2 * atan((a - sqrt(delta)) / (b - c + cos_theta));
			psi_2 = 2 * atan((a + sqrt(delta)) / (b - c + cos_theta));
			double psi_u, psi_l;
			psi_u = psi_1 > psi_2 ? psi_1 : psi_2;
			psi_l = psi_1 < psi_2 ? psi_1 : psi_2;

			double psi_mid = (psi_l + psi_u) / 2;
			double theta_at_mid = acos(a*sin(psi_mid) + b*cos(psi_mid) + c);
			if (theta_at_mid < joint_u)
				region[0] = 1;
			else
				region[0] = 2;

			region[1] = psi_l;
			region[2] = psi_u;
		}
	}

	//get the union of region_1 and region_2
	void region_union(const double* region_1, const double* region_2, double* region)
	{
		if (fabs(region_1[0] - 1) < ZER && fabs(region_2[0] - 1)<ZER)
		{
			region[0] = 1;
			region[1] = region_1[1] > region_2[1] ? region_1[1] : region_2[1];
			region[2] = region_1[2] < region_2[2] ? region_1[2] : region_2[2];
		}
		else if (fabs(region_1[0] - 2) < ZER && fabs(region_2[0] - 1) < ZER)
		{
			if ((region_2[2]<region_1[1]) || (region_2[1]>region_1[2]))
			{
				region[0] = 1;
				region[1] = region_2[1];
				region[2] = region_2[2];
			}
			else if ((region_2[1]<region_1[1]) && (region_2[2]<region_1[2]))
			{
				region[0] = 1;
				region[1] = region_2[1];
				region[2] = region_1[1];
			}
			else if ((region_2[1] > region_1[1]) && (region_2[2] > region_1[2]))
			{
				region[0] = 1;
				region[1] = region_1[2];
				region[2] = region_2[2];
			}
			else if ((region_2[1] < region_1[1]) && (region_2[2] > region_1[2]))
			{
				region[0] = 2;
				region[1] = region_2[1];
				region[2] = region_1[1];
				region[3] = region_2[2];
				region[4] = region_1[2];
			}
			else region[0] = 3;
		}
		else if (fabs(region_1[0] - 1) < ZER && fabs(region_2[0] - 2) < ZER)
		{
			if ((region_1[2]<region_2[1]) || (region_1[1]>region_2[2]))
			{
				region[0] = 1;
				region[1] = region_1[1];
				region[2] = region_1[2];
			}
			else if ((region_1[1]<region_2[1]) && (region_1[2]<region_2[2]))
			{
				region[0] = 1;
				region[1] = region_1[1];
				region[2] = region_2[1];
			}
			else if ((region_1[1] > region_2[1]) && (region_1[2] > region_2[2]))
			{
				region[0] = 1;
				region[1] = region_2[2];
				region[2] = region_1[2];
			}
			else if ((region_1[1] < region_2[1]) && (region_1[2] > region_2[2]))
			{
				region[0] = 2;
				region[1] = region_1[1];
				region[2] = region_2[1];
				region[3] = region_1[2];
				region[4] = region_2[2];
			}
			else region[0] = 3;
		}
	}

	//rotate of shoulder
	//shoulder[3]是肩关节的3个转角参数,产生一个选择矩阵为R[3][3]
	void Rot_shoulder(const double shoulder[3], double R[3][3])
	{
		R[0][0] = cos(shoulder[0])*cos(shoulder[1])*cos(shoulder[2]) - sin(shoulder[0])*sin(shoulder[2]);
		R[0][1] = -cos(shoulder[0])*sin(shoulder[1]);
		R[0][2] = -cos(shoulder[2])*sin(shoulder[0]) - cos(shoulder[0])*cos(shoulder[1])*sin(shoulder[2]);

		R[1][0] = cos(shoulder[0])*sin(shoulder[2]) + cos(shoulder[1])*cos(shoulder[2])*sin(shoulder[0]);
		R[1][1] = -sin(shoulder[0])*sin(shoulder[1]);
		R[1][2] = cos(shoulder[0])*cos(shoulder[2]) - cos(shoulder[1])*sin(shoulder[0])*sin(shoulder[2]);

		R[2][0] = -cos(shoulder[2])*sin(shoulder[1]);
		R[2][1] = -cos(shoulder[1]);
		R[2][2] = sin(shoulder[1])*sin(shoulder[2]);
	}

	//rotate of elbow
	void Rot_elbow(double elbow, double R[3][3])
	{
		R[0][0] = cos(elbow);	R[0][1] = 0;	R[0][2] = sin(elbow);
		R[1][0] = sin(elbow);	R[1][1] = 0;	R[1][2] = -cos(elbow);
		R[2][0] = 0;			R[2][1] = 1;	R[2][2] = 0;
	}

	//rotate of wrist
	void Rot_wrist(const double wrist[3], double R[3][3])
	{
		R[0][0] = cos(wrist[0])*cos(wrist[1])*cos(wrist[2]) - sin(wrist[0])*sin(wrist[2]);
		R[0][1] = -cos(wrist[2])*sin(wrist[0]) - cos(wrist[0])*cos(wrist[1])*sin(wrist[2]);
		R[0][2] = cos(wrist[0])*sin(wrist[1]);

		R[1][0] = cos(wrist[0])*sin(wrist[2]) + cos(wrist[1])*cos(wrist[2])*sin(wrist[0]);
		R[1][1] = cos(wrist[0])*cos(wrist[2]) - cos(wrist[1])*sin(wrist[0])*sin(wrist[2]);
		R[1][2] = sin(wrist[0])*sin(wrist[1]);

		R[2][0] = -cos(wrist[2])*sin(wrist[1]);
		R[2][1] = sin(wrist[1])*sin(wrist[2]);
		R[2][2] = cos(wrist[1]);
	}
	 
	//Rl*Rr
	void Mat_multi(const double Rl[3][3], const double Rr[3][3], double R[3][3])
	{
		R[0][0] = Rl[0][0] * Rr[0][0] + Rl[0][1] * Rr[1][0] + Rl[0][2] * Rr[2][0];
		R[0][1] = Rl[0][0] * Rr[0][1] + Rl[0][1] * Rr[1][1] + Rl[0][2] * Rr[2][1];
		R[0][2] = Rl[0][0] * Rr[0][2] + Rl[0][1] * Rr[1][2] + Rl[0][2] * Rr[2][2];

		R[1][0] = Rl[1][0] * Rr[0][0] + Rl[1][1] * Rr[1][0] + Rl[1][2] * Rr[2][0];
		R[1][1] = Rl[1][0] * Rr[0][1] + Rl[1][1] * Rr[1][1] + Rl[1][2] * Rr[2][1];
		R[1][2] = Rl[1][0] * Rr[0][2] + Rl[1][1] * Rr[1][2] + Rl[1][2] * Rr[2][2];

		R[2][0] = Rl[2][0] * Rr[0][0] + Rl[2][1] * Rr[1][0] + Rl[2][2] * Rr[2][0];
		R[2][1] = Rl[2][0] * Rr[0][1] + Rl[2][1] * Rr[1][1] + Rl[2][2] * Rr[2][1];
		R[2][2] = Rl[2][0] * Rr[0][2] + Rl[2][1] * Rr[1][2] + Rl[2][2] * Rr[2][2];
	}

	//R.transports，3x3矩阵转置
	void Mat_transpose(const double R[3][3], double Rt[3][3])
	{
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			Rt[i][j] = R[j][i];
		}
	}

  }
	//正运动学
  void forward(const double *joint, double *T)
  {
	  double s1 = sin(joint[0]), c1 = cos(joint[0]);
	  double s2 = sin(joint[1]), c2 = cos(joint[1]);
	  double s3 = sin(joint[2]), c3 = cos(joint[2]);
	  double s4 = sin(joint[3]), c4 = cos(joint[3]);
	  double s5 = sin(joint[4]), c5 = cos(joint[4]);
	  double s6 = sin(joint[5]), c6 = cos(joint[5]);
	  double s7 = sin(joint[6]), c7 = cos(joint[6]);
    *T = c7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + s7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3));
    T++; *T = c7*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - s7*(s6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - c6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
    T++; *T = -c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    T++; *T = Dse*c1*s2 - Dwt*(c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - Dew*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2);

    T++; *T = -c7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - s7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
    T++; *T = s7*(s6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) - c6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - c7*(s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
    T++; *T = c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
    T++; *T = Dew*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + Dwt*(c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + Dse*s1*s2;

    T++; *T = s7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) - c7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
    T++; *T = c7*(s5*(c2*s4 + c3*c4*s2) + c5*s2*s3) + s7*(c6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) + s6*(c2*c4 - c3*s2*s4));
    T++; *T = c6*(c2*c4 - c3*s2*s4) - s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5);
    T++; *T = Dbs + Dew*(c2*c4 - c3*s2*s4) + Dse*c2 - Dwt*(s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) - c6*(c2*c4 - c3*s2*s4));

    T++; *T = 0;
    T++; *T = 0;
    T++; *T = 0;
    T++; *T = 1;
  }


  //逆运动学 joint是初始值，joint_sols是最终的8组解
  int inverse(const double T[][4], double* joint, double* joint_sols) {
	double si_last;
	double *psi_last = &si_last;

    double Lbs[3] = { 0, 0, Dbs };  //肩关节在0系里的坐标
    double Lse[3] = { 0, -Dse, 0 }; //se矢量在3系里的表达
    double Lew[3] = { 0, 0, Dew };  //腕关节在4系里的坐标
    double Lwt[3] = { 0, 0, Dwt };  //wt矢量在7系中的表达

    //end: separate rotate and trans
    double end_r[3][3];
    double end_t[3];
    end_r[0][0] = T[0][0];	end_r[0][1] = T[0][1];	end_r[0][2] = T[0][2];	end_t[0] = T[0][3];
    end_r[1][0] = T[1][0];	end_r[1][1] = T[1][1];	end_r[1][2] = T[1][2];	end_t[1] = T[1][3];
    end_r[2][0] = T[2][0];	end_r[2][1] = T[2][1];	end_r[2][2] = T[2][2];	end_t[2] = T[2][3];


    //arm angle
    //     肩腕距离 
    double w_norm, psi_cur, sintheta, costheta, sqrtcheck;

    double psi, psi_cache[2];
    // double As_cache[2][3][3];
    // double Bs_cache[2][3][3];
    // double Cs_cache[2][3][3];
    // double Aw_cache[2][3][3];
    // double Bw_cache[2][3][3];
    // double Cw_cache[2][3][3];

    //loops for 16/8 solutions at fixed 循环的判断条件
    int lp1 = 1;
    int lp2 = 1;
    int lp3 = 1;

    //save the 8 solutions 8个解
    // double joint_temp[8][7];
    // double joint_tar[7] = { 10, 10, 10, 10, 10, 10, 10 };

    double theta1, theta2, theta3, theta4, theta5, theta6, theta7;


    //joint angle when ��=0
    double theta1_0, theta2_0, theta3_0, theta4_0;
    theta3_0 = 0.0;
    double shoulder_0[3];
    double r43_0[3][3], r43_0_t[3][3];
    double r30_0[3][3];
    double Lsw[3];  //sw在3系里的表达
    double Usw[3], Usw_m[3][3];
    double Uswx[3][3], Uswx_neg[3][3], Uswx_sq[3][3];
    double As[3][3], Bs[3][3], Cs[3][3], As_t[3][3], Bs_t[3][3], Cs_t[3][3];
    double Aw[3][3], Bw[3][3], Cw[3][3], Aw_temp[3][3], Bw_temp[3][3], Cw_temp[3][3];

    //best ��
    //     3对0系以及7对4系的旋转矩阵
    double r30_d[3][3], r74_d[3][3], r30_d_t[3][3], r74_d_t[3][3];
    double shoulder_d[3] = { 0.0, 0.0, 0.0 };   //肩关节的三个转动角度
    double wrist_d[3] = { 0.0, 0.0, 0.0 };  //腕关节的三个转动角度

    double AsR30dt[3][3], BsR30dt[3][3], CsR30dt[3][3], AwR74dt[3][3], BwR74dt[3][3], CwR74dt[3][3];
    double psi_best_1, psi_best_2, psi_best;
    double aaa, bbb, ccc;

    //region of ��
    double region_1[3], region_2[3], region[5]; //region_1&joint2   region2&joint6


    w_norm = sqrt(pow(Dse, 2) + pow(Dew, 2) + 2 * Dse*Dew*cos(joint[3]));   //涉及关节4,是肩腕距离(初始值?)
    sintheta = Dse*Dew*sin(joint[1])*sin(joint[2])*sin(joint[3]) / w_norm;
    costheta = -Dse / pow(w_norm, 2) * (-Dew * Dew * cos(joint[3]) * cos(joint[2]) * sin(joint[1]) * sin(joint[3]) + Dew * Dew * cos(joint[1]) * pow(cos(joint[3]), 2) - Dew * Dse * cos(joint[2]) * sin(joint[1]) * sin(joint[3]) + 2 * Dew * Dse * cos(joint[1]) * cos(joint[3]) + Dse * Dse * cos(joint[1]) - cos(joint[1]) * w_norm * w_norm);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    //ATAN2的实现,根据sintheta和costheta解出theta
    if (fabs(sintheta)<ZER)			//psi_cur = ATAN2psi_cur_y, psi_cur_x);
    {
        if (costheta>0)
            psi_cur = 0.0;
        else
            psi_cur = PI;
    }
    else if (sintheta > 0)
        psi_cur = acos(costheta / sqrt(pow(sintheta, 2) + pow(costheta, 2)));
    else
        psi_cur = -acos(costheta / sqrt(pow(sintheta, 2) + pow(costheta, 2)));


    //Xsw 极坐标系下的肩腕坐标差
    double Xsw0[3];
    Xsw0[0] = end_t[0] - Lbs[0] - (end_r[0][0] * Lwt[0] + end_r[0][1] * Lwt[1] + end_r[0][2] * Lwt[2]);
    Xsw0[1] = end_t[1] - Lbs[1] - (end_r[1][0] * Lwt[0] + end_r[1][1] * Lwt[1] + end_r[1][2] * Lwt[2]);
    Xsw0[2] = end_t[2] - Lbs[2] - (end_r[2][0] * Lwt[0] + end_r[2][1] * Lwt[1] + end_r[2][2] * Lwt[2]);

    //最终要求的肩腕距离
    double Xsw0_L = sqrt(pow(Xsw0[0], 2) + pow(Xsw0[1], 2) + pow(Xsw0[2], 2));

    //三个方向余弦(将肩腕位矢单位化)
    Usw[0] = Xsw0[0] / Xsw0_L; Usw[1] = Xsw0[1] / Xsw0_L; Usw[2] = Xsw0[2] / Xsw0_L;
    //肩腕位矢反对称矩阵
    Uswx[0][0] = 0;			Uswx[0][1] = -Usw[2];	Uswx[0][2] = Usw[1];
    Uswx[1][0] = Usw[2];	Uswx[1][1] = 0;			Uswx[1][2] = -Usw[0];
    Uswx[2][0] = -Usw[1];	Uswx[2][1] = Usw[0];	Uswx[2][2] = 0;

    //肩腕位矢反对称矩阵求反,negative
    for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
        Uswx_neg[i][j] = -Uswx[i][j];
    }

    //best ��
    //根据三个肩关节角度shoulder_d,求得3对0系的变换矩阵
    Rot_shoulder(shoulder_d, r30_d);
    //根据三个腕关节角度wrist_d,求得7对4系的变换矩阵
    Rot_wrist(wrist_d, r74_d);


    /*************************start loop**************************/
    //loop1:			//for two solutions of theta4_0
    //求得关节4的角度余弦,按这个公式算出来肯定是正数
    costheta = (pow(Xsw0_L, 2) - pow(Dse, 2) - pow(Dew, 2)) / (2 * Dse*Dew);
    if (fabs(costheta) > 1.0)   //说明在可达空间之外,sw的长度比se和ew加起来还大
        costheta = 1.0;

    if (joint[3] > 0)   //关节4初始值为正
    {
        if (fabs(costheta) > 1) //超出工作空间范围
            theta4_0 = 0;           //则不设定关节4
        else                    //未超出范围
            theta4_0 = acos(costheta);  //关节4设定为正值,转动角度较小(因为关节4的初始值也为正)
    }
    else
    {
        if (fabs(costheta) > 1) //超出工作空间范围
            theta4_0 = 0;
        else
            theta4_0 = -acos(costheta); //这时则选负值
    }

    //设置关节4
    theta4 = theta4_0;  

    //根据关节4的角度,计算3到4系的变换矩阵r43_0
    Rot_elbow(theta4_0, r43_0);
    //在3系里表达sw矢量
    Lsw[0] = Lse[0] + (r43_0[0][0] * Lew[0] + r43_0[0][1] * Lew[1] + r43_0[0][2] * Lew[2]);
    Lsw[1] = Lse[1] + (r43_0[1][0] * Lew[0] + r43_0[1][1] * Lew[1] + r43_0[1][2] * Lew[2]);
    Lsw[2] = Lse[2] + (r43_0[2][0] * Lew[0] + r43_0[2][1] * Lew[1] + r43_0[2][2] * Lew[2]);

loop1:				//for two solutions of theta2_0
    //
    sqrtcheck = pow(Lsw[0], 2) + pow(Lsw[1], 2) - pow(Xsw0[2], 2);
    if (sqrtcheck < ZER)
        sqrtcheck = 0.0;

    if (1 == lp1)
    {
        theta2_0 = 2 * atan((Lsw[0] - sqrt(sqrtcheck)) / (Lsw[1] - Xsw0[2]));
        lp1 = 0;
    }
    else
    {
        theta2_0 = 2 * atan((Lsw[0] + sqrt(sqrtcheck)) / (Lsw[1] - Xsw0[2]));
        lp1 = 1;
    }



    sintheta = Xsw0[1] / (Lsw[0] * cos(theta2_0) - Lsw[1] * sin(theta2_0));
    costheta = Xsw0[0] / (Lsw[0] * cos(theta2_0) - Lsw[1] * sin(theta2_0));
    //theta1_0 = atan2(sintheta, costheta);     //calculated slower than calculation below
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta1_0 = 0.0;
        else
            theta1_0 = PI;
    }
    else if (sintheta > 0)
        theta1_0 = acos(costheta);
    else
        theta1_0 = -acos(costheta);

    //As Bs Cs Aw Bw Cw
    shoulder_0[0] = theta1_0;
    shoulder_0[1] = theta2_0;
    shoulder_0[2] = theta3_0;
    Rot_shoulder(shoulder_0, r30_0);

    Mat_multi(Uswx, r30_0, As);
    Mat_multi(Uswx_neg, Uswx, Uswx_sq);
    Mat_multi(Uswx_sq, r30_0, Bs);

    Usw_m[0][0] = Usw[0] * Usw[0];	Usw_m[0][1] = Usw[0] * Usw[1];	Usw_m[0][2] = Usw[0] * Usw[2];
    Usw_m[1][0] = Usw[1] * Usw[0];	Usw_m[1][1] = Usw[1] * Usw[1];	Usw_m[1][2] = Usw[1] * Usw[2];
    Usw_m[2][0] = Usw[2] * Usw[0];	Usw_m[2][1] = Usw[2] * Usw[1];	Usw_m[2][2] = Usw[2] * Usw[2];
    Mat_multi(Usw_m, r30_0, Cs);

    Mat_transpose(r43_0, r43_0_t);
    Mat_transpose(As, As_t);
    Mat_transpose(Bs, Bs_t);
    Mat_transpose(Cs, Cs_t);
    Mat_multi(r43_0_t, As_t, Aw_temp);
    Mat_multi(Aw_temp, end_r, Aw);
    Mat_multi(r43_0_t, Bs_t, Bw_temp);
    Mat_multi(Bw_temp, end_r, Bw);
    Mat_multi(r43_0_t, Cs_t, Cw_temp);
    Mat_multi(Cw_temp, end_r, Cw);


    region_cos_type(-As[2][1], -Bs[2][1], -Cs[2][1], theta2_u, theta2_l, region_1);
    region_cos_type(Aw[2][2], Bw[2][2], Cw[2][2], theta6_u, theta6_l, region_2);
    region_union(region_1, region_2, region);

    Mat_transpose(r30_d, r30_d_t);
    Mat_transpose(r74_d, r74_d_t);
    Mat_multi(As, r30_d_t, AsR30dt);
    Mat_multi(Bs, r30_d_t, BsR30dt);
    Mat_multi(Cs, r30_d_t, CsR30dt);
    Mat_multi(Aw, r74_d_t, AwR74dt);
    Mat_multi(Bw, r74_d_t, BwR74dt);
    Mat_multi(Cw, r74_d_t, CwR74dt);

    aaa = (3 * (AsR30dt[0][0] + AsR30dt[1][1] + AsR30dt[2][2]) + 2 * (AwR74dt[0][0] + AwR74dt[1][1] + AwR74dt[2][2])) / 5;
    bbb = (3 * (BsR30dt[0][0] + BsR30dt[1][1] + BsR30dt[2][2]) + 2 * (BwR74dt[0][0] + BwR74dt[1][1] + BwR74dt[2][2])) / 5;
    ccc = (3 * (CsR30dt[0][0] + CsR30dt[1][1] + CsR30dt[2][2]) + 2 * (CwR74dt[0][0] + CwR74dt[1][1] + CwR74dt[2][2])) / 5;

    if (fabs(aaa)<ZER)
    {
        psi_best = 0;
    }
    else
    {
        psi_best_1 = 2 * atan((-bbb - sqrt(pow(aaa, 2) + pow(bbb, 2))) / aaa);
        psi_best_2 = 2 * atan((-bbb + sqrt(pow(aaa, 2) + pow(bbb, 2))) / aaa);
        psi_best = (aaa*sin(psi_best_1) + bbb*cos(psi_best_1) + ccc) > (aaa*sin(psi_best_2) + bbb*cos(psi_best_2) + ccc) ? psi_best_1 : psi_best_2;
    }

    if (fabs(region[0] - 1)<ZER)
    {
        if (psi_best>region[1] && psi_best < region[2])
        {
            psi = psi_best;
        }
        else if (psi_best<region[1])
        {
            psi = region[1];
        }
        else psi = region[2];
    }
    else
    {
        if ((psi_best>region[1] && psi_best < region[2]) || (psi_best>region[3] && psi_best < region[4]))
            psi = psi_best;
        else if (psi_best < region[1])
            psi = region[1];
        else if (psi_best > region[4])
            psi = region[4];
        else
            psi = fabs(region[2] - psi_best) < fabs(region[3] - psi_best) ? region[2] : region[3];
    }

    // psi_cache[lp1] = psi;

    // memcpy(As_cache + lp1 * 9 * sizeof(double), As, 9 * sizeof(double));
    // memcpy(Bs_cache + lp1 * 9 * sizeof(double), Bs, 9 * sizeof(double));
    // memcpy(Cs_cache + lp1 * 9 * sizeof(double), Cs, 9 * sizeof(double));
    // memcpy(Aw_cache + lp1 * 9 * sizeof(double), Aw, 9 * sizeof(double));
    // memcpy(Bw_cache + lp1 * 9 * sizeof(double), Bw, 9 * sizeof(double));
    // memcpy(Cw_cache + lp1 * 9 * sizeof(double), Cw, 9 * sizeof(double));

    if (fabs(*psi_last - psi_cur) > 0.01)
        *psi_last = psi_cur;

    if (false)  //原文是if(false)
        psi = *psi_last;
    else
    {
        if (fabs(psi - *psi_last) > 0.00001)
            psi = *psi_last + fabs(psi - *psi_last) / (psi - *psi_last)*0.00001;
        *psi_last = psi;
    }


loop2:				//for two solutions of theta2
    //theta2
    costheta = -As[2][1] * sin(psi) - Bs[2][1] * cos(psi) - Cs[2][1];

    if (fabs(costheta) > 1.0)
        costheta = fabs(costheta) / costheta;

    if (1 == lp2)
    {
        theta2 = acos(costheta);
        lp2 = 0;
    }
    else
    {
        theta2 = -acos(costheta);
        lp2 = 1;
    }

    //theta1
    sintheta = (-As[1][1] * sin(psi) - Bs[1][1] * cos(psi) - Cs[1][1]) / sin(theta2);
    costheta = (-As[0][1] * sin(psi) - Bs[0][1] * cos(psi) - Cs[0][1]) / sin(theta2);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta1 = 0.0;
        else
            theta1 = PI;
    }
    else if (sintheta > 0)
        theta1 = acos(costheta);
    else
        theta1 = -acos(costheta);

    //theta3
    sintheta = (As[2][2] * sin(psi) + Bs[2][2] * cos(psi) + Cs[2][2]) / sin(theta2);
    costheta = (-As[2][0] * sin(psi) - Bs[2][0] * cos(psi) - Cs[2][0]) / sin(theta2);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta3 = 0.0;
        else
            theta3 = PI;
    }
    else if (sintheta > 0)
        theta3 = acos(costheta);
    else
        theta3 = -acos(costheta);

loop3:				//for two solutions of theta6
    costheta = Aw[2][2] * sin(psi) + Bw[2][2] * cos(psi) + Cw[2][2];
    if (fabs(costheta) > 1.0)
        costheta = 1.0;
    //theta6
    if (1 == lp3)
    {
        theta6 = acos(costheta);
        lp3 = 0;
    }
    else
    {
        theta6 = -acos(costheta);
        lp3 = 1;
    }

    //theta5
    sintheta = (Aw[1][2] * sin(psi) + Bw[1][2] * cos(psi) + Cw[1][2]) / sin(theta6);
    costheta = (Aw[0][2] * sin(psi) + Bw[0][2] * cos(psi) + Cw[0][2]) / sin(theta6);

    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta5 = 0.0;
        else
            theta5 = PI;
    }
    else if (sintheta > 0)
        theta5 = acos(costheta);
    else
        theta5 = -acos(costheta);

    //theta7
    sintheta = (Aw[2][1] * sin(psi) + Bw[2][1] * cos(psi) + Cw[2][1]) / sin(theta6);
    costheta = (-Aw[2][0] * sin(psi) - Bw[2][0] * cos(psi) - Cw[2][0]) / sin(theta6);

    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta7 = 0.0;
        else
            theta7 = PI;
    }
    if (fabs(sintheta)<ZER)
        theta7 = 0.0;
    else if (sintheta>0)
        theta7 = acos(costheta);
    else
        theta7 = -acos(costheta);

    //save all solutions
	int row = 4 * lp1 + 2 * lp2 + lp3;
    joint_sols[7 * row] = theta1;
    joint_sols[7 * row + 1] = theta2;
    joint_sols[7 * row + 2] = theta3;
    joint_sols[7 * row + 3] = theta4;
    joint_sols[7 * row + 4] = theta5;
    joint_sols[7 * row + 5] = theta6;
    joint_sols[7 * row + 6] = theta7;


    if (0 == lp3)	goto loop3;
    if (0 == lp2)	goto loop2;
    if (0 == lp1)	goto loop1;
 
	return 8;
 };
}

#define IKFAST_HAS_LIBRARY
#include <mra_kinematics/ikfast.h>
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==61);

#ifdef IKFAST_NAMESPACE	//如果定义了IKFAST_NAMESPACE，后面这些就属于IKFAST_NAMESPACE的内容，若未定义则属于全局空间
namespace IKFAST_NAMESPACE {
#endif

void to_mat44(double * mat4_4, const IkReal* eetrans, const IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        mat4_4[i*4+0] = eerot[i*3+0];
        mat4_4[i*4+1] = eerot[i*3+1];
        mat4_4[i*4+2] = eerot[i*3+2];
        mat4_4[i*4+3] = eetrans[i];
    }
    mat4_4[3*4+0] = 0;
    mat4_4[3*4+1] = 0;
    mat4_4[3*4+2] = 0;
    mat4_4[3*4+3] = 1;
}

void from_mat44(const double * mat4_4, IkReal* eetrans, IkReal* eerot)
{
    for(int i=0; i< 3;++i){
        eerot[i*3+0] = mat4_4[i*4+0];
        eerot[i*3+1] = mat4_4[i*4+1];
        eerot[i*3+2] = mat4_4[i*4+2];
        eetrans[i] = mat4_4[i*4+3];
    }
}


//IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
//  if(!pfree) return false;

//  int n = GetNumJoints();
//  double q_sols[8*6];
//  double T[16];

//  to_mat44(T, eetrans, eerot);

//  int num_sols = mra_kinematics::inverse(T, q_sols, pfree[0]);

//  std::vector<int> vfree(0);

//  for (int i=0; i < num_sols; ++i){
//    std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
//    for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
//    solutions.AddSolution(vinfos,vfree);
//  }
//  return num_sols > 0;
//}

//IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot)
//{
//    double T[16];
//    mra_kinematics::forward(j,T);
//    from_mat44(T,eetrans,eerot);
//}

//IKFAST_API int GetNumFreeParameters() { return 1; }
//IKFAST_API int* GetFreeParameters() { static int freeparams[] = {5}; return freeparams; }
//IKFAST_API int GetNumJoints() { return 6; }

//IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

//#ifdef IKFAST_NAMESPACE
//} // end namespace
//#endif

//#ifndef IKFAST_NO_MAIN

//using namespace std;
//using namespace mra_kinematics;

//int main(int argc, char* argv[])
//{
//  double q[6] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
//  double* T = new double[16];
//  forward(q, T);
//  for(int i=0;i<4;i++) {
//    for(int j=i*4;j<(i+1)*4;j++)
//      printf("%1.3f ", T[j]);
//    printf("\n");
//  }
//  double q_sols[8*6];
//  int num_sols;
//  num_sols = inverse(T, q_sols);
//  for(int i=0;i<num_sols;i++)
//    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//  for(int i=0;i<=4;i++)
//    printf("%f ", PI/2.0*i);
//  printf("\n");
//  return 0;
//}
//#endif
