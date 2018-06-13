/*****************************************************************
* Description: pumakins.c
*   Kinematics for puma typed robots
*   Set the params using HAL to fit your robot
*
*   Derived from a work by Fred Proctor
*
* Author: 
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
* $Revision: 1.4 $
* $Author: alex_joni $
* $Date: 2007/03/03 22:34:32 $
*******************************************************************
*/ 
#include<math.h>
#include <stdio.h>
#include "posemath.h"
#include "pumakins.h"
#include "kinematics.h"             /* decls for kinematicsForward, etc. */

#define DBL_MIN  0.000001


int isEqual(double v1, double v2)
{
	if (fabs(v1 - v2) < 1e-6)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int cartIsEqual(PmCartesian c1, PmCartesian c2)
{
	if ((0 == isEqual(c1.x, c2.x)) && (0 == isEqual(c1.y, c2.y)) && (0 == isEqual(c1.z, c2.z)))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}


int matricIsEqual(PmRotationMatrix m1, PmRotationMatrix m2)
{
	if ((0 == cartIsEqual(m1.x, m2.x)) && (0 == cartIsEqual(m1.y, m2.y)) && (0 == cartIsEqual(m1.z, m2.z)))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}



  	extern double PUMA_A1;
	extern double PUMA_A2;
	extern double PUMA_A3;
	extern double PUMA_D3;
	extern double PUMA_D4;
	extern double PUMA_D6;
//extern MotionConfig *emcmotConfig ;


int kinematicsForward2( double * joint,
                      PmRotationMatrix* Ehom_M,PmCartesian* Ehom_P,
                       KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
   
		double s1, s2, s3, s4, s5, s6;
		double c1, c2, c3, c4, c5, c6;
		double f[16],h[16],g[16];
		PmHomogeneous hom;

//		double PUMA_A1=emcmotConfig->PUMA_A[0];
//		double PUMA_A2=emcmotConfig->PUMA_A[1];
//		double PUMA_A3=emcmotConfig->PUMA_A[2];
//		double PUMA_D4=emcmotConfig->PUMA_A[3];
//		double PUMA_D6=emcmotConfig->PUMA_A[4];

 //   world->tran.x= joint[0];
 //   world->tran.y= joint[1];
 //   world->tran.z= joint[2];
 //   world->a=joint[3] ;
 //   world->b=joint[4] ;
  //  world->c=joint[5] ;


		joint[1]=joint[1]-90;
		/* Calculate sin of joints for future use */

		s1 = sin(joint[0]*PM_PI/180);
		s2 = sin(joint[1]*PM_PI/180);
		s3 = sin(joint[2]*PM_PI/180);
		s4 = sin(joint[3]*PM_PI/180);
		s5 = sin(joint[4]*PM_PI/180);
		s6 = sin(joint[5]*PM_PI/180);

		/* Calculate cos of joints for future use */
		c1 = cos(joint[0]*PM_PI/180);
		c2 = cos(joint[1]*PM_PI/180);
		c3 = cos(joint[2]*PM_PI/180);
		c4 = cos(joint[3]*PM_PI/180);
		c5 = cos(joint[4]*PM_PI/180);
		c6 = cos(joint[5]*PM_PI/180);


		f[0]=c1*c2*c3+c1*s2*s3;
		f[1]=-s1;
		f[2]=-c1*c2*s3+c1*s2*c3;
		f[3]=PUMA_A3*f[0]+PUMA_A1*c1+PUMA_A2*c1*c2;
		f[4]=s1*c2*c3+s1*s2*s3;
		f[5]=c1;
		f[6]=-s1*c2*s3+s1*s2*c3;
		f[7]=PUMA_A3*f[4]+PUMA_A1*s1+PUMA_A2*s1*c2;
		f[8]=-s2*c3+c2*s3;
		f[9]=0;
		f[10]=s2*s3+c2*c3;
		f[11]=PUMA_A3*f[8]-PUMA_A2*s2;

		g[0]=c4*f[0]+s4*f[1];
		g[1]=f[2];
		g[2]=s4*f[0]-c4*f[1];
		g[3]=-PUMA_D4*f[2]+f[3];
		g[4]=c4*f[4]+s4*f[5];
		g[5]=f[6];
		g[6]=s4*f[4]-c4*f[5];
		g[7]=-PUMA_D4*f[6]+f[7];
		g[8]=c4*f[8]+s4*f[9];
		g[9]=f[10];
		g[10]=s4*f[8]-c4*f[9];
		g[11]=-PUMA_D4*f[10]+f[11];

		h[0]=c5*g[0]+s5*g[1];
		h[1]=-g[2];
		h[2]=-s5*g[0]+c5*g[1];
		h[3]=g[3];
		h[4]=c5*g[4]+s5*g[5];
		h[5]=-g[6];
		h[6]=-s5*g[4]+c5*g[5];
		h[7]=g[7];
		h[8]=c5*g[8]+s5*g[9];
		h[9]=-g[10];
		h[10]=-s5*g[8]+c5*g[9];
		h[11]=g[11];

		hom.rot.x.x=c6*h[0]+s6*h[1];
		hom.rot.x.y=c6*h[4]+s6*h[5];
		hom.rot.x.z=c6*h[8]+s6*h[9];

		hom.rot.y.x=s6*h[0]-c6*h[1];	
		hom.rot.y.y=s6*h[4]-c6*h[5];
		hom.rot.y.z=s6*h[8]-c6*h[9];

		hom.rot.z.x=-h[2];
		hom.rot.z.y=-h[6];	
		hom.rot.z.z=-h[10];

		hom.tran.x=-PUMA_D6*h[2]+h[3];
		hom.tran.y=-PUMA_D6*h[6]+h[7];
		hom.tran.z=-PUMA_D6*h[10]+h[11];
		*Ehom_M =hom.rot;
		*Ehom_P=hom.tran;


	return 0;

}
int kinematicsForward_axis5(double * joint,
							RobotPose* world,
							PmRotationMatrix* Ehom,
							KINEMATICS_FORWARD_FLAGS * fflags,
							KINEMATICS_INVERSE_FLAGS * iflags)
{ 
	double s1, s2, s3, s4, s5, s345, s34;
	double c1, c2, c3, c4, c5, c345, c34;
	double theta345,theta34;
	PmHomogeneous hom;

	double L1, L2, L3, L4;
	L1 = 100;
	L2 = 100;
	L3 = 100;
	L4 = 100;

	joint[1] = joint[1] - 90;
	joint[2] = joint[2] - 90;

	theta34 = joint[2] + joint[3];
	theta345 = joint[2] + joint[3] + joint[4];

	s1 = sin(joint[0] * PM_PI / 180);
	s2 = sin(joint[1] * PM_PI / 180);
	s3 = sin(joint[2] * PM_PI / 180);
	s4 = sin(joint[3] * PM_PI / 180);
	s5 = sin(joint[4] * PM_PI / 180);
	s34 = sin(theta34 * PM_PI / 180);
	s345= sin(theta345 * PM_PI / 180);



	c1 = cos(joint[0] * PM_PI / 180);
	c2 = cos(joint[1] * PM_PI / 180);
	c3 = cos(joint[2] * PM_PI / 180);
	c4 = cos(joint[3] * PM_PI / 180);
	c5 = cos(joint[4] * PM_PI / 180);
	c34 = cos(theta34 * PM_PI / 180);
	c345 = cos(theta345 * PM_PI / 180);


	hom.rot.x.x = c1*c2*c345 + s1*s345;
	hom.rot.x.y = s1*c2*c345 - c1*s345;
	hom.rot.x.z = -1 * s2*c345 ;

	hom.rot.y.x = -1 * c1*c2*c345 + s1*s345;
	hom.rot.y.y = -1 * s1*c2*c345 - c1*c345;
	hom.rot.y.z = s2*s345;

	hom.rot.z.x = -c1*s2;
	hom.rot.z.y = -c1*c34;
	hom.rot.z.z = -c2;

	hom.tran.x = (c1*c2*c34+s1*s34)*L2+(c1*c2*c3+s1*s3)*L3-s1*L4;
	hom.tran.y = (s1*c2*c34-c1*s34)*L2+(s1*c2*c3-c1*s3)*L3+c1*L4;
	hom.tran.z = -s2*c34*L2-s2*c3*L3;
	

	*Ehom = hom.rot;
	world->tran = hom.tran;

	joint[1] = joint[1] + 90;
	joint[2] = joint[2] + 90;

	return 0;
}

int kinematicsInverse_axis5(RobotPose * world, PmHomogeneous* hom,
	double * joint, double *pre_joint,
	int  flag)
{
	double goal[8][5];//顺序存储八组解
	double th1, th2, th3, th4, th5;//存储各关节的弧度值
	double TH1[2], TH2[2], TH4[2]; //关节1,2,4分别有两个值
	double s1, s2, s3, s4, s5, s345;//各关节的正弦值	
	double c1, c2, c3, c4, c5, c345;//各关节的余弦值
	int i, j, k, l, m, n;		//循环迭代
	double A, B, M, N; //计算的中间值

	double L1, L2, L3, L4;
	L1 = 100;
	L2 = 100;
	L3 = 100;
	L4 = 100;
	

	TH1[0] = atan2(hom->rot.z.y, hom->rot.z.x);
	TH1[1] = atan2(-1*(hom->rot.z.y), -1*(hom->rot.z.x));


	for (i = 0; i < 2; i++)
	{
		c1=cos(TH1[i]);
		s1 = sin(TH1[i]);	
		goal[i * 4][0] = TH1[i];
		TH2[0] = atan2(-1 * (hom->tran.z), c1*hom->tran.x + s1*hom->tran.y);
		TH2[1] = atan2((hom->tran.z), -1* c1*hom->tran.x - s1*hom->tran.y);
		for (j = 0; j < 2; j++)
		{
			c2 = cos(TH2[j]);
			s2 = sin(TH2[j]);
			goal[i * 4 + j * 2][1] = TH2[j];
			A = c1*c2*hom->tran.x + s1*c2*hom->tran.y - s2*hom->tran.z;
			B = -1 * s1*hom->tran.x + c1*hom->tran.y - L4;
			for (k = 0; k < 2; k++)
			{
				c4 = (A*A + B*B - L2*L2 - L3*L3) / (2 * L2*L3);
				s4 = sqrt(1 - c4*c4);
				TH4[0] = atan2(s4, c4);
				TH4[1] = atan2(-s4, c4);
				s4 = sin(TH4[k]);

				M = c4*L2 + L3;
				N = s4*L2;
				s3 = (A*N - B*M) / (M*M - N*N);
				c3 = (A*M - B*N) / (M*M - N*N);

				th3 = atan2(s3, c3);

				th5 = atan2(s1*hom->rot.x.x - c1*hom->rot.x.z, s1*hom->rot.y.x - c1*hom->rot.y.y)-th3-TH4[k];
	
				goal[i * 4 + j * 2 + k][2] = th3;
				goal[i * 4 + j * 2 + k][3] = TH4[k];
				goal[i * 4 + j * 2 + k][4] = th5;
			}
		}
	}
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 5; j++)
		{
			goal[i][i] = (goal[i][i] * 180 / PM_PI);
		}
	}


	return 0;
}


int kinematicsForward( double * joint,
                      RobotPose* world,
                       PmRotationMatrix* Ehom,
                       KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
   
		   double s1, s2, s3, s4, s5, s6;
		   double c1, c2, c3, c4, c5, c6;
   //	   double f[16],h[16],g[16];
		   double th1, th2, th3, th4 ,th5, th6;
		   PmHomogeneous hom;
		   double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	//	 world->tran.x= joint[0];
	//	 world->tran.y= joint[1];
	//	 world->tran.z= joint[2];
	//	 world->a=joint[3] ;
	//	 world->b=joint[4] ;
	 //  world->c=joint[5] ;
   
   
   
   
		   joint[1]=joint[1]-90;
		   /* Calculate sin of joints for future use */
   
   
   
   
		   s1 = sin(joint[0]*PM_PI/180);
		   s2 = sin(joint[1]*PM_PI/180);
		   s3 = sin(joint[2]*PM_PI/180);
		   s4 = sin(joint[3]*PM_PI/180);
		   s5 = sin(joint[4]*PM_PI/180);
		   s6 = sin(joint[5]*PM_PI/180);
   
		   /* Calculate cos of joints for future use */
		   c1 = cos(joint[0]*PM_PI/180);
		   c2 = cos(joint[1]*PM_PI/180);
		   c3 = cos(joint[2]*PM_PI/180);
		   c4 = cos(joint[3]*PM_PI/180);
		   c5 = cos(joint[4]*PM_PI/180);
		   c6 = cos(joint[5]*PM_PI/180);
   
		   //nx = sin(th6)*(cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + cos(th6)*(cos(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - sin(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)))
		   nx = s6*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)) - c6*(s5*(c1*c2*s3 + c1*c3*s2) - c5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)));
		   ny = -1*c6*(s5*(c2*s1*s3 + c3*s1*s2) + c5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - s6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3));
		   nz = s4*s6*(c2*s3 + c3*s2) - c6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2));
		   
		   ox = s6*(s5*(c1*c2*s3 + c1*c3*s2) - c5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + c6*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3));
		   oy = s6*(s5*(c2*s1*s3 + c3*s1*s2) + c5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - c6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3));
		   oz = s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) + c6*s4*(c2*s3 + c3*s2);
   
		   ax = -1*c5*(c1*c2*s3 + c1*c3*s2) - s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3));
		   ay = s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3)) - c5*(c2*s1*s3 + c3*s1*s2);
		   az = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3);
   
		   px = PUMA_A1*c1 - PUMA_D3*s1 - PUMA_D6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + PUMA_A3*(c1*c2*c3 - c1*s2*s3) - PUMA_D4*(c1*c2*s3 + c1*c3*s2) + PUMA_A2*c1*c2;
		   py = PUMA_D3*c1 + PUMA_A1*s1 - PUMA_D6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + PUMA_A3*(c2*c3*s1 - s1*s2*s3) - PUMA_D4*(c2*s1*s3 + c3*s1*s2) + PUMA_A2*c2*s1;
		   pz = -1* PUMA_A2*s2 - PUMA_A3*(c2*s3 + c3*s2) - PUMA_D4*(c2*c3 - s2*s3) - PUMA_D6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2));
   
   
   
   
   
		   hom.rot.x.x = nx;
		   hom.rot.x.y = ny;
		   hom.rot.x.z = nz;
   
		   hom.rot.y.x = ox;
		   hom.rot.y.y = oy;
		   hom.rot.y.z = oz;
   
		   hom.rot.z.x = ax;
		   hom.rot.z.y = ay;
		   hom.rot.z.z = az;
   
		   hom.tran.x = px;
		   hom.tran.y = py;
		   hom.tran.z = pz;
		   *Ehom = hom.rot;
		   world->tran= hom.tran;
   
   
		   /*f[0]=c1*c2*c3+c1*s2*s3;
		   f[1]=-s1;
		   f[2]=-c1*c2*s3+c1*s2*c3;
		   f[3]=PUMA_A3*f[0]+PUMA_A1*c1+PUMA_A2*c1*c2;
		   f[4]=s1*c2*c3+s1*s2*s3;
		   f[5]=c1;
		   f[6]=-s1*c2*s3+s1*s2*c3;
		   f[7]=PUMA_A3*f[4]+PUMA_A1*s1+PUMA_A2*s1*c2;
		   f[8]=-s2*c3+c2*s3;
		   f[9]=0;
		   f[10]=s2*s3+c2*c3;
		   f[11]=PUMA_A3*f[8]-PUMA_A2*s2;
   
		   g[0]=c4*f[0]+s4*f[1];
		   g[1]=f[2];
		   g[2]=s4*f[0]-c4*f[1];
		   g[3]=-PUMA_D4*f[2]+f[3];
		   g[4]=c4*f[4]+s4*f[5];
		   g[5]=f[6];
		   g[6]=s4*f[4]-c4*f[5];
		   g[7]=-PUMA_D4*f[6]+f[7];
		   g[8]=c4*f[8]+s4*f[9];
		   g[9]=f[10];
		   g[10]=s4*f[8]-c4*f[9];
		   g[11]=-PUMA_D4*f[10]+f[11];
   
		   h[0]=c5*g[0]+s5*g[1];
		   h[1]=-g[2];
		   h[2]=-s5*g[0]+c5*g[1];
		   h[3]=g[3];
		   h[4]=c5*g[4]+s5*g[5];
		   h[5]=-g[6];
		   h[6]=-s5*g[4]+c5*g[5];
		   h[7]=g[7];
		   h[8]=c5*g[8]+s5*g[9];
		   h[9]=-g[10];
		   h[10]=-s5*g[8]+c5*g[9];
		   h[11]=g[11];
   
		   hom.rot.x.x=c6*h[0]+s6*h[1];
		   hom.rot.x.y=c6*h[4]+s6*h[5];
		   hom.rot.x.z=c6*h[8]+s6*h[9];
   
		   hom.rot.y.x=s6*h[0]-c6*h[1];    
		   hom.rot.y.y=s6*h[4]-c6*h[5];
		   hom.rot.y.z=s6*h[8]-c6*h[9];
   
		   hom.rot.z.x=-h[2];
		   hom.rot.z.y=-h[6];  
		   hom.rot.z.z=-h[10];
   
		   hom.tran.x=-PUMA_D6*h[2]+h[3];
		   hom.tran.y=-PUMA_D6*h[6]+h[7];
		   hom.tran.z=-PUMA_D6*h[10]+h[11];
		   *Ehom_M =hom.rot;
		   *Ehom_P=hom.tran;*/
   		joint[1]=joint[1]+90;
   
	   return 0;


}

int kinematicsInverse( RobotPose * world,PmHomogeneous* hom,
                      double * joint,double *pre_joint,
                      int  flag)
{
		int i,r,n;	
		int k,j,m;//循环变量	
		double goal[8][6];//顺序存储八组解
		//double testInverse[8][6];
	
		double dsum[8]={0}; //计算8组解的同前一组角度的误差
		int MN;
	//	int err[8]; 
		unsigned char errFlag;
		int skipFlag[8];
		int Flag=0; 
		double th1,th2,th3,th4,th5,th6;//存储各关节的弧度值
		double TH1[2],TH2[2],TH5[2]; //th1 th2 th5 th6各有两种解法	
		double s1, s2, s3, s4, s5, s6;//各关节的正弦值	
		double c1, c2, c3, c4, c5, c6;//各关节的余弦值
		double K,L; 
		double A, B, C; 			//计算2关节角度值
		double A3, B3, C3;			//计算关节3角度值
		double px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az;
	
	
		errFlag = 0;   //错误提示？
	
		double e1=atan2(1, 1);
		double e2 = atan2(-1, -1);
	
	
		//计算5关节点位置。
		PmHomogeneous T7N;
		PmHomogeneous hom_5;
		T7N.tran.x = 0;
		T7N.tran.y = 0;
		T7N.tran.z = -1*PUMA_D6;
	
		T7N.rot.x.x = 1;
		T7N.rot.x.y = 0;
		T7N.rot.x.z = 0;
	
		T7N.rot.y.x = 0;
		T7N.rot.y.y = 1;
		T7N.rot.y.z = 0;
	
		T7N.rot.z.x = 0;
		T7N.rot.z.y = 0;
		T7N.rot.z.z = 1;
	
	
		pmMatMatMult(hom->rot, T7N.rot, &(hom_5.rot));
		pmMatCartMult(hom->rot, T7N.tran, &(hom_5.tran));
		pmCartCartAdd(hom_5.tran, hom->tran, &(hom_5.tran));
	
		//计算1关节角度
		px = hom_5.tran.x;
		py = hom_5.tran.y;
		pz = hom_5.tran.z;
	
		nx = hom_5.rot.x.x;
		ny = hom_5.rot.x.y;
		nz = hom_5.rot.x.z;
	
		ox = hom_5.rot.y.x;
		oy = hom_5.rot.y.y;
		oz = hom_5.rot.y.z;
	
		ax = hom_5.rot.z.x;
		ay = hom_5.rot.z.y;
		az = hom_5.rot.z.z;
	
		TH1[0] = atan2(py, px) - atan2(PUMA_D3, sqrt(px*px + py*py - PUMA_D3*PUMA_D3));
		TH1[1]= atan2(py, px) - atan2(PUMA_D3, -1*sqrt(px*px + py*py - PUMA_D3*PUMA_D3));
	
	
		//计算2关节角度
	
		for (i = 0; i < 2; i++)
		{
			if (TH1[i] > PM_PI)
			{
				TH1[i] -= PM_2_PI;
			}
			else if (TH1[i] < -1 * PM_PI)
			{
				TH1[i] += PM_2_PI;
			}
			th1 = TH1[i];
			s1 = sin(th1);
			c1 = cos(th1);
			C = PUMA_A3*PUMA_A3 + PUMA_D4*PUMA_D4 - px*px*c1*c1 + py*py*c1*c1 - PUMA_A1*PUMA_A1 - PUMA_A2*PUMA_A2 - py*py - pz*pz + 2 * PUMA_A1*py*s1 - px*py*sin(2 * th1) + 2 * PUMA_A1*px*c1;
			A = 2 * PUMA_A1*PUMA_A2 - 2*PUMA_A2*px*c1 - 2*PUMA_A2*py*s1;
			B = -1*2 * PUMA_A2*pz;
			//TH2[0] = atan2(B + sqrt(B*B + A*A - C*C), A + C);
			//TH2[1] = atan2(B - sqrt(B*B + A*A - C*C), A + C);
			TH2[0] = atan2(A, B) - atan2(C,sqrt(A*A + B*B - C*C));
			TH2[1] = atan2(A, B) - atan2(C, -1*sqrt(A*A + B*B - C*C));
	
	
			//计算关节3 角度
			for (j = 0; j < 2; j++)
			{
				if (TH2[j] > PM_PI)
				{
					TH2[j] -= PM_2_PI;
				}
				else if (TH2[j] < -1 * PM_PI)
				{
					TH2[j] += PM_2_PI;
				}
				th2 = TH2[j];
				s2 = sin(th2);
				c2 = cos(th2);
				B3 = px*c1*c2-PUMA_A1*c2- PUMA_A2 - pz*s2 + py*s1*c2;
				A3 = PUMA_A1*s2 - pz*c2 - px*c1*s2 - py*s1*s2;
				th3 = atan2((A3*PUMA_A3 - B3*PUMA_D4) / (A3*A3 + B3*B3), (B3*PUMA_A3 + A3*PUMA_D4) / (A3*A3 + B3*B3));
				s3 = sin(th3);
				c3 = cos(th3);
	
				//计算关节5的角度值
				c5 = -1 * az*(c2*c3 - s2*s3) - ax*c1*(c2*s3 + c3*s2) - ay*s1*(c2*s3 + c3*s2);
				s5 = sqrt(1 - c5*c5);
				TH5[0] = atan2(s5, c5);
				TH5[1] = atan2(-1*s5, c5);
				
				//计算关节4的角度值
				for (k = 0; k < 2; k++)
				{
					c5 = cos(TH5[k]);
					s5 = sin(TH5[k]);
					if (fabs(sin(TH5[k])) < DBL_MIN)
					{
						th4 = pre_joint[3] / 180 * PM_PI;
					}
					else
					{
						th4 = atan2((ay*c1 - ax*s1)/s5 , (az*(c2*s3+c3*s2) - ax*(c2*c3-s2*s3)*c1 - ay*(c2*c3-s2*s3)*s1)/s5);
					}
					s4 = cos(th4);
					c4 = sin(th4);
					//-nz*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - nx*cos(th1)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - ny*sin(th1)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))
					//-oz*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - ox*cos(th1)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - oy*sin(th1)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))
						//th6 = atan2(nz*(c2*s3*s4 + c3*s2*s4) + nx*(c4*s1 - c1*(c2*c3*s4 - s2*s3*s4)) - ny*(c1*c4 + s1*(c2*c3*s4 - s2*s3*s4)), oz*(c2*s3*s4 + c3*s2*s4) + ox*(c4*s1 - c1*(c2*c3*s4 - s2*s3*s4)) - oy*(c1*c4 + s1*(c2*c3*s4 - s2*s3*s4)));
					if (fabs(sin(TH5[k])) < DBL_MIN)
					{
						//th4 = pre_joint[3];
						//nx*cos(th1)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - nz*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + ny*sin(th1)*(cos(th2)*cos(th3) - sin(th2)*sin(th3));
						//ny*cos(th1) - nx*sin(th1);
						th6 = atan2(nx*s1-ny*c1, nx*c1*(c2*c3 - s2*s3) - nz*(c2*s3 + c3*s2) + ny*s1*(c2*c3 - s2*s3)) - th4;
					}
					else
					{
						th6 = atan2((oz*(c2*c3 - s2*s3) + ox*c1*(c2*s3 + c3*s2) + oy*s1*(c2*s3 + c3*s2))/s5, (-1 * nz*(c2*c3 - s2*s3) - nx*c1*(c2*s3 + c3*s2) - ny*s1*(c2*s3 + c3*s2))/s5);
					}
	
					
					//记录8组解的位置
					goal[i * 4 + j * 2 + k][0] = TH1[i];
					goal[i * 4 + j * 2 + k][1] = TH2[j];
					goal[i * 4 + j * 2 + k][2] = th3;
					goal[i * 4 + j * 2 + k][4] = TH5[k];
					goal[i * 4 + j * 2 + k][3] = th4;
					goal[i * 4 + j * 2 + k][5] = th6;
				}
			}
		
	
		}
	
		//计算最优解。
	
		for (i = 0; i < 8; i++)
		{
			dsum[i] = 0.0;
			for (j = 0; j < 6; j++)
			{
				goal[i][j] = (goal[i][j] * 180 / PM_PI);
				if (1 == j)
				{
					goal[i][1] = goal[i][1] + 90;
	
				}
				//joint[j]=goal[0][j];
				if (j < 6)
				dsum[i] += (goal[i][j] - pre_joint[j])*(goal[i][j] - pre_joint[j]);
			}
			if (dsum[i] - dsum[Flag]<0)
			{
				Flag = i;
			}
		}
		for (i = 0; i<6; i++)
		{
			joint[i] = goal[Flag][i];
		}
		if (pre_joint[5] > 170.0&&joint[5]<0.0)
		{
			joint[5] += 360.0;
		}
		else if (pre_joint[5] < -170.0&&joint[5]>0.0)
		{
			joint[5] -= 360.0;
		}
		if( 	goal[Flag][0]-181>1e-6 || goal[Flag][0]+181<1e-6
			|| goal[Flag][1]-181>1e-6 || goal[Flag][1]+181<1e-6
			|| goal[Flag][2]-181>1e-6 || goal[Flag][2]+181<1e-6
			|| goal[Flag][3]-181>1e-6 || goal[Flag][3]+181<1e-6 
			|| goal[Flag][4]-181>1e-6 || goal[Flag][4]+181<1e-6
			|| goal[Flag][5]-360>1e-6 || goal[Flag][5]+360<1e-6  )
		{
		
			return -1;
		}

		
	
	/*//	rtapi_print("a1,a2,a3,d4,d6 =%d,%d,%d,%d,%d \n",(int)(1000*PUMA_A1),(int)(1000*PUMA_A2),(int)(1000*PUMA_A3),(int)(1000*PUMA_D4),(int)(1000*PUMA_D6));
	//		rtapi_print("inverse hom.angle3=%d,%d,%d,\n",(int)(1000*world->a),(int)(1000*world->b),(int)(1000*world->c));
	
	//		rtapi_print("hom.rot9=\n%d,%d,%d,\n%d,%d,%d,\n%d,%d,%d,\n%d,%d,%d\n",(int)(1000*hom->rot.x.x),(int)(1000*hom->rot.x.y),(int)(1000*hom->rot.x.z),(int)(1000*hom->rot.y.x),(int)(1000*hom->rot.y.y),(int)(1000*hom->rot.y.z),(int)(1000*hom->rot.z.x),(int)(1000*hom->rot.z.y),(int)(1000*hom->rot.z.z),(int)(1000*hom->tran.x),(int)(1000*hom->tran.y),(int)(1000*hom->tran.z));
	
			TH1[0] = atan2(-1*hom->rot.z.y*PUMA_D6+hom->tran.y,-1*hom->rot.z.x*PUMA_D6+hom->tran.x);
			TH1[1] = atan2(hom->rot.z.y*PUMA_D6-hom->tran.y,hom->rot.z.x*PUMA_D6-hom->tran.x);		
	
		for(i=0; i<2; i++)
		{
			double h,v,w;
			th1 = TH1[i];
			for(j=i*4; j<(i+1)*4; j++)
			{
				g[j][0] = th1*180/PM_PI;
			}
			s1 = sin(th1);
			c1 = cos(th1);
	
			h = PUMA_A1+(hom->rot.z.x*PUMA_D6-hom->tran.x)*c1+hom->rot.z.y*PUMA_D6*s1-hom->tran.y*s1;
			v = hom->rot.z.z*PUMA_D6-hom->tran.z;
			
			if (fabs(h*h+v*v) < DBL_MIN)   //无法计算，回退
			{
			//	errFlag |= 0x01;   
			//	goto LabelErr;
				joint[0]=pre_joint[0];
				joint[1]=pre_joint[1];	 
				joint[2]=pre_joint[2];	
				joint[3]=pre_joint[3];	
				joint[4]=pre_joint[4];	
				joint[5]=pre_joint[5]; 
	
				return -1; 
			}
				
			w = (PUMA_A3*PUMA_A3+PUMA_D4*PUMA_D4-PUMA_A2*PUMA_A2-h*h-v*v)/(2*PUMA_A2*sqrt(h*h+v*v));
			
			if(fabs(w)>1)
			{
				MN = 4;
				continue;
			}
	
			else MN = 8;
	
			
			TH2[0] = atan2(h,v) - atan2(w,sqrt(1-w*w));
			TH2[1] = atan2(h,v) - atan2(w,-sqrt(1-w*w));
	
			for(k=0; k<2; k++)
			{
				double temp;
				th2 = TH2[k];
				s2 = sin(th2);
				c2 = cos(th2);
				for(m=0; m<2; m++)
				{
					double CC,DD;
					g[i*4+k*2+m][1] = th2*180/PM_PI; //????
					
					CC = -PUMA_A2-h*c2+v*s2;
					DD = -v*c2-h*s2;
	
					th3 = atan2(CC*PUMA_D4+DD*PUMA_A3,CC*PUMA_A3-DD*PUMA_D4);
					g[i*4+k*2+m][2] = th3*180/PM_PI; 
					s3 = sin(th3);
					c3 = cos(th3);
				}
							
				temp = c1*hom->rot.z.y - s1*hom->rot.z.x;
	
				K = -c1*c2*hom->rot.z.x - s1*c2*hom->rot.z.y + s2*hom->rot.z.z;  //ax'
				L = -c1*s2*hom->rot.z.x - s1*s2*hom->rot.z.y - c2*hom->rot.z.z;  //ay'
	
				if(fabs(c3*L-s3*K-1)>1e-10)   //s5!=0 
				{
					double t[3]={0};
					int fg,r=0;
					double pre5;
					pre5 = pre_joint[5]*PM_PI/180;
					c5 = (c2*s3-s2*c3)*(hom->rot.z.x*c1+hom->rot.z.y*s1)-hom->rot.z.z*(s2*s3+c2*c3);
	
					TH5[0] = atan2(sqrt(1-c5*c5),c5);
					TH5[1] = atan2(-sqrt(1-c5*c5),c5);
					
					for(n=0; n<2; n++)
					{
						double Ax,Bx;
						double A,B;
						th5 = TH5[n];
						s5 = sin(th5);
						c5 = cos(th5);
	
						g[i*4+k*2+n][4]= th5*180/PM_PI;
											
						s4 = temp/s5;
						A = (-c1*c2*hom->rot.z.x - s1*c2*hom->rot.z.y+s2*hom->rot.z.z)*c3;
						B = (-c1*s2*hom->rot.z.x - s1*s2*hom->rot.z.y-c2*hom->rot.z.z)*s3;
	
						c4 = -(A+B)/s5;
						th4 =  atan2(s4,c4);
			
						
						g[i*4+k*2+n][3]= th4*180/PM_PI;
			
						Ax = s1*hom->rot.x.x - c1*hom->rot.x.y;  
						Bx = s1*hom->rot.y.x - c1*hom->rot.y.y;
					
						if (Ax*Ax+Bx*Bx < DBL_MIN)	 //无法计算，回退
						{
						//	errFlag |= 0x03;
						//	goto LabelErr;
							joint[0]=pre_joint[0];
							joint[1]=pre_joint[1];	 
							joint[2]=pre_joint[2];	
							joint[3]=pre_joint[3];	
							joint[4]=pre_joint[4];	
							joint[5]=pre_joint[5]; 
	
							return -1;
						}
					
						s6 = (-c5*s4*Bx-c4*Ax)/(Ax*Ax+Bx*Bx);  //利用Nz'和Oz'计算th6
						c6 = (c4*Bx-c5*s4*Ax)/(Ax*Ax+Bx*Bx);
						th6 = atan2(s6,c6);
	
						t[0] = th6;
						t[1] = th6+PM_PI;
						t[2] = th6-PM_PI;
						fg = 0;
						for(r=1; r<3; r++)
						{
							if(fabs(t[r]-pre5)-fabs(t[fg]-pre5)<1e-6)
								fg = r;
						}
						th6=t[fg];
						
						g[i*4+k*2+n][5] = t[0]*180/PM_PI;
					}
				}
	
				else  //s5=0
				{
				
						int fg;
						double h=0,t[5]={0};  //h=th4+th6
						double pj=0,d=0;
	
						pj= (pre_joint[3]+pre_joint[5])*PM_PI/180;
	
						th5 = 0;
						h = atan2(hom->rot.y.z,hom->rot.x.z); 
	
						t[0] = h;
						t[1] = h+PM_PI;
						t[2] = h-PM_PI;
						t[3] = h+2*PM_PI;
						t[4] = h-2*PM_PI; 
						
						d = fabs(h-pj); 
						fg = 0;
						
						for(r=1; r<=4; r++)
						{
							if(fabs(t[r]-pj)-fabs(t[fg]-pj)<1e-6) 
								fg = r;
						}
						
						th4 = pre_joint[3]*PM_PI/180 + (t[fg]-pj)/2;
						th6 = pre_joint[5]*PM_PI/180 + (t[fg]-pj)/2;
	
						for(r=0; r<2; r++)
						{
							g[i*4+k*2+r][3] = th4*180/PM_PI;
							g[i*4+k*2+r][4] = th5*180/PM_PI;
							g[i*4+k*2+r][5] = th6*180/PM_PI;
						}
				
				}
			
		}
	
			}	
		for (i = 0; i < 8; i++)
		{
			double tempjoint[6];
			RobotPose tempWorld;
			PmRotationMatrix tempEhom;
			KINEMATICS_FORWARD_FLAGS fflags;
			KINEMATICS_INVERSE_FLAGS iflags;
			int num = 0;
			for (j = 0; j < 6; j++)
			{
				if (1 == j)
				{
					tempjoint[j] = g[i][j] + 90.0;
				}
				else
				{
					tempjoint[j] = g[i][j];
				}
				
			}
			kinematicsForward(tempjoint, &tempWorld, &tempEhom, &fflags, &iflags);
			if (1==flag&&matricIsEqual(tempEhom, hom->rot) != 0)
			{
				skipFlag[i] = 1;
			}
			else
			{
				skipFlag[i] = 0;
			}
		}
	
	
	
		
		//求出8组解，计算最优解
		Flag = 0;
		for(i=0; i<MN; i++)
		{
			if (1 == skipFlag[i])
			{
				dsum[i] = 1.79769e+308;
				continue;
			}
			for(j=0; j<5; j++)
			{
		//		rtapi_print("%d ",(int)(g[i][j]));
		//		rtapi_print("aaa%d	",(int)(pre_joint[j]));
				if (j == 1)
				{
					dsum[i] += (g[i][j]+ 90 - pre_joint[j])*(g[i][j]+ 90 - pre_joint[j]);
				}
				else
				{
					dsum[i] += (g[i][j]-pre_joint[j])*(g[i][j]-pre_joint[j]);
				}
					
				
				if(j==5) 
				{
	//				printf("	dsum=%6f	",dsum[i]);
		//			rtapi_print("\n");
				}
			}
			if(dsum[i]-dsum[Flag]<0 ) Flag = i;
			
		}
		
			if( 	g[Flag][0]-181>1e-6 || g[Flag][0]+181<1e-6
				|| g[Flag][1]-181>1e-6 || g[Flag][1]+181<1e-6
				|| g[Flag][2]-181>1e-6 || g[Flag][2]+181<1e-6
				|| g[Flag][3]-181>1e-6 || g[Flag][3]+181<1e-6 
				|| g[Flag][4]-181>1e-6 || g[Flag][4]+181<1e-6
				|| g[Flag][5]-360>1e-6 || g[Flag][5]+360<1e-6  )
			{
	
				return -1;
			}
		
	//	Flag = 1;
	
	//	printf("\nFlag=%d\n",Flag);
	
		if(dsum[Flag]>10000)
		{
		//	goto LabelErr;
			return -1;
		}
	
	//	printf("\n最优解为：\n");
		for(i=0; i<6; i++)
		{
	//		if(1==i){
	//		joint[i] = g[Flag][i]+90;}
			
			joint[i] = g[Flag][i];
				  if(1==i){
			joint[i] = g[Flag][i]+90.0;}
		}
	
	//	  printf("\n\n");
		if (pre_joint[5] > 170.0&&joint[5]<0.0)
		{
			joint[5] += 360.0;
		}
		else if (pre_joint[5] < -170.0&&joint[5]>0.0)
		{
			joint[5] -= 360.0;
		}
	
		*/
	
	
	
	
		return 0;

}






