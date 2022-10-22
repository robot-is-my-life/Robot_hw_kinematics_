#include <stdio.h>
#include <math.h>
#define DEGREE2RAD 0.017453292
#define RAD2DEGREE 57.29577951


extern void multiply_matrix_4x4(double  first[][4], double  second[][4], double  result[][4]);		// 4X4 행렬 곱셈 계산기.
extern void copy_matrix_A2B_4x4(double A[][4], double B[][4]);										// 4X4 행렬 매트릭스 카피
extern void print_matrix_4x4(double matrix[][4]);													// 4X4 행렬 출력
extern void init_matrix_4x4(double matrix[][4]);													// 4X4 행렬 초기화.



extern double T[6][4][4];
extern double theta[6];
extern double d[6];
extern double alpha[6];
extern double a[6];



void kinematics_matrix_DH_notation(double theta, double d, double alpha, double a, double matrix[][4]) // input : DH parameters & 4x4 matrix for memorized output
{
	double tmp[4][4]
		= { {cos(theta)	,-1 * sin(theta) * cos(alpha)	,sin(theta) * sin(alpha)		,a * cos(theta)	},
			{sin(theta)	,cos(theta) * cos(alpha)		,-1 * cos(theta) * sin(alpha)	,a * sin(theta)	},
			{0			,sin(alpha)						, cos(alpha)					,d				},
			{0			,0								,0								,1				} };


	//matrix copy
	copy_matrix_A2B_4x4(tmp, matrix);

}
void PUMA_kinematics_easy(double output[][4])
{



	double tmp[4][4] = { '\0', };
	double result[4][4] = { '\0', };
	for (int i = 0; i < 6; i++)
	{
		kinematics_matrix_DH_notation(DEGREE2RAD *theta[i], d[i], DEGREE2RAD * alpha[i], a[i], T[i]);
	}
	multiply_matrix_4x4(T[0], T[1], tmp);
	for (int i = 1; i < 5; i++)
	{
		multiply_matrix_4x4(tmp, T[i + 1], result);
		copy_matrix_A2B_4x4(result, tmp);
		init_matrix_4x4(result);
	}

	//printf("Kinematics! (easy form) !!!\n\n");
	//printf("n \t\to \t\ta \t\tp \n");
	//print_matrix_4x4(tmp);
	copy_matrix_A2B_4x4(tmp, output);
}