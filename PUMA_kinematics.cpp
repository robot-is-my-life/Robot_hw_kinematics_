#include <stdio.h>
#include <math.h>

// for rand
#include <stdlib.h> 
#include <time.h> //time()

extern void multiply_matrix_4x4(double  first[][4], double  second[][4], double  result[][4]);		// 4X4 행렬 곱셈 계산기.
extern void copy_matrix_A2B_4x4(double A[][4], double B[][4]);										// 4X4 행렬 매트릭스 카피
extern void print_matrix_4x4(double matrix[][4]);													// 4X4 행렬 출력
extern void init_matrix_4x4(double matrix[][4]);													// 4X4 행렬 초기화.

#define DEGREE2RAD 0.017453292
#define RAD2DEGREE 57.29577951

double T[6][4][4] = { '\0', };
double theta[6] = { 0,0,-90,0,0,0 };
double d[6] = { 1,1,0,1,0,1 };
double alpha[6] = { 90,0,-90,90,-90,0 };
double a[6] = { 0,1,0,0,0,0 };


int PUMA_kinematics(int start, int end, double  result[][4], double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);
extern int PUMA_invers_kinematics(double  kinematics_result[][4], double  result_inv[6]);

extern void PUMA_kinematics_easy(double output[][4]);
void main()
{
    double    result[4][4] = { 0, }; // kinematics 출력 (4x4 matrix) 
    double    result_inv[6] = { 0, }; // inverse kinematics 출력 (각도값)
/*
    //랜덤변수 입력 0~90
    srand((unsigned int)time(NULL));
    
    theta_1 = rand() % 90;
    theta_2 = rand() % 90;
    theta_3 = rand() % 90;
    theta_4 = rand() % 90;
    theta_5 = rand() % 90;
    theta_6 = rand() % 90;
    */
    
    

    PUMA_kinematics_easy(result);
    
    
    //PUMA_kinematics(0, 6, result, theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]);
     
    printf("Kinematics!!!!\n\n");
    printf("n \t\to \t\ta \t\tp \n");
    print_matrix_4x4(result);

   
    
    printf("Inverse Kinematics!!!!\n\n");
    PUMA_invers_kinematics(result, result_inv);


    printf("\n해 검증 (Inverse Kinematics함수로 출력된 각도값을 다시 Kinematics 모델에 대입) \n");
    for (int i = 0; i < 6; i++)
    {
        theta[i] = result_inv[i];
    }
    PUMA_kinematics_easy(result);
    //PUMA_kinematics(0, 6, result, result_inv[0], result_inv[1], result_inv[2], result_inv[3], result_inv[4], result_inv[5]);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%f \t", result[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    
}



//221022 기준 이제 미사용, 너무 복잡한 수식,
int PUMA_kinematics(int start, int end, double  result[][4], double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6)
{
    if (end - start < 2)
    {
        printf("잘못된 입력\n");
        return 0;
    }
    double  T_0to1[4][4] = {
                          {cos(DEGREE2RAD * theta_1)  ,0                               ,sin(DEGREE2RAD * theta_1)       ,0                               },
                          {sin(DEGREE2RAD * theta_1)  ,0                               ,-1 * cos(DEGREE2RAD * theta_1)  ,0                               },
                          {0                          ,1                               ,0                               ,d[1]                             },
                          {0                          ,0                               ,0                               ,1                               } };

    double  T_1to2[4][4] = {
                               {cos(DEGREE2RAD * theta_2)  ,-1 * sin(DEGREE2RAD * theta_2)  ,0                               ,a[2] * cos(DEGREE2RAD * theta_2) },
                               {sin(DEGREE2RAD * theta_2)  ,cos(DEGREE2RAD * theta_2)       ,0                               ,a[2] * sin(DEGREE2RAD * theta_2) },
                               {0                          ,0                               ,1                               ,d[2]                             },
                               {0                          ,0                               ,0                               ,1                               } };

    double  T_2to3[4][4] = {
                               {cos(DEGREE2RAD * theta_3)   ,0                              ,-1 * sin(DEGREE2RAD * theta_3)  ,a[3] * cos(DEGREE2RAD * theta_3)                               },
                               {sin(DEGREE2RAD * theta_3)   ,0                              ,cos(DEGREE2RAD * theta_3)       ,a[3] * sin(DEGREE2RAD * theta_3)                               },
                               {0                           ,-1                             ,0                               ,0                               },
                               {0                           ,0                              ,0                               ,1                               } };

    double  T_3to4[4][4] = {
                               {cos(DEGREE2RAD * theta_4)   ,0                              ,sin(DEGREE2RAD * theta_4)       ,0                               },
                               {sin(DEGREE2RAD * theta_4)   ,0                              ,-1 * cos(DEGREE2RAD * theta_4)  ,0                               },
                               {0                           ,1                              ,0                               ,d[4]                             },
                               {0                           ,0                              ,0                               ,1                               } };

    double  T_4to5[4][4] = {
                               {cos(DEGREE2RAD * theta_5)   ,0                              ,-1 * sin(DEGREE2RAD * theta_5)  ,0                               },
                               {sin(DEGREE2RAD * theta_5)   ,0                              ,cos(DEGREE2RAD * theta_5)       ,0                               },
                               {0                           ,-1                             ,0                               ,0                               },
                               {0                           ,0                              ,0                               ,1                               } };

    double  T_5to6[4][4] = {
                               {cos(DEGREE2RAD * theta_6)   ,-1 * sin(DEGREE2RAD * theta_6) ,0                               ,0                               },
                               {sin(DEGREE2RAD * theta_6)   ,cos(DEGREE2RAD * theta_6)      ,0                               ,0                               },
                               {0                           ,0                              ,1                               ,d[6]                             },
                               {0                           ,0                              ,0                               ,1                               } };


    double  arr_tmp[4][4] = { 0 };
    double  arr_tmp1[4][4] = { 0 };
    double  arr_tmp2[4][4] = { 0 };
    double  arr_tmp3[4][4] = { 0 };
    double  arr_tmp4[4][4] = { 0 };
    double  arr_tmp5[4][4] = { 0 };
    double  arr_tmp6[4][4] = { 0 };

    //print_matrix_4x4(T_2to3);

    if (start == 0)
    {
        multiply_matrix_4x4(T_0to1, T_1to2, arr_tmp);
        multiply_matrix_4x4(arr_tmp, T_2to3, arr_tmp1);
        multiply_matrix_4x4(arr_tmp1, T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 1)
    {
        multiply_matrix_4x4(T_1to2, T_2to3, arr_tmp1);
        multiply_matrix_4x4(arr_tmp1, T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 2)
    {
        multiply_matrix_4x4(T_2to3, T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 3)
    {
        multiply_matrix_4x4(T_3to4, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 4)
    {
        multiply_matrix_4x4(T_4to5, T_5to6, arr_tmp4);
    }



    if (end == 2)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[i][j] = arr_tmp[i][j];
            }
        }

    }
    else if (end == 3)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[i][j] = arr_tmp1[i][j];
            }
        }
    }
    else if (end == 4)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[i][j] = arr_tmp2[i][j];
            }
        }
    }
    else if (end == 5)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[i][j] = arr_tmp3[i][j];
            }
        }
    }
    else if (end == 6)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[i][j] = arr_tmp4[i][j];
            }
        }
    }
    return 1;
}









