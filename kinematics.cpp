#include <stdio.h>
#include <math.h>


void multiply_matrix_4x4(double  first[][4], double  second[][4], double  result[][4]); // 4X4 행렬 곱셈 계산기.

#define DEGREE2RAD 0.017453292
#define RAD2DEGREE 57.29577951
double  theta_1 = 70;
double  theta_2 = 70;
double  theta_3 = 70;
double  theta_4 = 44;
double  theta_5 = 99.55;
double  theta_6 = 0;

double  d_1 = 2;
double  d_2 = 1;
double  d_3 = 0;
double  d_4 = 1;
double  d_5 = 0;
double  d_6 = 1;

double  a_1 = 0;
double  a_2 = 1;
double  a_3 = 0;
double  a_4 = 0;
double  a_5 = 0;
double  a_6 = 0;

double  T_0to1[4][4] = { 
                           {cos(DEGREE2RAD * theta_1)  ,0                               ,sin(DEGREE2RAD * theta_1)       ,0                               },
                           {sin(DEGREE2RAD * theta_1)  ,0                               ,-1 * cos(DEGREE2RAD * theta_1)  ,0                               },
                           {0                          ,1                               ,0                               ,d_1                             },
                           {0                          ,0                               ,0                               ,1                               } };

double  T_1to2[4][4] = { 
                           {cos(DEGREE2RAD * theta_2)  ,-1 * sin(DEGREE2RAD * theta_2)  ,0                               ,a_2 * cos(DEGREE2RAD * theta_2) },
                           {sin(DEGREE2RAD * theta_2)  ,cos(DEGREE2RAD * theta_2)       ,0                               ,a_2 * sin(DEGREE2RAD * theta_2) },
                           {0                          ,0                               ,1                               ,d_2                             },
                           {0                          ,0                               ,0                               ,1                               } };

double  T_2to3[4][4] = { 
                           {cos(DEGREE2RAD * theta_3)   ,0                              ,-1 * sin(DEGREE2RAD * theta_3)  ,0                               },
                           {sin(DEGREE2RAD * theta_3)   ,0                              ,cos(DEGREE2RAD * theta_3)       ,0                               },
                           {0                           ,-1                             ,0                               ,0                               },
                           {0                           ,0                              ,0                               ,1                               } };

double  T_3to4[4][4] = { 
                           {cos(DEGREE2RAD * theta_4)   ,0                              ,sin(DEGREE2RAD * theta_4)       ,0                               },
                           {sin(DEGREE2RAD * theta_4)   ,0                              ,-1 * cos(DEGREE2RAD * theta_4)  ,0                               },
                           {0                           ,1                              ,0                               ,d_4                             },
                           {0                           ,0                              ,0                               ,1                               } };

double  T_4to5[4][4] = { 
                           {cos(DEGREE2RAD * theta_5)   ,0                              ,-1 * sin(DEGREE2RAD * theta_5)  ,0                               },
                           {sin(DEGREE2RAD * theta_5)   ,0                              ,cos(DEGREE2RAD * theta_5)       ,0                               },
                           {0                           ,-1                             ,0                               ,0                               },
                           {0                           ,0                              ,0                               ,1                               } };

double  T_5to6[4][4] = { 
                           {cos(DEGREE2RAD * theta_6)   ,-1 * sin(DEGREE2RAD * theta_6) ,0                               ,0                               },
                           {sin(DEGREE2RAD * theta_6)   ,cos(DEGREE2RAD * theta_6)      ,0                               ,0                               },
                           {0                           ,0                              ,1                               ,d_6                             },
                           {0                           ,0                              ,0                               ,1                               } };


int PUMA_kinematics(int start, int end , double  result[][4]);
int PUMA_invers_kinematics(double  kinematics_result[][4], double  result_inv[6]);

void main()
{
   /* double  a[4][4] = {{1, 3, 5, 7}, {4, 7, 1, 2}, {3, 4, 5, 6}, {4, 7, 5, 7}};
    double  b[4][4] = { {2, 4, 6, 8}, {1, 3, 5, 7}, {1, 2, 2, 9}, {1, 9, 9 ,8} };

        for (int i = 0; i < 1; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                printf("%f \t", a[i][j]);
            }
            printf("\n");
        }
        */


    double    result[4][4]  = { 0, }; // kinematics 출력 (4x4 matrix) 
    double    result_inv[6] = { 0, }; // inverse kinematics 출력 (각도값)



    
    //multiply_matrix_4x4(a, b, result);

    // for debug

    /*
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (PUMA_kinematics(i, j, result))
            {
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
        }

    }
    */
    // for debug
    
    //printf("sin 1/2 : %f \n " ,sin(3.141592 / 6));
    //printf("degree to rad : %f \n ", sin(DEGREE2RAD*30));
    //printf("rad to degree : %f \n ", 3.141592 /6 *RAD2DEGREE);
    //printf("atan 30도 : %f \n ", RAD2DEGREE*atan2(1,sqrt(3)));
    PUMA_kinematics(0, 6, result);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%f \t", result[i][j]);
        }
        printf("\n");
    }
    printf("\n");


    PUMA_invers_kinematics(result, result_inv);
}


int PUMA_kinematics(int start , int end , double  result[][4])
{
    if (end - start < 2 )
    {
        printf("잘못된 입력\n");
        return 0;
    }

    
    
    double  arr_tmp[4][4]  = { 0 };
    double  arr_tmp1[4][4] = { 0 };
    double  arr_tmp2[4][4] = { 0 };
    double  arr_tmp3[4][4] = { 0 };
    double  arr_tmp4[4][4] = { 0 };
    double  arr_tmp5[4][4] = { 0 };
    double  arr_tmp6[4][4] = { 0 };
    


    if (start == 0)
    {
        multiply_matrix_4x4(T_0to1  , T_1to2, arr_tmp);
        multiply_matrix_4x4(arr_tmp , T_2to3, arr_tmp1);
        multiply_matrix_4x4(arr_tmp1, T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 1)
    {
        multiply_matrix_4x4(T_1to2  , T_2to3, arr_tmp1);
        multiply_matrix_4x4(arr_tmp1, T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 2)
    {
        multiply_matrix_4x4(T_2to3  , T_3to4, arr_tmp2);
        multiply_matrix_4x4(arr_tmp2, T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 3)
    {
        multiply_matrix_4x4(T_3to4  , T_4to5, arr_tmp3);
        multiply_matrix_4x4(arr_tmp3, T_5to6, arr_tmp4);
    }
    else if (start == 4)
    {
        multiply_matrix_4x4(T_4to5  , T_5to6, arr_tmp4);
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




int PUMA_invers_kinematics(double  kinematics_result[][4], double  result_inv[6])
{
    // Piper's Rule 만족하는 손목좌표 x y z
    double  P_x = kinematics_result[0][3] - d_6 * kinematics_result[0][2];
    double  P_y = kinematics_result[1][3] - d_6 * kinematics_result[1][2];
    double  P_z = kinematics_result[2][3] - d_6 * kinematics_result[2][2];

    //double arr_for_debug[4][4] = 0;
    //PUMA_kinematics(0,3)
    //printf("Px_err : ", P_x - );

  


    double theta1[2] = { atan2(P_y * sqrt(pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)) + P_x * d_2,
                    P_x * sqrt(pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)) - P_y * d_2)
                        ,
                        atan2(-1 * P_y * sqrt(pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)) + P_x * d_2,
                     -1 * P_x * sqrt(pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)) - P_y * d_2)
    };


    double theta3[2] = { atan2(-1 * (pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d_1, 2) - pow(d_4, 2) - pow(a_2, 2) - pow(d_2, 2)),
                     sqrt(
                         4 * pow(a_2,2) * pow(d_4,2) - 1 * pow(
                                                            pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d_1, 2) - pow(d_4, 2) - pow(a_2, 2) - pow(d_2, 2),
                                                            2
                                                            )
                         )
                     )
        ,
                        atan2(-1 * (pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d_1, 2) - pow(d_4, 2) - pow(a_2, 2) - pow(d_2, 2)),
                     -1 * sqrt(
                         4 * pow(a_2,2) * pow(d_4,2) - 1 * pow(
                                                            pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d_1, 2) - pow(d_4, 2) - pow(a_2, 2) - pow(d_2, 2),
                                                            2
                                                            )
                         )
                     ) };
    
    /*/double theta2[4] = {atan2(
                                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[0])) - sqrt(
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                                      ) * d_4 * cos(theta3[0]) ,
                                    (P_z - d_1) * d_4 * cos(theta3[0]) + sqrt(
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                            ) * (a_2 - d_4 * sin(theta3[0]))
                                     )
                         ,
                            atan2(
                                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[0])) - sqrt(
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                                      ) * d_4 * cos(theta3[0]) ,
                                    (P_z - d_1) * d_4 * cos(theta3[0]) + sqrt(
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                            ) * (a_2 - d_4 * sin(theta3[0]))
                                     )
                            ,
                             atan2(
                                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[1])) + sqrt(
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                                      ) * d_4 * cos(theta3[1]) ,
                                    (P_z - d_1) * d_4 * cos(theta3[1]) - sqrt(
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                            ) * (a_2 - d_4 * sin(theta3[1]))
                                     )
                            ,
                            atan2(
                                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[1])) + sqrt(
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                                      ) * d_4 * cos(theta3[1]) ,
                                    (P_z - d_1) * d_4 * cos(theta3[1]) - sqrt(
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                                                                            ) * (a_2 - d_4 * sin(theta3[1]))
                                     )
    };*/
    double theta2[4];
    for (int i = 0; i < 2; i++)     // theta3 loop
    {
        for (int j = 0; j < 2; j++) // theta2 loop
        {
            if (j)
            {
                theta2[2 * i + j] = atan2(
                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[i])) + sqrt(
                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                    ) * d_4 * cos(theta3[i]),
                    (P_z - d_1) * d_4 * cos(theta3[i]) - sqrt(
                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                    ) * (a_2 - d_4 * sin(theta3[i]))
                );
            }
            else
            {
                theta2[2 * i + j] = atan2(
                    (P_z - d_1) * (a_2 - d_4 * sin(theta3[i])) - sqrt(
                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                    ) * d_4 * cos(theta3[i]),
                    (P_z - d_1) * d_4 * cos(theta3[i]) + sqrt(
                        pow(P_x, 2) + pow(P_y, 2) - pow(d_2, 2)
                    ) * (a_2 - d_4 * sin(theta3[i]))
                );
            }
           
        }
    }


    double theta4[16] = { 0, };
    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    if (z) 
                    {
                        theta4[8 * i + 4 * j + 2 * k + z] = atan2(-1*(sin(theta1[i]) * kinematics_result[0][2] - cos(theta1[i]) * kinematics_result[1][2])
                            ,
                            (sin(theta2[2 * j + k] + theta3[j]) * kinematics_result[2][2] +
                            sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * kinematics_result[1][2] +
                            cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * kinematics_result[0][2]));
                    }
                    else
                    {
                        theta4[8 * i + 4 * j + 2 * k + z] = atan2((sin(theta1[i]) * kinematics_result[0][2] - cos(theta1[i]) * kinematics_result[1][2])
                            ,
                            -1 * (sin(theta2[2 * j + k] + theta3[j]) * kinematics_result[2][2] +
                            sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * kinematics_result[1][2] +
                            cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * kinematics_result[0][2]));
                    }
                    
                }
            }
        }
    }

    /*theta4[0] = atan2(-1 * (sin(theta1[1]) * kinematics_result[0][2] - cos(theta1[1]) * kinematics_result[1][2])
        ,
        1*((sin(theta2[1] + theta3[0]) * kinematics_result[2][2] +
            sin(theta1[1]) * cos(theta2[1] + theta3[0]) * kinematics_result[1][2] +
            cos(theta1[1]) * cos(theta2[1] + theta3[0]) * kinematics_result[0][2])));
    theta4[1] = atan2(1 * (sin(theta1[1]) * kinematics_result[0][2] - cos(theta1[1]) * kinematics_result[1][2])
        ,
        -1 * ((sin(theta2[1] + theta3[0]) * kinematics_result[2][2] +
            sin(theta1[1]) * cos(theta2[1] + theta3[0]) * kinematics_result[1][2] +
            cos(theta1[1]) * cos(theta2[1] + theta3[0]) * kinematics_result[0][2])));*/
    double theta5[16] = { 0, };
    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    theta5[8 * i + 4 * j + 2 * k + z] = atan2(- 1 * kinematics_result[0][2] * (cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) - sin(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                                                              - 1 * kinematics_result[1][2] * (sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) + cos(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                                                              - 1 * kinematics_result[2][2] * (sin(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                        ,
                          kinematics_result[0][2] * (-1 * cos(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[1][2] * (-1 * sin(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[2][2] * cos(theta2[2 * j + k] + theta3[j])
                    );

                }
            }
        }
    }

    double theta6[16] = { 0, };
    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    theta6[8 * i + 4 * j + 2 * k + z] = atan2(- 1 * kinematics_result[0][0] * (cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) - sin(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                                                              - 1 * kinematics_result[1][2] * (sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) + cos(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                                                              - 1 * kinematics_result[2][2] * (sin(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                        ,
                        kinematics_result[0][2] * (-1 * cos(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[1][2] * (-1 * sin(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[2][2] * cos(theta2[2 * j + k] + theta3[j])
                    );

                }
            }
        }
    }






    printf("theta1_p : %f \t theta1_n : %f \n", RAD2DEGREE * theta1[0], RAD2DEGREE * theta1[1]);
    printf("theta2_p_theta3_p : %f \t theta2_n_theta3_p : %f \n", RAD2DEGREE * theta2[0], RAD2DEGREE * theta2[1]);
    printf("theta2_p_theta3_n : %f \t theta2_n_theta3_n : %f \n", RAD2DEGREE * theta2[2], RAD2DEGREE * theta2[3]);
    printf("theta3_p : %f \t theta3_n : %f \n \n", RAD2DEGREE * theta3[0], RAD2DEGREE * theta3[1]);

    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    printf("[%d] theta1 : %f theta2 : %f theta3 : %f theta4 : %f theta5 : %f\n", 8 * i + 4 * j + 2 * k + z,
                        RAD2DEGREE* theta1[i], RAD2DEGREE* theta2[2*j+k], RAD2DEGREE* theta3[j], RAD2DEGREE* theta4[8 * i + 4 * j + 2 * k + z], RAD2DEGREE* theta5[8 * i + 4 * j + 2 * k + z]);


                }
            }
        }
    }
    for (int i = 0; i < 16; i++)
    {
       
    }
    return 0;
}







void multiply_matrix_4x4(double  first[][4], double  second[][4], double  result[][4])
{
    for (int k = 0; k < 4; k++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                result[k][j] += first[k][i] * second[i][j];
            }
        }
    }
}
