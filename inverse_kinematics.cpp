#include <stdio.h>
#include <math.h>

#define DEGREE2RAD  0.017453292
#define RAD2DEGREE  57.29577951
#define OFFSET      0.000001
extern double T[6][4][4];
extern double theta[6];
extern double d[6];
extern double alpha[6];
extern double a[6];

int PUMA_invers_kinematics(double  kinematics_result[][4], double  result_inv[6])
{
    // Piper's Rule 만족하는 손목좌표 x y z
    double  P_x = kinematics_result[0][3] - d[5] * kinematics_result[0][2];
    double  P_y = kinematics_result[1][3] - d[5] * kinematics_result[1][2];
    double  P_z = kinematics_result[2][3] - d[5] * kinematics_result[2][2];

    //double arr_for_debug[4][4] = 0;
    //PUMA_kinematics(0,3)
    //printf("Px_err : ", P_x - );




    double theta1[2] = { atan2(P_y * sqrt(OFFSET+pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)) + P_x * d[1],
                    P_x * sqrt(OFFSET+pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)) - P_y * d[1])
                        ,
                        atan2(-1 * P_y * sqrt(OFFSET+pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)) + P_x * d[1],
                     -1 * P_x * sqrt(OFFSET+pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)) - P_y * d[1])
    };


    double theta3[2] = { atan2(-1 * (pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d[0], 2) - pow(d[3], 2) - pow(a[1], 2) - pow(d[1], 2)),
                     sqrt(OFFSET+
                         4 * pow(a[1],2) * pow(d[3],2) - 1 * pow(pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d[0], 2) - pow(d[3], 2) - pow(a[1], 2) - pow(d[1], 2),2)
                         )
                     )
        ,
                        atan2(-1 * (pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d[0], 2) - pow(d[3], 2) - pow(a[1], 2) - pow(d[1], 2)),
                     -1 * sqrt(OFFSET+
                         4 * pow(a[1],2) * pow(d[3],2) - 1 * pow(
                                                            pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d[0], 2) - pow(d[3], 2) - pow(a[1], 2) - pow(d[1], 2),
                                                            2
                                                            )
                         )
                     ) };
    printf("\n\n  sqrt : %lf \n\n ", OFFSET +
        4 * pow(a[1], 2) * pow(d[3], 2) - 1 * pow(pow(P_x, 2) + pow(P_y, 2) + pow(P_z - d[0], 2) - pow(d[3], 2) - pow(a[1], 2) - pow(d[1], 2), 2)
    );
    /*/double theta2[4] = {atan2(
                                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[0])) - sqrt(OFFSET+
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                                      ) * d[3] * cos(theta3[0]) ,
                                    (P_z - d[0]) * d[3] * cos(theta3[0]) + sqrt(OFFSET+
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                            ) * (a[1] - d[3] * sin(theta3[0]))
                                     )
                         ,
                            atan2(
                                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[0])) - sqrt(OFFSET+
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                                      ) * d[3] * cos(theta3[0]) ,
                                    (P_z - d[0]) * d[3] * cos(theta3[0]) + sqrt(OFFSET+
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                            ) * (a[1] - d[3] * sin(theta3[0]))
                                     )
                            ,
                             atan2(
                                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[1])) + sqrt(OFFSET+
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                                      ) * d[3] * cos(theta3[1]) ,
                                    (P_z - d[0]) * d[3] * cos(theta3[1]) - sqrt(OFFSET+
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                            ) * (a[1] - d[3] * sin(theta3[1]))
                                     )
                            ,
                            atan2(
                                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[1])) + sqrt(OFFSET+
                                                                                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                                      ) * d[3] * cos(theta3[1]) ,
                                    (P_z - d[0]) * d[3] * cos(theta3[1]) - sqrt(OFFSET+
                                                                                pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                                                                            ) * (a[1] - d[3] * sin(theta3[1]))
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
                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[i])) + sqrt(OFFSET+
                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                    ) * d[3] * cos(theta3[i]),
                    (P_z - d[0]) * d[3] * cos(theta3[i]) - sqrt(OFFSET+
                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                    ) * (a[1] - d[3] * sin(theta3[i]))
                );
            }
            else
            {
                theta2[2 * i + j] = atan2(
                    (P_z - d[0]) * (a[1] - d[3] * sin(theta3[i])) - sqrt(OFFSET+
                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                    ) * d[3] * cos(theta3[i]),
                    (P_z - d[0]) * d[3] * cos(theta3[i]) + sqrt(OFFSET+
                        pow(P_x, 2) + pow(P_y, 2) - pow(d[1], 2)
                    ) * (a[1] - d[3] * sin(theta3[i]))
                );
            }

        }
    }


    double theta4[16] = { 0, };
    double theta5[16] = { 0, };
    double theta6[16] = { 0, };
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
                        theta4[8 * i + 4 * j + 2 * k + z] = atan2(-1 * (sin(theta1[i]) * kinematics_result[0][2] - cos(theta1[i]) * kinematics_result[1][2])
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
    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    theta5[8 * i + 4 * j + 2 * k + z] = atan2(
                        -1 * kinematics_result[0][2] * (cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) - sin(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                        - 1 * kinematics_result[1][2] * (sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]) + cos(theta1[i]) * sin(theta4[8 * i + 4 * j + 2 * k + z]))
                        - 1 * kinematics_result[2][2] * (sin(theta2[2 * j + k] + theta3[j]) * cos(theta4[8 * i + 4 * j + 2 * k + z]))
                        ,
                        kinematics_result[0][2] * (-1 * cos(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[1][2] * (-1 * sin(theta1[i]) * sin(theta2[2 * j + k] + theta3[j]))
                        + kinematics_result[2][2] * cos(theta2[2 * j + k] + theta3[j])
                    );

                    double y5_x =
                        cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                        + sin(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z]);

                    double y5_y = sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                        - cos(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z]);

                    double y5_z = sin(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z]);

                    theta6[8 * i + 4 * j + 2 * k + z] = atan2(-1 * y5_x * kinematics_result[0][0] - 1 * y5_y * kinematics_result[1][0] - 1 * y5_z * kinematics_result[2][0]
                        ,
                        -1 * y5_x * kinematics_result[0][1] - 1 * y5_y * kinematics_result[1][1] - 1 * y5_z * kinematics_result[2][1]);
                    /*
                    theta6[8 * i + 4 * j + 2 * k + z] = atan2(
                        - 1 *  (
                            cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                            + sin(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z])
                            )* kinematics_result[0][0]
                        - 1 * (
                            sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                            - cos(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z])
                            ) * kinematics_result[1][0]
                        ,
                        - 1 * (
                            cos(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                            + sin(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z])
                            )* kinematics_result[0][1]
                        - 1 *  (
                            sin(theta1[i]) * cos(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                            - cos(theta1[i]) * cos(theta4[8 * i + 4 * j + 2 * k + z])
                            )* kinematics_result[1][1]
                        - 1 *  (
                            sin(theta2[2 * j + k] + theta3[j]) * sin(theta4[8 * i + 4 * j + 2 * k + z])
                            ) * kinematics_result[2][1]
                    );
                    */
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







            //printf("theta1_p : %f \t theta1_n : %f \n", RAD2DEGREE * theta1[0], RAD2DEGREE * theta1[1]);
            //printf("theta2_p_theta3_p : %f \t theta2_n_theta3_p : %f \n", RAD2DEGREE * theta2[0], RAD2DEGREE * theta2[1]);
            //printf("theta2_p_theta3_n : %f \t theta2_n_theta3_n : %f \n", RAD2DEGREE * theta2[2], RAD2DEGREE * theta2[3]);
            //printf("theta3_p : %f \t theta3_n : %f \n \n", RAD2DEGREE * theta3[0], RAD2DEGREE * theta3[1]);

    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    printf("[%d] theta1 : %.2lf theta2 : %.2lf theta3 : %.2lf theta4 : %.2lf theta5 : %.2lf theta6 : %.2lf\n", 8 * i + 4 * j + 2 * k + z,
                        RAD2DEGREE * theta1[i], RAD2DEGREE * theta2[2 * j + k], RAD2DEGREE * theta3[j],
                         RAD2DEGREE * theta4[8 * i + 4 * j + 2 * k + z], RAD2DEGREE * theta5[8 * i + 4 * j + 2 * k + z], RAD2DEGREE * theta6[8 * i + 4 * j + 2 * k + z]);
                }
            }
        }
    }



    double angle_err_sum[16] = { 0, };

    for (int i = 0; i < 2; i++)             // theta1 loop
    {
        for (int j = 0; j < 2; j++)         // theta3 loop
        {
            for (int k = 0; k < 2; k++)     // theta2 loop
            {
                for (int z = 0; z < 2; z++) // theta4 loop
                {
                    angle_err_sum[8 * i + 4 * j + 2 * k + z] =
                        fabs(DEGREE2RAD * theta[0] - theta1[i]) +
                        fabs(DEGREE2RAD * theta[1] - theta2[2 * j + k]) +
                        fabs(DEGREE2RAD * theta[2] - theta3[j]) +
                        fabs(DEGREE2RAD * theta[3] - theta4[8 * i + 4 * j + 2 * k + z]) +
                        fabs(DEGREE2RAD * theta[4] - theta5[8 * i + 4 * j + 2 * k + z]) +
                        fabs(DEGREE2RAD * theta[5] - theta6[8 * i + 4 * j + 2 * k + z]);
                }
            }
        }
    }
    double min = 100000;
    int min_num = 0;
    for (int i = 0; i < 16; i++)
    {
        if (min > angle_err_sum[i])
        {
            min = angle_err_sum[i];
            min_num = i;
        }
    }

    printf("\n입력한 각도\n ");
    printf("[Input] \ntheta1 : %.2lf \ntheta2 : %.2lf \ntheta3 : %.2lf \ntheta4 : %.2lf \ntheta5 : %.2lf \ntheta6 : %.2lf\n",
        theta[0], theta[1], theta[2],
        theta[3], theta[4], theta[5]);
    printf("\n입력한 각도와 가장 유사한 해는 \n ");
    printf("[  %d  ] \ntheta1 : %.2lf \ntheta2 : %.2lf \ntheta3 : %.2lf \ntheta4 : %.2lf \ntheta5 : %.2lf \ntheta6 : %.2lf\n", min_num,
        RAD2DEGREE * theta1[min_num / 8], RAD2DEGREE * theta2[min_num % 8 / 2], RAD2DEGREE * theta3[min_num % 8 / 4],
        RAD2DEGREE * theta4[min_num], RAD2DEGREE * theta5[min_num], RAD2DEGREE * theta6[min_num]);
    result_inv[0] = RAD2DEGREE * theta1[min_num / 8];
    result_inv[1] = RAD2DEGREE * theta2[min_num % 8 / 2];
    result_inv[2] = RAD2DEGREE * theta3[min_num % 8 / 4];
    result_inv[3] = RAD2DEGREE * theta4[min_num];
    result_inv[4] = RAD2DEGREE * theta5[min_num];
    result_inv[5] = RAD2DEGREE * theta6[min_num];
    printf("입니다. \n ");


    double result[4][4] = { 0, };

    return 0;
}