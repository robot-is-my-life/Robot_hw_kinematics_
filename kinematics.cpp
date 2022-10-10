#include <stdio.h>
#include <math.h>


void multiply_matrix_4x4(float first[][4], float second[][4], float result[][4]); // 4X4 Çà·Ä °ö¼À °è»ê±â.


float theta_1 = 0;
float theta_2 = 0;
float theta_3 = 0;
float theta_4 = 0;
float theta_5 = 0;
float theta_6 = 0;

float d_1 = 1;
float d_2 = 2;
float d_3 = 0;
float d_4 = 1;
float d_5 = 0;
float d_6 = 1;

float a_1 = 0;
float a_2 = 1;
float a_3 = 0;
float a_4 = 0;
float a_5 = 0;
float a_6 = 0;

float T_0to1[4][4] = { {cos(theta_1)    ,0                 ,sin(theta_1)       ,0                  },
                       {sin(theta_1)    ,0                 ,-1 * cos(theta_1)  ,0                  },
                       {0               ,1                 ,0                  ,d_1                },
                       {0               ,0                 ,0                  ,1                  } };

float T_1to2[4][4] = { {cos(theta_2)    ,-1 * sin(theta_2) ,0                  ,a_2 * cos(theta_2) },
                       {sin(theta_2)    ,cos(theta_2)      ,0                  ,a_2 * sin(theta_2) },
                       {0               ,0                 ,1                  ,d_2                },
                       {0               ,0                 ,0                  ,1                  } };

float T_2to3[4][4] = { {cos(theta_3)    ,0                 ,-1 * sin(theta_3)  ,0                  },
                       {sin(theta_3)    ,0                 ,cos(theta_3)       ,0                  },
                       {0               ,-1                ,0                  ,0                  },
                       {0               ,0                 ,0                  ,1                  } };

float T_3to4[4][4] = { {cos(theta_4)    ,0                 ,sin(theta_4)       ,0                  },
                       {sin(theta_4)    ,0                 ,-1 * cos(theta_4)  ,0                  },
                       {0               ,1                 ,0                  ,d_4                },
                       {0               ,0                 ,0                  ,1                  } };

float T_4to5[4][4] = { {cos(theta_5)    ,0                 ,-1 * sin(theta_5)  ,0                  },
                       {sin(theta_5)    ,0                 ,cos(theta_5)       ,0                  },
                       {0               ,-1                ,0                  ,0                  },
                       {0               ,0                 ,0                  ,1                  } };

float T_5to6[4][4] = { {cos(theta_6)    ,0                 ,-1 * sin(theta_6)  ,0                  },
                       {sin(theta_6)    ,0                 ,cos(theta_6)       ,0                  },
                       {0               ,0                 ,1                  ,d_6                },
                       {0               ,0                 ,0                  ,1                  } };

void main()
{
    float a[4][4] = { {1, 3, 5, 7}, {4, 7, 1, 2}, {3, 4, 5, 6}, {4, 7, 5, 7} };
    float b[4][4] = { {2, 4, 6, 8}, {1, 3, 5, 7}, {1, 2, 2, 9}, {1, 9, 9 ,8} };
    float result[4][4] = { 0 };

    multiply_matrix_4x4(a, b, result);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%d ", result[i][j]);
        }

        printf("\n");
    }

}




void multiply_matrix_4x4(float first[][4], float second[][4], float result[][4])
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