#include <stdio.h>
#include <math.h>

// for rand
#include <stdlib.h> 
#include <time.h> //time()




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


void copy_matrix_A2B_4x4(double A[][4], double B[][4])

{
    //matrix copy
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            B[i][j] = A[i][j];
        }
    }
}
void print_matrix_4x4(double matrix[][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%f \t", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");


}

void init_matrix_4x4(double matrix[][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            matrix[i][j] = 0;
        }
    }
}

double compare_matrix_4x4(double A[][4], double B[][4])
{
    double err_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
             err_sum += fabs(A[i][j]- B[i][j]);
        }
    }
    return err_sum;
}