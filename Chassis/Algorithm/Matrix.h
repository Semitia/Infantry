#ifndef __MATRIX_H
#define __MATRIX_H

#include<math.h>
#include<stdio.h>
#include<stdlib.h>

#define INV 3

/**
 * @brief 
 * @param m 行数
 * @param n 列数
 * @param matrix 矩阵
 */
typedef struct __Matrix_t{
    int m;
    int n;
    double matrix[INV][INV];
}Matrix_t;

Matrix_t get_I(int t);
Matrix_t inv_matrix(Matrix_t M);
Matrix_t tran_matrix(Matrix_t M);
Matrix_t matrix_init2(int m, int n);
void swap_col(int i, int j, Matrix_t *M);
void swap_row(int i, int j, Matrix_t *M);
void fprintf_matrix(Matrix_t *mat, FILE *fp);
Matrix_t add_matrix(Matrix_t m1, Matrix_t m2);
Matrix_t sub_matrix(Matrix_t m1, Matrix_t m2);
Matrix_t mul_matrix(Matrix_t m1, Matrix_t m2);
void add_row(int i, int j, double k, Matrix_t *M);
void add_col(int i, int j, double k, Matrix_t *M);
Matrix_t matrix_init(int m, int n, double matrix[][INV]);



#endif
