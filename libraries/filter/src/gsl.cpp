/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>
#include "gsl.h"
#include <global.h>
#include <stdarg.h>

namespace vlr {

gsl_vector *vector_init(int length)
{
  gsl_vector *vector;

  vector = gsl_vector_calloc(length);
  dgc::dgc_test_alloc(vector);
  return vector;
}

void vector_free(gsl_vector *vector)
{
  gsl_vector_free(vector);
}

gsl_matrix *matrix_init(int rows, int cols)
{
  gsl_matrix *matrix;

  matrix = gsl_matrix_calloc(rows, cols);
  dgc::dgc_test_alloc(matrix);
  return matrix;
}

void matrix_free(gsl_matrix *matrix)
{
  gsl_matrix_free(matrix);
}

void vector_add(gsl_vector *result, gsl_vector *a, gsl_vector *b)
{
  gsl_vector_memcpy(result, a);
  gsl_vector_add(result, b);
}

void vector_add_ip(gsl_vector *a, gsl_vector *b)
{
  gsl_vector_add(a, b);
}

void vector_elem_add(gsl_vector *v, int elem, double x)
{
  gsl_vector_set(v, elem, gsl_vector_get(v, elem) + x);
}

void vector_subtract(gsl_vector *result, gsl_vector *a, gsl_vector *b)
{
  gsl_vector_memcpy(result, a);
  gsl_vector_sub(result, b);
}

void vector_subtract_ip(gsl_vector *a, gsl_vector *b)
{
  gsl_vector_sub(a, b);
}

void matrix_add(gsl_matrix *result, gsl_matrix *a, gsl_matrix *b)
{
  gsl_matrix_memcpy(result, a);
  gsl_matrix_add(result, b);
}

void matrix_add_ip(gsl_matrix *a, gsl_matrix *b)
{
  gsl_matrix_add(a, b);
}

void matrix_elem_add(gsl_matrix *m, int row, int col, double x)
{
  gsl_matrix_set(m, row, col, gsl_matrix_get(m, row, col) + x);
}

void matrix_subtract(gsl_matrix *result, gsl_matrix *a, gsl_matrix *b)
{
  gsl_matrix_memcpy(result, a);
  gsl_matrix_sub(result, b);
}

void matrix_subtract_ip(gsl_matrix *a, gsl_matrix *b)
{
  gsl_matrix_sub(a, b);
}

void matrix_multiply(gsl_matrix *result, gsl_matrix *a, int a_t,
                            gsl_matrix *b, int b_t)
{
  gsl_blas_dgemm(a_t?CblasTrans:CblasNoTrans, b_t?CblasTrans:CblasNoTrans,
                 1.0, a, b, 0.0, result);
}

void matrix_invert(gsl_matrix *inverse, gsl_matrix *m)
{
  gsl_matrix *A;
  gsl_permutation *P;
  int signum;

  A = matrix_init(m->size1, m->size2);
  gsl_matrix_memcpy(A, m);
  P = gsl_permutation_calloc(m->size1);
  dgc::dgc_test_alloc(P);
  gsl_linalg_LU_decomp(A, P, &signum);
  gsl_linalg_LU_invert(A, P, inverse);
  gsl_matrix_free(A);
  gsl_permutation_free(P);
}

void matrix_invert_ip(gsl_matrix *m)
{
  gsl_matrix *A;
  gsl_permutation *P;
  int signum;

  A = matrix_init(m->size1, m->size2);
  gsl_matrix_memcpy(A, m);
  P = gsl_permutation_calloc(m->size1);
  dgc::dgc_test_alloc(P);
  gsl_linalg_LU_decomp(A, P, &signum);
  gsl_linalg_LU_invert(A, P, m);
  gsl_matrix_free(A);
  gsl_permutation_free(P);
}

void matrix_vector_multiply(gsl_vector *result, gsl_matrix *a, gsl_vector *x)
{
  gsl_blas_dgemv(CblasNoTrans, 1.0, a, x, 0.0, result);
}

double vector_inner_product(gsl_vector *a, gsl_vector *b)
{
  double result;

  gsl_blas_ddot(a, b, &result);
  return result;
}

double matrix_determinant(gsl_matrix *A)
{
  int signum = 0;
  double det;
  gsl_matrix *M = matrix_init(A->size1, A->size2);
  gsl_permutation *p;

  gsl_matrix_memcpy(M, A);
  p = gsl_permutation_calloc(M->size1);
  gsl_linalg_LU_decomp(M, p, &signum);
  det = gsl_linalg_LU_det(M, signum);
  gsl_permutation_free(p);
  matrix_free(M);
  return(det);
}

void matrix_linear_transform(gsl_matrix *result, gsl_matrix *m, gsl_matrix *a)
{
  gsl_matrix *temp = matrix_init(m->size1, a->size1);

  matrix_multiply(temp, m, 0, a, 1);
  matrix_multiply(result, a, 0, temp, 0);
  matrix_free(temp);
}

/* A' inv(M) A */

void matrix_inverse_linear_transform(gsl_matrix *result, gsl_matrix *m,
					    gsl_matrix *a)
{
  gsl_matrix *temp1 = matrix_init(m->size1, m->size2);
  gsl_matrix *temp2 = matrix_init(m->size1, a->size2);

  matrix_invert(temp1, m);
  matrix_multiply(temp2, temp1, 0, a, 0);
  matrix_multiply(result, a, 1, temp2, 0);
  matrix_free(temp1);
  matrix_free(temp2);
}

void vector_fill(gsl_vector *v, ...)
{
  va_list args;
  unsigned int i;

  va_start(args, v);
  for(i = 0; i < v->size; i++)
    gsl_vector_set(v, i, va_arg(args, double));
  va_end(args);
}

void matrix_fill(gsl_matrix *m, ...)
{
  va_list args;
  unsigned int r, c;

  va_start(args, m);
  for(r = 0; r < m->size1; r++)
    for(c = 0; c < m->size2; c++)
      gsl_matrix_set(m, r, c, va_arg(args, double));
  va_end(args);
}

void matrix_scale(gsl_matrix *m, double scale)
{
  gsl_matrix_scale(m, scale);
}

void matrix_print(gsl_matrix *m, const char *str)
{
  unsigned int i, j;

  fprintf(stderr, "\n%s\n\n", str);
  for(i = 0; i < m->size1; i++) {
    for(j = 0; j < m->size2; j++)
      fprintf(stderr, "%8.3f ", gsl_matrix_get(m, i, j));
    fprintf(stderr, "\n");
  }
}

void matrix_print_signs(gsl_matrix *m, const char *str)
{
  int r, c;

  fprintf(stderr, "\n%s\n\n", str);
  for(r = 0; r < (int)m->size1; r++) {
    for(c = 0; c < (int)m->size2; c++)
      if(gsl_matrix_get(m, r, c) < 0)
        fprintf(stderr, "- ");
      else if(gsl_matrix_get(m, r, c) > 0)
        fprintf(stderr, "+ ");
      else
        fprintf(stderr, "0 ");
    fprintf(stderr, "\n");
  }
}

void vector_print(gsl_vector *v, const char *str)
{
  unsigned int i;

  fprintf(stderr, "\n%s\n\n", str);
  for(i = 0; i < v->size; i++)
    fprintf(stderr, "%8.6f ", gsl_vector_get(v, i));
  fprintf(stderr, "\n");
}

void matrix_to_file(gsl_matrix *m, const char *filename)
{
  unsigned int i, j;
  FILE *fp;

  fp = fopen(filename, "w");
  for(i = 0; i < m->size1; i++) {
    for(j = 0; j < m->size2; j++)
      fprintf(fp, "%f ", gsl_matrix_get(m, i, j));
    fprintf(fp, "\n");
  }
  fclose(fp);
}

void vector_to_file(gsl_vector *v, const char *filename)
{
  unsigned int i;
  FILE *fp;

  fp = fopen(filename, "w");
  for(i = 0; i < v->size; i++)
    fprintf(fp, "%f\n", gsl_vector_get(v, i));
  fclose(fp);
}

} // namespace vlr

