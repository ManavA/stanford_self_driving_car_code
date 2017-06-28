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


#ifndef GSL_SUPPORT_H
#define GSL_SUPPORT_H

#include <gsl/gsl_blas.h>

namespace vlr {

#define matrix_get gsl_matrix_get
#define vector_get gsl_vector_get
#define matrix_set gsl_matrix_set
#define vector_set gsl_vector_set
#define vector_copy gsl_vector_memcpy
#define matrix_copy gsl_matrix_memcpy
#define matrix_set_identity gsl_matrix_set_identity
#define vector_set_zero gsl_vector_set_zero
#define matrix_set_zero gsl_matrix_set_zero
#define vector_scale gsl_vector_scale

gsl_vector *vector_init(int length);
void vector_fill(gsl_vector *v, ...);
void vector_free(gsl_vector *vector);
gsl_matrix *matrix_init(int rows, int cols);
void matrix_fill(gsl_matrix *m, ...);
void matrix_free(gsl_matrix *matrix);
void vector_add(gsl_vector *result, gsl_vector *a, gsl_vector *b);
void vector_add_ip(gsl_vector *a, gsl_vector *b);
void vector_elem_add(gsl_vector *v, int elem, double x);
void vector_subtract(gsl_vector *result, gsl_vector *a,
                                gsl_vector *b);
void vector_subtract_ip(gsl_vector *a, gsl_vector *b);
void matrix_add(gsl_matrix *result, gsl_matrix *a, gsl_matrix *b);
void matrix_add_ip(gsl_matrix *a, gsl_matrix *b);
void matrix_elem_add(gsl_matrix *m, int row, int col, double x);
void matrix_subtract(gsl_matrix *result, gsl_matrix *a,
                                gsl_matrix *b);
void matrix_subtract_ip(gsl_matrix *a, gsl_matrix *b);
void matrix_multiply(gsl_matrix *result, gsl_matrix *a, int a_t,
                     gsl_matrix *b, int b_t);
void matrix_invert(gsl_matrix *inverse, gsl_matrix *m);
void matrix_invert_ip(gsl_matrix *m);
void matrix_vector_multiply(gsl_vector *result, gsl_matrix *a,
                                       gsl_vector *x);
void matrix_linear_transform(gsl_matrix *result, gsl_matrix *m,
                                        gsl_matrix *a);
void matrix_inverse_linear_transform(gsl_matrix *result,
                                                gsl_matrix *m, gsl_matrix *a);
void matrix_scale(gsl_matrix *m, double scale);
void matrix_print(gsl_matrix *m, const char *str);
void vector_print(gsl_vector *v, const char *str);
void matrix_print_signs(gsl_matrix *m, const char *str);
void matrix_to_file(gsl_matrix *m, const char *filename);
void vector_to_file(gsl_vector *v, const char *filename);
double vector_inner_product(gsl_vector *a, gsl_vector *b);
void gaussian_sample(gsl_vector *sample, gsl_vector *mean,
                                gsl_matrix *cov);
double matrix_determinant(gsl_matrix *A);

} // namespace vlr

#endif
