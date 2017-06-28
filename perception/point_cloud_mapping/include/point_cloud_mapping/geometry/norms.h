/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: norms.h 16379 2009-05-29 19:20:46Z hsujohnhsu $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_NORMS_H_
#define _CLOUD_GEOMETRY_NORMS_H_

namespace cloud_geometry
{

  namespace norms
  {
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the L1 norm between two nD points (aka aka Manhattan norm,
    * rectilinear distance, Minkowski's L1 distance, taxi cab metric, or city
    * block distance)
    * L1 = Sum (|x_i|), i=1..n
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      L1_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += fabs (A[i] - B[i]);

      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the squared L2 norm between two nD points (aka Euclidean metric)
    * L2_SQR = Sum (|x_i|^2), i=1..n
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      L2_Norm_SQR (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += (A[i] - B[i]) * (A[i] - B[i]);

      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the L2 norm between two nD points (aka Euclidean metric)
    * L2 = SQRT (Sum (|x_i|^2)), i=1..n
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      L2_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += (A[i] - B[i]) * (A[i] - B[i]);

      return sqrt (norm);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Linf norm between two nD points (aka Minkowski distance,
    * Chebyshev norm, or supremum norm)
    * Linf = max(|xi|), i=1..n
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      Linf_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm = (fabs (A[i] - B[i]) > norm) ? fabs (A[i] - B[i]) : norm;

      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Jeffries-Matusita (JM) distance between two nD points (aka
    * Hellinger distance)
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      JM_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += (sqrt (A[i]) - sqrt (B[i])) * (sqrt (A[i]) - sqrt (B[i]));

      return sqrt (norm);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Bhattacharyya (B) distance between two nD points
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      B_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0, result;

      for (int i = 0; i < dim; i++)
        norm += sqrt (A[i] * B[i]);

      if (norm > 0)
        result = -log (norm);
      else
        result = 0;

      return result;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Sublinear kernel distance between two nD points
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      Sublinear_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += sqrt (fabs (A[i] - B[i]));

      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Chi-Square (CS) distance between two nD points
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      CS_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        if ((A[i] + B[i]) != 0)
          norm += (A[i] - B[i]) * (A[i] - B[i]) / (A[i] + B[i]);
        else
          norm += 0;
      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Divergence distance between two nD points
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      Div_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        if ((A[i] / B[i]) > 0)
          norm += (A[i] - B[i]) * log (A[i] / B[i]);
        else
          norm += 0;
      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Patrick-Fisher distance between two nD points (Same as L2 ! - when P1 = P2)
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    * \param P1 P1
    * \param P2 P2
    */
    inline double
      PF_Norm (float *A, float *B, int dim, double P1, double P2)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += (P1 * A[i] - P2 * B[i]) * (P1 * A[i] - P2 * B[i]);
      return sqrt (norm);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Kolmogorov distance between two nD points (Same as L1 ! - when P1 = P2)
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    * \param P1 P1
    * \param P2 P2
    */
    inline double
      K_Norm (float *A, float *B, int dim, double P1, double P2)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        norm += fabs (P1 * A[i] - P2 * B[i]);
      return norm;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the Kullback-Leibler distance between two nD points
    * \param A first point
    * \param B second point
    * \param dim number of dimensions
    */
    inline double
      KL_Norm (float *A, float *B, int dim)
    {
      double norm = 0.0;

      for (int i = 0; i < dim; i++)
        if ( (B[i] != 0) && ((A[i] / B[i]) > 0) )
          norm += A[i] * log (A[i] / B[i]);
        else
          norm += 0;
      return norm;
    }
  }
}

#endif
