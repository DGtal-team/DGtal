/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file EigenValues3D.h
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/07/10
 *
 * Computes EigenValues and EigenVectors from 3D Matrix.
 *
 * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
 * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
 * Fortran subroutine in EISPACK.
 *
 * This file is part of the DGtal library.
 */


#if defined(EigenValues3D_RECURSES)
#error Recursive header files inclusion detected in EigenValues3D.h
#else // defined(EigenValues3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EigenValues3D_RECURSES

#if !defined EigenValues3D_h
/** Prevents repeated inclusion of headers. */
#define EigenValues3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/kernel/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

struct EigenValues3D
{
  typedef SimpleMatrix< double, 3, 3 > Matrix33;
  typedef Matrix33::RowVector Vector3;

  /**
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      */
  static void tred2 ( Matrix33 & V, Vector3 & d, Vector3 & e )
  {
    int dimension = 3;
    int dimensionMinusOne = 2;

    for ( int j = 0; j < dimension; ++j )
    {
      d[ j ] = V ( dimensionMinusOne, j );
    }

    // Householder reduction to tridiagonal form.

    for ( int i = dimensionMinusOne; i > 0; --i )
    {
      // Scale to avoid under/overflow.

      double scale = 0.0;
      double h = 0.0;
      for ( int k = 0; k < i; ++k )
      {
        scale += fabs( d[ k ] );
      }

      if ( scale == 0.0 )
      {
        e[ i ] = d[ i - 1 ];

        for ( int j = 0; j < i; ++j )
        {
          d[ j ] = V(( i - 1 ),  j );
          V.setComponent ( i, j, 0.0 );
          V.setComponent ( j, i, 0.0);
        }
      }
      else
      {
        // Generate Householder vector.
        for ( int k = 0; k < i; ++k )
        {
          d[ k ] /= scale;
          h += d[ k ] * d[ k ];
        }

        double f = d[ i - 1 ];
        double g = sqrt( h );

        if ( f > 0.0 )
        {
          g = -g;
        }

        e[ i ] = scale * g;
        h -= f * g;
        d[ i - 1 ] = f - g;

        for ( int j = 0; j < i; ++j)
        {
          e[ j ] = 0.0;
        }

        // Apply similarity transformation to remaining columns.
        for ( int j = 0; j < i; ++j )
        {
          f = d[ j ];
          V.setComponent ( j, i, f );
          g = e[ j ] + V( j, j ) * f;

          for ( int k = j + 1; k <= i - 1; ++k )
          {
            g += V ( k, j ) * d[ k ];
            e[ k ] += V ( k, j ) * f;
          }

          e[ j ] = g;
        }

        f = 0.0;
        for ( int j = 0; j < i; ++j )
        {
          e[ j ] /= h;
          f += e[ j ] * d[ j ];
        }

        double hh = f / ( h + h );

        for ( int j = 0; j < i; ++j )
        {
          e[ j ] -= hh * d[ j ];
        }

        for ( int j = 0; j < i; ++j )
        {
          f = d[ j ];
          g = e[ j ];

          for ( int k = j; k <= i - 1; ++k )
          {
            V.setComponent ( k, j, V ( k, j ) - ( f * e[ k ] + g * d[ k ] ));
          }

          d[ j ] = V( i - 1, j );
          V.setComponent ( i, j, 0.0 );
        }
      }
      d[ i ] = h;
    }

    // Accumulate transformations.
    for ( int i = 0; i < dimensionMinusOne; ++i )
    {
      V.setComponent ( dimensionMinusOne, i, V ( i, i ));
      V.setComponent ( i, i, 1.0 );
      double h = d[ i + 1 ];

      if ( h != 0.0 )
      {
        for ( int k = 0; k <= i; ++k )
        {
          d[ k ] = V ( k, i + 1 ) / h;
        }

        for ( int j = 0; j <= i; ++j )
        {
          double g = 0.0;

          for ( int k = 0; k <= i; ++k )
          {
            g += V ( k, i + 1 ) * V( k, j );
          }

          for ( int k = 0; k <= i; ++k )
          {
            V.setComponent ( k, j, V ( k, j ) - ( g * d[ k ] ));
          }
        }
      }
      for ( int k = 0; k <= i; ++k )
      {
        V.setComponent ( k, i + 1, 0.0);
      }
    }

    for ( int j = 0; j < dimension; ++j )
    {
      d[ j ] = V ( dimensionMinusOne, j );
      V.setComponent ( dimensionMinusOne, j, 0.0 );
    }

    V.setComponent ( dimensionMinusOne, dimensionMinusOne, 1.0 );
    e[ 0 ] = 0.0;
  }

  /**
      * Symmetric tridiagonal QL algorithm.
      *
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      */
  static void tql2 ( Matrix33 & V, Vector3 & d, Vector3 e )
  {
    unsigned short int dimension = 3;
    unsigned short int dimensionMinusOne = 2;

    for ( int i = 1; i < dimension; ++i )
    {
      e[ i - 1 ] = e[ i ];
    }

    e[ dimensionMinusOne ] = 0.0;

    double f = 0.0;
    double tst1 = 0.0;
    double eps = pow ( 2.0, -52.0 );
    for  (int l = 0; l < dimension; ++l )
    {
      // Find small subdiagonal element
      tst1 = max ( tst1, fabs ( d[ l ] ) + fabs( e[ l ] ));
      int m = l;
      while ( m < dimension )
      {
        if ( fabs ( e[ m ] ) <= eps * tst1 )
        {
          break;
        }
        ++m;
      }

      // If m == l, d[l] is an eigenvalue,
      // otherwise, iterate.
      if ( m > l )
      {
        int iter = 0;
        do
        {
          ++iter;  // (Could check iteration count here.)

          // Compute implicit shift

          double g = d[ l ];
          double p = ( d[ l + 1 ] - g ) / ( 2.0 * e[ l ] );
          double r = sqrt ( p * p + 1.0 * 1.0 );

          if ( p < 0 )
          {
            r = -r;
          }

          d[ l ] = e[ l ] / ( p + r );
          d[ l + 1 ] = e[ l ] * ( p + r );
          double dl1 = d[ l + 1 ];
          double h = g - d[ l ];
          for ( int i = l + 2; i < dimension; ++i )
          {
            d[ i ] -= h;
          }
          f = f + h;

          // Implicit QL transformation.
          p = d[ m ];
          double c = 1.0;
          double c2 = c;
          double c3 = c;
          double el1 = e[ l + 1 ];
          double s = 0.0;
          double s2 = 0.0;
          for ( int i = m - 1; i >= l; --i )
          {
            c3 = c2;
            c2 = c;
            s2 = s;
            g = c * e[ i ];
            h = c * p;
            r = sqrt ( p * p + e[ i ] * e[ i ] );
            e[ i + 1 ] = s * r;
            s = e[ i ] / r;
            c = p / r;
            p = c * d[ i ] - s * g;
            d[ i + 1 ] = h + s * ( c * g + s * d[ i ] );

            // Accumulate transformation.
            for ( int k = 0; k < dimension; ++k )
            {
              h = V ( k, i + 1 );
              V.setComponent ( k, i + 1, ( s * V ( k, i ) + c * h ));
              V.setComponent ( k, i, ( c * V ( k, i ) - s * h ));
            }
          }

          p = - s * s2 * c3 * el1 * e[ l ] / dl1;
          e[ l ] = s * p;
          d[ l ] = c * p;

          // Check for convergence.

        }
        while ( fabs ( e[ l ] ) > eps * tst1 );
      }

      d[ l ] = d[ l ] + f;
      e[ l ] = 0.0;
    }

    // Sort eigenvalues and corresponding vectors.
    for ( int i = 0; i < dimensionMinusOne; ++i )
    {
      int k = i;
      double p = d[ i ];

      for ( int j = i + 1; j < dimension; ++j )
      {
        if ( d[ j ] < p )
        {
          k = j;
          p = d[ j ];
        }
      }

      if ( k != i )
      {
        d[ k ] = d[ i ];
        d[ i ] = p;

        for ( int j = 0; j < dimension; ++j )
        {
          p = V ( j, i );
          V.setComponent ( j, i, V ( j, k ));
          V.setComponent ( j, k, p );
        }
      }
    }
  }

  /**
      * Compute both eigen vectors and eigen values from an input matrix.
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      *
      * @param matrix input 3D matrix where eigen values/vectors will be computed (size = 3x3).
      * @param eigenVectors output matrix of eigenvectors (size = 3x3).
      * @param eigenValues output vector of eigenvalues (size = 3).
      */
  static void getEigenDecomposition ( Matrix33 & matrix, Matrix33 & eigenVectors, Vector3 & eigenValues)
  {
    Vector3 e;

    for ( int i = 0; i < 3; ++i )
    {
      for ( int j = 0; j < 3; ++j )
      {
        eigenVectors.setComponent( i, j, matrix( i, j ));
      }

      e[ i ] = 0.0;
      eigenValues[ i ] = 0.0;
    }

    tred2 ( eigenVectors, eigenValues, e );

    tql2 ( eigenVectors, eigenValues, e );
  }
};
}


#endif // !defined EigenValues3D_h

#undef EigenValues3D_RECURSES
#endif // else defined(EigenValues3D_RECURSES)
