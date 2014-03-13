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
 * @brief Computes EigenValues and EigenVectors from 3D Matrix.
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
#include "DGtal/math/linalg/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/**
 * Description of struct 'EigenValues3D' <p>
 * \brief Aim: Computes EigenValues and EigenVectors from 3D Matrix.
 *
 * @tparam TQuantity Type of the quantity inside the matrix.
 */
template< typename TQuantity >
class EigenValues3D
{
public:
  typedef TQuantity Quantity;
  typedef SimpleMatrix< Quantity, 3, 3 > Matrix33;
  typedef typename Matrix33::RowVector Vector3;

private:
  /**
      * \brief Reduces a real symmetric matrix to a symmetric tridiagonal matrix using and accumulating
      * orthogonal similarity transformations
      *
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      *
      * @param[out] d contains the diagonal elements of the tridiagonal matrix.
      * @param[out] e contains the subdiagonal elements of the tridiagonal matrix in its last n-1 positions. e[0] is set to 0.
      * @param[in,out] V input symmetric matrix / output the orthogonal transformation matrix produced in the reduction.
      */
  static void tred2 ( Matrix33 & V, Vector3 & d, Vector3 & e )
  {
    Dimension dimension = 3;
    Dimension dimensionMinusOne = dimension - 1;

    for( Dimension j = 0; j < dimension; ++j )
    {
      d[ j ] = V ( dimensionMinusOne, j );
    }

    // Householder reduction to tridiagonal form.

    for( Dimension i = dimensionMinusOne; i > 0 && i <= dimensionMinusOne; --i )
    {
      // Scale to avoid under/overflow.

      Quantity scale = Quantity( 0.0 );
      Quantity h = Quantity( 0.0 );
      for( Dimension k = 0; k < i; ++k )
      {
        scale += Quantity( std::fabs( d[ k ] ));
      }

      if( scale == Quantity( 0.0 ))
      {
        e[ i ] = d[ i - 1 ];

        for( Dimension j = 0; j < i; ++j )
        {
          d[ j ] = V(( i - 1 ),  j );
          V.setComponent ( i, j, Quantity( 0.0 ));
          V.setComponent ( j, i, Quantity( 0.0 ));
        }
      }
      else
      {
        // Generate Householder vector.
        for ( Dimension k = 0; k < i; ++k )
        {
          d[ k ] /= scale;
          h += d[ k ] * d[ k ];
        }

        Quantity f = d[ i - 1 ];
        Quantity g = Quantity( std::sqrt( h ));

        if ( f > Quantity( 0.0 ))
        {
          g = -g;
        }

        e[ i ] = scale * g;
        h -= f * g;
        d[ i - 1 ] = f - g;

        for ( Dimension j = 0; j < i; ++j)
        {
          e[ j ] = Quantity( 0.0 );
        }

        // Apply similarity transformation to remaining columns.
        for ( Dimension j = 0; j < i; ++j )
        {
          f = d[ j ];
          V.setComponent ( j, i, f );
          g = e[ j ] + V( j, j ) * f;

          for ( Dimension k = j + 1; k <= i - 1; ++k )
          {
            g += V ( k, j ) * d[ k ];
            e[ k ] += V ( k, j ) * f;
          }

          e[ j ] = g;
        }

        f = Quantity( 0.0 );
        for ( Dimension j = 0; j < i; ++j )
        {
          e[ j ] /= h;
          f += e[ j ] * d[ j ];
        }

        double hh = f / ( h + h );

        for ( Dimension j = 0; j < i; ++j )
        {
          e[ j ] -= hh * d[ j ];
        }

        for ( Dimension j = 0; j < i; ++j )
        {
          f = d[ j ];
          g = e[ j ];

          for ( Dimension k = j; k <= i - 1; ++k )
          {
            V.setComponent ( k, j, V ( k, j ) - ( f * e[ k ] + g * d[ k ] ));
          }

          d[ j ] = V( i - 1, j );
          V.setComponent ( i, j, Quantity( 0.0 ));
        }
      }
      d[ i ] = h;
    }

    // Accumulate transformations.
    for ( Dimension i = 0; i < dimensionMinusOne; ++i )
    {
      V.setComponent ( dimensionMinusOne, i, V ( i, i ));
      V.setComponent ( i, i, Quantity( 1.0 ));
      Quantity h = d[ i + 1 ];

      if ( h != Quantity( 0.0 ) )
      {
        for ( Dimension k = 0; k <= i; ++k )
        {
          d[ k ] = V ( k, i + 1 ) / h;
        }

        for ( Dimension j = 0; j <= i; ++j )
        {
          Quantity g = Quantity( 0.0 );

          for ( Dimension k = 0; k <= i; ++k )
          {
            g += V ( k, i + 1 ) * V( k, j );
          }

          for ( Dimension k = 0; k <= i; ++k )
          {
            V.setComponent ( k, j, V ( k, j ) - ( g * d[ k ] ));
          }
        }
      }
      for ( Dimension k = 0; k <= i; ++k )
      {
        V.setComponent ( k, i + 1, Quantity( 0.0 ));
      }
    }

    for ( Dimension j = 0; j < dimension; ++j )
    {
      d[ j ] = V ( dimensionMinusOne, j );
      V.setComponent ( dimensionMinusOne, j, Quantity( 0.0 ));
    }

    V.setComponent ( dimensionMinusOne, dimensionMinusOne, Quantity( 1.0 ));
    e[ 0 ] = Quantity( 0.0 );
  }

  /**
      * \brief finds the eigenvalues and eigenvectors of a symmetric tridiagonal matrix by the QL method.
      * The eigenvectrs of a full symmetric matric can also be found if tred2() has been used to reduce this
      * full matrix to tridiagonal form.
      *
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      *
      * @param[out] e contains the subdiagonal elements of the input matrix in its last n-1 positions. e(0) is arbitrary.
      * @param[in,out] d input the diagonal elements of the input matrix / output orthonormal eigenvalues in ascending order.
      * @param[in,out] V input transformation matrix produced in the reduction by tred2(), if performed. If the eigenvectors of the tridiagonal matrix
      * are desired, V must contain the identity matrix / output orthonormal eigenvectors of the symmetric tridiagonal (or full) matrix.
      */
  static void tql2 ( Matrix33 & V, Vector3 & d, Vector3 e )
  {
    Dimension dimension = 3;
    Dimension dimensionMinusOne = dimension - 1;

    for ( Dimension i = 1; i < dimension; ++i )
    {
      e[ i - 1 ] = e[ i ];
    }

    e[ dimensionMinusOne ] = 0.0;

    Quantity f = Quantity( 0.0 );
    Quantity tst1 = Quantity( 0.0 );
    Quantity eps = Quantity( std::pow( 2.0, -52.0 ));
    for( Dimension l = 0; l < dimension; ++l )
    {
      // Find small subdiagonal element
      tst1 = Quantity( std::max( tst1, std::fabs ( d[ l ] ) + std::fabs( e[ l ] )));
      Dimension m = l;
      while ( m < dimension )
      {
        if ( std::fabs ( e[ m ] ) <= eps * tst1 )
        {
          break;
        }
        ++m;
      }

      // If m == l, d[l] is an eigenvalue,
      // otherwise, iterate.
      if( m > l )
      {
        int iter = 0;
        do
        {
          ++iter;  // (Could check iteration count here.)

          // Compute implicit shift

          Quantity g = d[ l ];
          Quantity p = ( d[ l + 1 ] - g ) / ( Quantity( 2.0 ) * e[ l ] );
          Quantity r = Quantity( std::sqrt ( p * p + Quantity( 1.0 ) * Quantity( 1.0 )));

          if( p < 0 )
          {
            r = -r;
          }

          d[ l ] = e[ l ] / ( p + r );
          d[ l + 1 ] = e[ l ] * ( p + r );
          Quantity dl1 = d[ l + 1 ];
          Quantity h = g - d[ l ];
          for( Dimension i = l + 2; i < dimension; ++i )
          {
            d[ i ] -= h;
          }
          f = f + h;

          // Implicit QL transformation.
          p = d[ m ];
          Quantity c = Quantity( 1.0 );
          Quantity c2 = c;
          Quantity c3 = c;
          Quantity el1 = e[ l + 1 ];
          Quantity s = Quantity( 0.0 );
          Quantity s2 = Quantity( 0.0 );
          for ( Dimension i = m - 1; i >= l && i <= m - 1; --i )
          {
            c3 = c2;
            c2 = c;
            s2 = s;
            g = c * e[ i ];
            h = c * p;
            r = Quantity( std::sqrt ( p * p + e[ i ] * e[ i ] ));
            e[ i + 1 ] = s * r;
            s = e[ i ] / r;
            c = p / r;
            p = c * d[ i ] - s * g;
            d[ i + 1 ] = h + s * ( c * g + s * d[ i ] );

            // Accumulate transformation.
            for( Dimension k = 0; k < dimension; ++k )
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
        while ( std::fabs ( e[ l ] ) > eps * tst1 );
      }

      d[ l ] = d[ l ] + f;
      e[ l ] = Quantity( 0.0 );
    }

    // Sort eigenvalues and corresponding vectors.
    for ( Dimension i = 0; i < dimensionMinusOne; ++i )
    {
      Dimension k = i;
      Quantity p = d[ i ];

      for ( Dimension j = i + 1; j < dimension; ++j )
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

        for ( Dimension j = 0; j < dimension; ++j )
        {
          p = V ( j, i );
          V.setComponent ( j, i, V ( j, k ));
          V.setComponent ( j, k, p );
        }
      }
    }
  }

  /**
      * \brief Compute both eigen vectors and eigen values from an input matrix.
      *
      * This is derived from the Algol procedures tred2 and tql2 by Bowdler, Martin, Reinsch
      * and Wilkinson, Handbook for Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
      * Fortran subroutine in EISPACK.
      *
      * @param[out] eigenVectors  matrix of eigenvectors (size = dimension * dimension). Eigenvectors are put in column.
      * @param[out] eigenValues   vector of eigenvalues (size = dimension).
      * @param[in]  matrix        3D matrix where eigen values/vectors will be computed (size = dimension * dimension).
      */

public:
  static void getEigenDecomposition( const Matrix33 & matrix, Matrix33 & eigenVectors, Vector3 & eigenValues)
  {
    Vector3 e;
    Dimension dimension = 3;

    for ( Dimension i = 0; i < dimension; ++i )
    {
      for ( Dimension j = 0; j < dimension; ++j )
      {
        eigenVectors.setComponent( i, j, matrix( i, j ));
      }

      e[ i ] = Quantity( 0.0 );
      eigenValues[ i ] = Quantity( 0.0 );
    }

    tred2 ( eigenVectors, eigenValues, e );

    tql2 ( eigenVectors, eigenValues, e );
  }
};
}


#endif // !defined EigenValues3D_h

#undef EigenValues3D_RECURSES
#endif // else defined(EigenValues3D_RECURSES)
