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
 * @file IntegerMatrixFunctions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/01
 *
 * Header file gathering functions specific to integer matrix
 * computations, like specific determinant or basis reduction via LLL.
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegerMatrixFunctions_RECURSES)
#error Recursive header files inclusion detected in IntegerMatrixFunctions.h
#else // defined(IntegerMatrixFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegerMatrixFunctions_RECURSES

#if !defined IntegerMatrixFunctions_h
/** Prevents repeated inclusion of headers. */
#define IntegerMatrixFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/kernel/ArithmeticConversionTraits.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/math/linalg/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace functions {

    /// Negates the components of the input vector.
    ///
    /// @tparam TComponent the scalar type of each component.
    /// @param[in,out] V the vector as input, outputed as -V.
    template <typename TComponent>
    void
    negate( std::vector<TComponent> &V );

    /// Returns 'true' iff all the components of the two  vectors are equal.
    ///
    /// @tparam TComponent the scalar type of each component.
    ///
    /// @param[in] a the left vector (same size)
    /// @param[in] b the right vector (same size)
    ///
    /// @return 'true' iff all the components of the two  vectors are equal.
    template <typename TComponent>
    bool
    equals( const std::vector<TComponent> &a, const std::vector<TComponent> &b );

    /// Overloaded dot product operator for vector of numbers.
    ///
    /// @tparam T the number type of the left operand vector.
    /// @tparam U the number type of the right operand vector.
    ///
    /// @param[in] a the left vector
    /// @param[in] b the right vector
    ///
    /// @return the dot product of a and b, a scalar value in the
    /// "best possible type" according to T and U.
    template <typename T, typename U >
    typename DGtal::ArithmeticConversionTraits<T,U>::type
    dotProduct( const std::vector<T>& a, const std::vector<U>& b );

    /// Overloaded dot product operator for vector of big integers
    /// with vector of doubles.
    ///
    /// @param[in] a the left vector
    /// @param[in] b the right vector
    ///
    /// @return the dot product of a and b.
    double
    dotProduct( const std::vector<BigInteger>& a, const std::vector<double>& b );

    /// Overloaded dot product operator for vector of big integers
    /// with vector of doubles.
    ///
    /// @param[in] a the left vector
    /// @param[in] b the right vector
    ///
    /// @return the dot product of a and b.
    double
    dotProduct( const std::vector<double>& a, const std::vector<BigInteger>& b );

    /// Overloaded cross product operator for vector of numbers.
    ///
    /// @tparam T the number type of the left operand vector.
    /// @tparam U the number type of the right operand vector.
    ///
    /// @param[in] a the left vector (must be of size 3 )
    /// @param[in] b the right vector (must be of size 3 )
    ///
    /// @return the cross product of a and b (a vector of size 3 ).
    template <typename T, typename U >
    std::vector< typename DGtal::ArithmeticConversionTraits<T,U>::type >
    crossProduct( const std::vector<T>& a, const std::vector<U>& b );

    /// Applies the binary operation \a op2 to every pair of elements
    /// (a[i],b[i]) and returns the resulting range of values
    /// `op2([a[0],b[0]), ..., op2([a[n-1],b[n-1])`.
    ///
    /// @tparam T the number type of the left operand vector.
    /// @tparam U the number type of the right operand vector.
    /// @tparam Op2 the type for the binary operator.
    ///
    /// @param a the range of values acting as left operands.
    /// @param b the range of values acting as rights operands.
    ///
    /// @param op2 the binary operator (T,U) -> V, where V is the best
    /// possible type combining T and U.
    template <typename T, typename U, typename Op2 >
    std::vector< typename DGtal::ArithmeticConversionTraits<T,U>::type >
    apply( const std::vector<T>& a, const std::vector<U>& b,
           Op2 op2 );
    
    /// Overloaded squared L2-norm operator for a vector of numbers.
    ///
    /// @tparam T the number type of the input vector.
    ///
    /// @param[in] a the vector of numbers
    ///
    /// @return the dot product a.a, i.e. the squared L2-norm of a.
    ///
    /// @see getSquaredNormL2 if you wish to use another type for computation.
    template <typename T>
    T
    squaredNormL2( const std::vector<T>& a );
    
    /// Overloaded L1-norm operator for a vector of numbers.
    ///
    /// @tparam T the number type of the input vector.
    ///
    /// @param[in] a the vector of numbers
    ///
    /// @return the L1-norm of a, i.e. \f$ \sum_i |a_i| \f$, sometimes called block distance.
    template <typename T>
    T
    normL1( const std::vector<T>& a );

    /// Overloaded Loo-norm operator for a vector of numbers.
    ///
    /// @tparam T the number type of the input vector.
    ///
    /// @param[in] a the vector of numbers
    ///
    /// @return the \f$ L_\infinity \f$-norm of a, i.e. \f$ \max_i |a_i| \f$.
    template <typename T>
    T
    normLoo( const std::vector<T>& a );
    
    /// Overloaded squared L2-norm operator for a vector of numbers.
    ///
    /// @tparam TOutput the desired number type for output.
    /// @tparam T the number type of the input vector.
    ///
    /// @param[out] n the dot product a.a, i.e. the squared L2-norm of a.
    ///
    /// @param[in] a the vector of numbers.
    template <typename TOutput, typename T>
    void
    getSquaredNormL2( TOutput& n, const std::vector<T>& a );

    /// Overloaded squared L2-norm operator for a vector of numbers.
    ///
    /// @tparam TOutput the desired number type for output.
    ///
    /// @tparam dim static constant of type DGtal::Dimension that
    /// specifies the static  dimension of the space and thus the number
    /// of elements  of the Point or Vector.
    ///
    /// @tparam TEuclideanRing speficies the number type assoicated to an
    /// Euclidean domain (or Euclidean ring) algebraic structure
    /// (commutative unitary ring with no zero divisors and with a division
    /// operator but not necessarily an inverse for the multiplication
    /// operator). This type is used to represent PointVector elements
    /// (Coordinate for Point and Component for Vector) and define
    /// operations on Point or Vectors.
    ///
    /// @tparam TContainer specifies the container to be used to store
    /// the point coordinates. At this point, such container must be a
    /// random access bidirectionnal a-la STL containers (e.g. vector,
    /// boost/array). If TContainer implements comparison operators == !=
    /// < <= > <=, then PointVector will also implements it and with the
    /// exact same behaviour.
    ///
    /// @param[out] n the dot product a.a, i.e. the squared L2-norm of a.
    ///
    /// @param[in] a the vector of numbers.
    template <typename TOutput,
              DGtal::Dimension dim,
              typename TEuclideanRing,
              typename TContainer>
    void
    getSquaredNormL2( TOutput& n,
                      const DGtal::PointVector<dim,TEuclideanRing,TContainer>& a );
    
    /// Computes the determinant of a squared matrix with integer or
    /// floating-point coefficients using Bareiss method.  Complexity
    /// is in O(n^3) if you assume O(1) for each arithmetic operation.
    ///
    /// @tparam TComponent the number type.
    /// @tparam TN the order of the squared matrix.
    /// @tparam TInternalNumber the number type used for internal
    /// computations and for the output.
    ///
    /// @param[out] result the determinant of this matrix.
    ///
    /// @param[in] matrix a squared matrix.
    ///
    /// @note In case of integer coefficients, intermediate integer
    /// values may grow quickly. Use int64_t or even
    /// boost::multiprecision::cpp_int to get robust result.
    template <typename TComponent, DGtal::Dimension TN,
              typename TInternalNumber>
    void
    getDeterminantBareiss( TInternalNumber& result,
                           const SimpleMatrix<TComponent, TN, TN>& matrix );

    /// Computes the determinant of a squared matrix with integer or
    /// floating-point coefficients using Bareiss method.  Complexity
    /// is in O(n^3) if you assume O(1) for each arithmetic operation.
    ///
    /// @tparam TComponent the number type.
    /// @tparam TInternalNumber the number type used for internal
    /// computations and for the output.
    ///
    /// @param[in] matrix a squared matrix, represented as a vector of row vectors.
    ///
    /// @param[out] result the determinant of this matrix.
    ///
    /// @note In case of integer coefficients, intermediate integer
    /// values may grow quickly. Use int64_t or even
    /// boost::multiprecision::cpp_int to get robust result.
    template <typename TComponent, typename TInternalNumber>
    void
    getDeterminantBareiss( TInternalNumber& result,
                           const std::vector< std::vector< TComponent > >& matrix );

    /// Outputs the matrix M of size m x n with coefficients \a c, as
    /// a vector of vector representation.
    ///
    /// @tparam TComponent the number type.
    ///
    /// @param[in] m, n the number of rows and columns of the matrix M.
    /// @param[in] c the coefficients of the matrix in order c_00, c01, etc.
    ///
    /// @return the m x n matrix with coefficients c filled row
    /// per row, and potentially completed with zero.
    template <typename TComponent>
    std::vector< std::vector< TComponent > >
    matrixAsVectorVector( std::size_t m, std::size_t n,
                          const std::vector< TComponent >& c );

    /// Outputs the matrix of size m x n with coefficients \a c, as
    /// a vector of vector representation.
    ///
    /// @tparam TComponent the number type.
    /// @tparam TN the number of rows
    /// @tparam TM the number of columns
    ///
    /// @param[in] M any matrix
    ///
    /// @return the m x n matrix cloning the coefficients of \a M.
    template <typename TComponent, DGtal::Dimension TM, DGtal::Dimension TN>
    std::vector< std::vector< TComponent > >
    matrixAsVectorVector( const SimpleMatrix<TComponent, TM, TN>& M );

    /// Computes the \f$ \delta \f$-LLL-reduced lattice of the input lattice B
    /// (represented as an integer matrix B whose rows are the lattice
    /// vectors), using the LLL algorithm.
    ///
    /// Such lattice has "almost" orthogonal vectors, while minimizing
    /// their norm at most as possible.
    ///
    /// @note A lattice \f$(b_1,...,b_m)\f$ is \f$ \delta
    /// \f$-LLL-reduced if (1), for any \f$ i>j, |\mu_{i,j}|
    /// \le 1/2 \f$, and (2), for any \f$ i<m , \delta |b^*_i|^2 \le
    /// |b^*_{i+1}+mu_{i+1,i} b^*_i|^2 \f$. Here \f$ \mu_{i,j} :=
    /// \langle b_i, b_j^* \rangle / \langle b_j^*, b_j^* \rangle \f$
    /// and \f$ b^*_i \f$ is the \a i-th vector of the Gram-Schmidt
    /// orthogonalisation of \f$ (b_1, ..., b_m) \f$.
    ///
    /// @tparam TComponent the integer type of each coefficient of the integer matrix \a B.
    ///
    /// @tparam TDouble the floating-point number type used for the
    /// Gram-Schmidt orthogonalisation of \a B.
    ///
    /// @param[in] B the lattice of m vectors in Z^n represented as a mxn matrix.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    ///
    /// @return the \f$ delta \f$-LLL-reduced lattice of \a B.
    ///
    /// @warning Computations requiring Gram-Schmidt orthogonalisation
    /// uses the floating-point number type `TDouble`.
    template <typename TComponent, typename TDouble = long double >
    std::vector< std::vector< TComponent > >
    computeLLLBasis( const std::vector< std::vector< TComponent > >& B,
                     TDouble delta = 0.75 );
    
    /// Computes the \f$ \delta \f$-LLL-reduced lattice of the input lattice B
    /// (represented as an integer matrix B whose rows are the lattice
    /// vectors), using the LLL algorithm.
    ///
    /// Such lattice has "almost" orthogonal vectors, while minimizing
    /// their norm at most as possible.
    ///
    /// @note A lattice \f$(b_1,...,b_m)\f$ is \f$ \delta
    /// \f$-LLL-reduced if (1), for any \f$ i>j, |\mu_{i,j}|
    /// \le 1/2 \f$, and (2), for any \f$ i<m , \delta |b^*_i|^2 \le
    /// |b^*_{i+1}+mu_{i+1,i} b^*_i|^2 \f$. Here \f$ \mu_{i,j} :=
    /// \langle b_i, b_j^* \rangle / \langle b_j^*, b_j^* \rangle \f$
    /// and \f$ b^*_i \f$ is the \a i-th vector of the Gram-Schmidt
    /// orthogonalisation of \f$ (b_1, ..., b_m) \f$.
    ///
    /// @tparam TComponent the integer type of each coefficient of the integer matrix \a B.
    ///
    /// @tparam TDouble the floating-point number type used for the
    /// Gram-Schmidt orthogonalisation of \a B.
    ///
    /// @param[in,out] B as input, the lattice of m vectors in Z^n
    /// represented as a mxn matrix, as output the \f$ delta
    /// \f$-LLL-reduced lattice of \a B.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    ///
    /// @warning Computations requiring Gram-Schmidt orthogonalisation
    /// uses the floating-point number type `TDouble`.
    template <typename TComponent, typename TDouble = long double >
    void
    reduceBasisWithLLL( std::vector< std::vector< TComponent > >& B,
                        TDouble delta = 0.75 );

    /// Makes a lattice vector primitive.
    ///
    /// @tparam TComponent the integer type for the input vector and for computations.
    ///
    /// @param[in,out] N the input vector, that is modified to be N/g,
    /// where g is the gcd of its components.
    ///
    /// @return the gcd of the components.
    template <typename TComponent>
    TComponent
    makePrimitive( std::vector< TComponent >& N );

    /// Extended Eucliden algorithm to compute the gcd g of two
    /// integers a and b as well as two other integers x and y such
    /// that `a*x+b*y=g`, where g the gcd of a and b.
    ///
    /// @param[out] x,y two integers x and y such that `a*x+b*y=g`.
    ///
    /// @param[in] a,b any pair of integers.
    ///
    /// @return the greatest common divisor g of a and b, i.e. gcd( a, b ).
    template <typename TComponent>
    TComponent
    extendedGcd( TComponent& x, TComponent& y, TComponent a,TComponent b );

    /// Extended Eucliden algorithm to compute the gcd g of n integers
    /// \a A as will as the vector of integers C such that
    /// `C[0]*A[0]+...+C[n-1]*A[n-1]=g`, where g is the gcd of
    /// `A[0],...,A[n-1]`.
    ///
    /// @param[out] C the array of integers such that `C[0]*A[0]+...+C[n-1]*A[n-1]=g`
    ///
    /// @param[in] A the array of input integers.
    ///
    /// @return the greatest common divisor g of \a A, i.e. gcd( A[0],
    /// gcd( A[1], ... )).
    template <typename TComponent>
    TComponent
    extendedGcd( std::vector<TComponent> &C, const std::vector<TComponent> &A );
    
    
    /// Computes a basis of the orthogonal lattice to the vector \a N,
    /// i.e. it has n-1 vectors if N had n components. It essentially
    /// follows
    /// https://math.stackexchange.com/questions/1049608/lattice-generated-by-vectors-orthogonal-to-an-integer-vector
    ///
    /// @note If N is a primitive 3D vector, then the returned basis has two
    /// vectors, whose cross product is N or -N.
    ///
    /// @tparam TComponent the integer type for the input vector and for computations.
    ///
    /// @param[in] N the input normal vector, which should be non
    /// null. Even if \a N is not primitive, it is considered
    /// primitive in the function.
    ///
    /// @return a basis \f$ (u_1, \dots, u_{n-1}) \f$ such that \f$
    /// u_i \cdot N = 0 \f$. Because \a N is made primitive, the
    /// orthogonal lattice is the finest possible for the d-1
    /// dimensional space. The returned basis is in \b echelon \b
    /// form.
    template <typename TComponent>
    std::vector< std::vector< TComponent > >
    computeOrthogonalLattice( std::vector< TComponent > N );

    /// Tries to shorten u and v while preserving their vector span by
    /// trying u+v and u-v.
    ///
    /// @tparam TComponent the integer type for the input vector and for computations.
    /// @param[in,out] u any vector
    /// @param[in,out] v any vector
    ///
    /// @return 'true' if one shortening was possible, then at least
    /// one of u and v is modified.
    template <typename TComponent>
    bool
    shortenVectors( std::vector< TComponent >& u,
                    std::vector< TComponent >& v );

    /// Shortens the vectors of the basis B (by pairwise shortenings)
    /// until no further shortening pairs are found, and returns the
    /// number of shortening operations.
    ///
    /// @tparam TComponent the integer type for the input vector and for computations.
    ///
    /// @param[in,out] B a range of vectors forming a basis, which is
    /// shorten as much as possible in terms of L2-norm. Note that the
    /// output basis is \b not in \b echelon \b form in general.
    ///
    /// @return the number of pairwise shortenings done.
    template <typename TComponent>
    std::size_t
    shortenBasis( std::vector< std::vector< TComponent > >& B );
    
  } // namespace functions

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/linalg/IntegerMatrixFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegerMatrixFunctions_h

#undef IntegerMatrixFunctions_RECURSES
#endif // else defined(IntegerMatrixFunctions_RECURSES)
