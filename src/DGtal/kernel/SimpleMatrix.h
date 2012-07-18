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
 * @file SimpleMatrix.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/07/10
 *
 * Header file for module SimpleMatrix.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleMatrix_RECURSES)
#error Recursive header files inclusion detected in SimpleMatrix.h
#else // defined(SimpleMatrix_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleMatrix_RECURSES

#if !defined SimpleMatrix_h
/** Prevents repeated inclusion of headers. */
#define SimpleMatrix_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CEuclideanRing.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SimpleMatrix
  /**
   * Description of template class 'SimpleMatrix' <p>
   * \brief Aim: implements basic MxN Matrix services (M,N>=1).
   *
   * This class defines simple matrix with static size. Computations
   * are performed in the euclidean ring associated with the type @a TComponent.
   *
   * This class also defines types for row and column vectors as
   * specialized PointVector types.
   *
   * Class inspired by Anis Benyoub (INSA-Lyon).
   *
   * @tparam TComponent any model of CEuclideanRing
   * @tparam TM number of rows of the matrix
   * @tparam TN number of columns of the matrix
   */
  template <typename TComponent, DGtal::Dimension TM, DGtal::Dimension TN>
  class SimpleMatrix
  {

  public:
    typedef TComponent Component;
    static const DGtal::Dimension M = TM;
    static const DGtal::Dimension N = TN;
    
    typedef PointVector<N,Component> RowVector;
    typedef PointVector<M,Component> ColumnVector;

    typedef SimpleMatrix<Component,TM,TN> Self;

    BOOST_CONCEPT_ASSERT(( CEuclideanRing<TComponent> ));
    BOOST_STATIC_ASSERT(TM > 0 );
    BOOST_STATIC_ASSERT(TM > 0 );
    
    /** 
     * Create a static mxn matrix.
     *
     * SimpleMatrix values are all set to the zero value associated to
     * Component type.
     */
    SimpleMatrix();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    SimpleMatrix ( const Self & other );

    // ----------------------- Standard services ------------------------------

    /** 
     * Clear matrix values
     * 
     */
    void clear();


    /** 
     * Set a constant scalar to each matrix component.
     * 
     * @param aScalar the scalar
     */
    void constant(const Component &aScalar);

    /** 
     * Get row vector.
     * 
     * @param i the row index
     * @return the i-th row
     */
    RowVector row(const DGtal::Dimension i) const;

    /** 
     * Get column vector.
     * 
     * @param j the column index.
     * @return the j-th column
     */
    ColumnVector column(const DGtal::Dimension j) const;

    /** 
     * Set a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * @param aValue a component
     * 
     */
    void setComponent(const DGtal::Dimension i, const DGtal::Dimension j,
                      const Component & aValue);

    /** 
     * Get a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * 
     */
    Component operator()(const DGtal::Dimension i, const DGtal::Dimension j) const;

    // ----------------------- SimpleMatrix computations ------------------------------


    /** 
     * SimpleMatrix comparison.
     * 
     * @param another matrix.
     * @return true if aMatrix equals this
     */
    bool operator==(const Self & aMatrix) const;

    /** 
     * Assignment operator from another matrix.
     * Note: a static_cast from TComponentOther to Component is performed.
     * 
     * @tparam TComponentOther another Component type.
     * @param aMatrix the matrix to copy.
     * 
     * @return 
     */
    template<typename TComponentOther>
    Self & operator=(const SimpleMatrix<TComponentOther, M, N>& aMatrix);

    /** 
     * Addition between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return the summed matrix
     */
    Self  operator+(const Self & aMatrix) const;

    /** 
     * Addition and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return a reference to the result
     */
    Self & operator+=(const Self & aMatrix);


    /** 
     * Substract between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return the difference matrix
     */
    Self  operator-(const Self & aMatrix) const;

    /** 
     * Substract and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return a reference to the result
     */
    Self & operator-=(const Self & aMatrix);

    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting matrix
     */
    Self  operator*(const Component & aScalar) const;
 
    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting a reference to the matrix
     */
    Self & operator*=(const Component & aScalar);
 
    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return the resulting matrix
     */
    Self  operator/(const Component & aScalar) const;

    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return a reference to the updated matrix
     */
    Self & operator/=(const Component & aScalar) ;
    
    /** 
     * Product between the matrix 'this' and @a aMatrix.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    SimpleMatrix<Component,TM,TM>  operator*(const SimpleMatrix<Component,N,M> & aMatrix) const;
    
   
    /** 
     * Product between the matrix and a Column vector.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    ColumnVector  operator*(const RowVector & aVector) const;
    
   
    /** 
     * Transpose the  matrix.
     *
     * @return the transposted NxM matrix 
     */
    SimpleMatrix<Component,TN,TM> transpose() const;


    /** 
     * Cofactor of the matrix at position (i,j).
     * 
     * @return the cofactor at (i,j).
     */
    Component cofactor(const DGtal::Dimension i,
                       const DGtal::Dimension j) const;
   
    /** 
     * Cofactor matrix computation.
     * 
     * @return the cofactor matrix.
     */
    Self cofactor() const;
     
    /** 
     * Return the minor determinant (i,j) of the current matrix
     * 
     * @param i row index
     * @param j column index
     * 
     * @return the minor (i,j)
     */
    Component minorDeterminant(const DGtal::Dimension i, 
			       const DGtal::Dimension j) const;

    /** 
     * Returns the determinant of square matrix.
     * Slow method for large matrices.
     * @pre this must be NxN
     * 
     * @return the determinant.
     */
    Component determinant() const;

  
    /**
     * Destructor.
     */
    ~SimpleMatrix();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
 
    
    // ------------------------- Private Datas --------------------------------
  private:

    ///Matrix values containers.
    boost::array< Component, M*N>  myValues;

    ///Static computation of cofactor coefficients
    /// @todo should be static 
    boost::array< Component, M*N>  myCofactorCoefs;

    // ------------------------- Hidden services ------------------------------
  protected:

  }; // end of class SimpleMatrix


  /////////////////////////////////////////////////////////////////////////////
  // template class SimpleMatrix
  /**
   * Description of template class 'SimpleMatrix' <p>
   * \brief Aim: implements basic 2x2 Matrix services.
   *
   * This class defines simple matrix with static size. Computations
   * are performed in the euclidean ring associated with the type @a TComponent.
   *
   * This class also defines types for row and column vectors as
   * specialized PointVector types.
   *
   * @tparam TComponent any model of CEuclideanRing
   * @tparam TM number of rows of the matrix
   * @tparam TN number of columns of the matrix
   */
  template <typename TComponent>
  class SimpleMatrix<TComponent,2,2>
  {

  public:
    typedef TComponent Component;
    static const DGtal::Dimension M = 2;
    static const DGtal::Dimension N = 2;
    
    typedef PointVector<N,Component> RowVector;
    typedef PointVector<M,Component> ColumnVector;

    typedef SimpleMatrix<Component,2,2> Self;

    BOOST_CONCEPT_ASSERT(( CEuclideanRing<TComponent> ));
    
    /** 
     * Create a static mxn matrix.
     *
     * SimpleMatrix values are all set to the zero value associated to
     * Component type.
     */
    SimpleMatrix();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    SimpleMatrix ( const Self & other );

    // ----------------------- Standard services ------------------------------

    /** 
     * Clear matrix values
     * 
     */
    void clear();


    /** 
     * Set a constant scalar to each matrix component.
     * 
     * @param aScalar the scalar
     */
    void constant(const Component &aScalar);

    /** 
     * Get row vector.
     * 
     * @param i the row index
     * @return the i-th row
     */
    RowVector row(const DGtal::Dimension i) const;

    /** 
     * Get column vector.
     * 
     * @param j the column index.
     * @return the j-th column
     */
    ColumnVector column(const DGtal::Dimension j) const;

    /** 
     * Set a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * @param aValue a component
     * 
     */
    void setComponent(const DGtal::Dimension i, const DGtal::Dimension j,
                      const Component & aValue);

    /** 
     * Get a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * 
     */
    Component operator()(const DGtal::Dimension i, const DGtal::Dimension j) const;

    // ----------------------- SimpleMatrix computations ------------------------------


    /** 
     * SimpleMatrix comparison.
     * 
     * @param another matrix.
     * @return true if aMatrix equals this
     */
    bool operator==(const Self & aMatrix) const;

    /** 
     * Assignment operator from another matrix.
     * Note: a static_cast from TComponentOther to Component is performed.
     * 
     * @tparam TComponentOther another Component type.
     * @param aMatrix the matrix to copy.
     * 
     * @return 
     */
    template<typename TComponentOther>
    Self & operator=(const SimpleMatrix<TComponentOther, M, N>& aMatrix);

    /** 
     * Addition between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return the summed matrix
     */
    Self  operator+(const Self & aMatrix) const;

    /** 
     * Addition and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return a reference to the result
     */
    Self & operator+=(const Self & aMatrix);


    /** 
     * Substract between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return the difference matrix
     */
    Self  operator-(const Self & aMatrix) const;

    /** 
     * Substract and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return a reference to the result
     */
    Self & operator-=(const Self & aMatrix);

    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting matrix
     */
    Self  operator*(const Component & aScalar) const;
 
    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting a reference to the matrix
     */
    Self & operator*=(const Component & aScalar);
 
    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return the resulting matrix
     */
    Self  operator/(const Component & aScalar) const;

    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return a reference to the updated matrix
     */
    Self & operator/=(const Component & aScalar) ;
    
    /** 
     * Product between the matrix 'this' and @a aMatrix.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    SimpleMatrix<Component,2,2>  operator*(const SimpleMatrix<Component,N,M> & aMatrix) const;
    
   
    /** 
     * Product between the matrix and a Column vector.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    ColumnVector  operator*(const RowVector & aVector) const;
    
   
    /** 
     * Transpose the  matrix.
     *
     * @return the transposted NxM matrix 
     */
    SimpleMatrix<Component,2,2> transpose() const;


    /** 
     * Cofactor of the matrix at position (i,j).
     * 
     * @return the cofactor at (i,j).
     */
    Component cofactor(const DGtal::Dimension i,
                       const DGtal::Dimension j) const;
   
    /** 
     * Cofactor matrix computation.
     * 
     * @return the cofactor matrix.
     */
    Self cofactor() const;
     
    /** 
     * Return the minor determinant (i,j) of the current matrix
     * 
     * @param i row index
     * @param j column index
     * 
     * @return the minor (i,j)
     */
    Component minorDeterminant(const DGtal::Dimension i, 
			       const DGtal::Dimension j) const;

    /** 
     * Returns the determinant of square matrix.
     * Slow method for large matrices.
     * @pre this must be NxN
     * 
     * @return the determinant.
     */
    Component determinant() const;

 
    /**
     * Destructor.
     */
    ~SimpleMatrix();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
 
    
    // ------------------------- Private Datas --------------------------------
  private:

    ///Matrix values containers.
    boost::array< Component, M*N>  myValues;

    ///Static computation of cofactor coefficients
    /// @todo should be static 
    boost::array< Component, M*N>  myCofactorCoefs;

    // ------------------------- Hidden services ------------------------------
  protected:

  }; // end of class SimpleMatrix

  /**
   * Description of template class 'SimpleMatrix' <p>
   * \brief Aim: implements basic 3x3 Matrix services.
   *
   * This class defines simple matrix with static size. Computations
   * are performed in the euclidean ring associated with the type @a TComponent.
   *
   * This class also defines types for row and column vectors as
   * specialized PointVector types.
   *
   * @tparam TComponent any model of CEuclideanRing
   * @tparam TM number of rows of the matrix
   * @tparam TN number of columns of the matrix
   */
  template <typename TComponent>
  class SimpleMatrix<TComponent,3,3>
  {

  public:
    typedef TComponent Component;
    static const DGtal::Dimension M = 3;
    static const DGtal::Dimension N = 3;
    
    typedef PointVector<N,Component> RowVector;
    typedef PointVector<M,Component> ColumnVector;

    typedef SimpleMatrix<Component,3,3> Self;

    BOOST_CONCEPT_ASSERT(( CEuclideanRing<TComponent> ));
    
    /** 
     * Create a static mxn matrix.
     *
     * SimpleMatrix values are all set to the zero value associated to
     * Component type.
     */
    SimpleMatrix();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    SimpleMatrix ( const Self & other );

    // ----------------------- Standard services ------------------------------

    /** 
     * Clear matrix values
     * 
     */
    void clear();


    /** 
     * Set a constant scalar to each matrix component.
     * 
     * @param aScalar the scalar
     */
    void constant(const Component &aScalar);

    /** 
     * Get row vector.
     * 
     * @param i the row index
     * @return the i-th row
     */
    RowVector row(const DGtal::Dimension i) const;

    /** 
     * Get column vector.
     * 
     * @param j the column index.
     * @return the j-th column
     */
    ColumnVector column(const DGtal::Dimension j) const;

    /** 
     * Set a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * @param aValue a component
     * 
     */
    void setComponent(const DGtal::Dimension i, const DGtal::Dimension j,
                      const Component & aValue);

    /** 
     * Get a value at position (i,j).
     *
     * @param i row index
     * @param j column index
     * 
     */
    Component operator()(const DGtal::Dimension i, const DGtal::Dimension j) const;

    // ----------------------- SimpleMatrix computations ------------------------------


    /** 
     * SimpleMatrix comparison.
     * 
     * @param another matrix.
     * @return true if aMatrix equals this
     */
    bool operator==(const Self & aMatrix) const;

    /** 
     * Assignment operator from another matrix.
     * Note: a static_cast from TComponentOther to Component is performed.
     * 
     * @tparam TComponentOther another Component type.
     * @param aMatrix the matrix to copy.
     * 
     * @return 
     */
    template<typename TComponentOther>
    Self & operator=(const SimpleMatrix<TComponentOther, M, N>& aMatrix);

    /** 
     * Addition between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return the summed matrix
     */
    Self  operator+(const Self & aMatrix) const;

    /** 
     * Addition and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to add to self.
     * 
     * @return a reference to the result
     */
    Self & operator+=(const Self & aMatrix);


    /** 
     * Substract between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return the difference matrix
     */
    Self  operator-(const Self & aMatrix) const;

    /** 
     * Substract and assignment between the matrix 'this' and @a aMatrix.
     * 
     * @param aMatrix the matrix to substract to self.
     * 
     * @return a reference to the result
     */
    Self & operator-=(const Self & aMatrix);

    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting matrix
     */
    Self  operator*(const Component & aScalar) const;
 
    /** 
     * Product between the matrix 'this' and a scalar
     * 
     * @param aScalar the scalar coefficient
     * 
     * @return the resulting a reference to the matrix
     */
    Self & operator*=(const Component & aScalar);
 
    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return the resulting matrix
     */
    Self  operator/(const Component & aScalar) const;

    /** 
     * Division of a matrix by a scalar.
     * 
     * @param aScalar the scalar value
     * 
     * @return a reference to the updated matrix
     */
    Self & operator/=(const Component & aScalar) ;
    
    /** 
     * Product between the matrix 'this' and @a aMatrix.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    SimpleMatrix<Component,3,3>  operator*(const SimpleMatrix<Component,N,M> & aMatrix) const;
    
   
    /** 
     * Product between the matrix and a Column vector.
     * @note the product is O(N^3) for NxN matrices.
     *
     * @param aMatrix the NxM matrix to multiply
     * 
     * @return the product MxM matrix 
     */
    ColumnVector  operator*(const RowVector & aVector) const;
    
   
    /** 
     * Transpose the  matrix.
     *
     * @return the transposted NxM matrix 
     */
    SimpleMatrix<Component,3,3> transpose() const;


    /** 
     * Cofactor of the matrix at position (i,j).
     * 
     * @return the cofactor at (i,j).
     */
    Component cofactor(const DGtal::Dimension i,
                       const DGtal::Dimension j) const;
   
    /** 
     * Cofactor matrix computation.
     * 
     * @return the cofactor matrix.
     */
    Self cofactor() const;
     
    /** 
     * Return the minor determinant (i,j) of the current matrix
     * 
     * @param i row index
     * @param j column index
     * 
     * @return the minor (i,j)
     */
    Component minorDeterminant(const DGtal::Dimension i, 
			       const DGtal::Dimension j) const;

    /** 
     * Returns the determinant of square matrix.
     * Slow method for large matrices.
     * @pre this must be NxN
     * 
     * @return the determinant.
     */
    Component determinant() const;

  
    /**
     * Destructor.
     */
    ~SimpleMatrix();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
 
    
    // ------------------------- Private Datas --------------------------------
  private:

    ///Matrix values containers.
    boost::array< Component, M*N>  myValues;

    ///Static computation of cofactor coefficients
    /// @todo should be static 
    boost::array< Component, M*N>  myCofactorCoefs;

    // ------------------------- Hidden services ------------------------------
  protected:

  }; // end of class SimpleMatrix


  /**
   * Overloads 'operator<<' for displaying objects of class 'SimpleMatrix'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SimpleMatrix' to write.
   * @return the output stream after the writing.
   */
  template <typename T, DGtal::Dimension M, DGtal::Dimension N>
  std::ostream&
  operator<< ( std::ostream & out, const SimpleMatrix<T,M,N> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/SimpleMatrix.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SimpleMatrix_h

#undef SimpleMatrix_RECURSES
#endif // else defined(SimpleMatrix_RECURSES)
