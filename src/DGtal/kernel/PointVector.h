#pragma once

/**
 * @file PointVector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 *
 * Header file for module PointVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PointVector_RECURSES)
#error Recursive header files inclusion detected in PointVector.h
#else // defined(PointVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointVector_RECURSES

#if !defined PointVector_h
/** Prevents repeated inclusion of headers. */
#define PointVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/array.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicTypes.h"


//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class PointVector
  /**
   * Description of class 'PointVector' <p>
   *
   * \brief Aim: Implements basic operations that will be used in  \ref Point  and \ref Vector classes.
   *
   */

  template<typename T, std::size_t N>
  class PointVector
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     *  Copy of the Boost::array iterator type
     *
     **/
    typedef typename boost::array<T,N>::iterator Iterator;
    typedef typename boost::array<T,N>::const_iterator ConstIterator;
    
    typedef T TValue;
    static const std::size_t Dimension = N;
    
    
    /**
     * Constructor.
     * \todo PointVector must be Virutal
     * \todo implement assignements with type conversion/cast
     */
    PointVector();

    /**
     * Constructor from array of values.
     *
     * @param ptrValues the array of values (should be at least as long as
     * the size of the vector)
     */
    PointVector( const T * ptrValues );


    /**
     * Destructor.
     */
    ~PointVector();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PointVector ( const PointVector & other );


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * PointVector begin() iterator.
     *
     **/
    Iterator begin() {
      return myArray.begin();
    }

    /**
     * PointVector end() iterator.
     *
     **/
    Iterator end() {
      return myArray.end();
    }


    /**
     * PointVector begin() const iterator.
     *
     **/
    ConstIterator begin() const {
      return myArray.begin();
    }

    /**
     * PointVector end() const iterator.
     *
     **/
    ConstIterator end() const {
      return myArray.end();
    }


    /**
     * Returns the size of the vector (i.e. the number of its
     * coefficients).
     * Same as getDimension
     */
    static std::size_t size();

    /**
     * Returns the size of the vector (i.e. the number of its
     * coefficients).
     */
    static std::size_t dimension();

    /**
     * Returns the  \a i-th coefficient of the vector.
     *
     * \pre The \a i index must lie between \a 0 and \a size() .
     *
     * \param i is the index of the retrieved coefficient.
     */
    const T& at ( std::size_t i ) const;

    /**
     * Returns a non-const reference to the \a i-th element of the
     * vector.
     *
     * \pre The \a i index must lie between \a 0 and \a size() .
     *
     * \param i is the index of the retrieved coefficient.
     */
    T& at ( std::size_t i );


    /**
     * Multiplies \a *this by the \a coeff scalar number.
     *
     * \param coeff is the factor \a *this get multiplied by.
     */
    PointVector<T,N>& operator*= ( T coeff );


    /**
     * Assignement Operator
     *
     * \param coeff is the factor \a *this get multiplied by.
     */
    PointVector<T,N>& operator= ( const PointVector<T,N>& aPointVector );

    /** 
     * Equality operator.
     * 
     * @param aPointVector Point/Vector to compare to this.
     *
     * @return true iff points are equal.
     */
    bool operator== ( const PointVector<T,N>& aPointVector ) const;
    
    /** 
     * Difference operator on Points/Vectors.
     * 
     * @param aPointVector the Point/Vector to compare to this.
     *
     * @return true iff this differs from aPointVector, false otherwise.
     */
    bool operator!= ( const PointVector<T,N>& aPointVector ) const;
    
    
    /** 
     * Comparison operator on Points/Vectors (LesserThan).
     * 
     * @param aPointVector the Point/Vector to compare to this.
     *
     * @return true iff this < aPointVector, false otherwise.
     */
    bool operator< ( const PointVector<T,N>& aPointVector ) const;
    
    /** 
     * Comparison operator on Points/Vectors (LesserOrEqualThan).
     * 
     * @param aPointVector the Point/Vector to compare to this.
     *
     * @return true iff this <= aPointVector, false otherwise.
     */
    bool operator<= ( const PointVector<T,N>& aPointVector ) const;
    
  
    /** 
     * Comparison operator on Points/Vectors (GreaterThan).
     * 
     * @param aPointVector the Point/Vector to compare to this.
     *
     * @return true iff this > aPointVector, false otherwise.
     */
    bool operator> ( const PointVector<T,N>& aPointVector ) const;
    
    /** 
     * Comparison operator on Points/Vectors (GreaterOrEqualThan).
     * 
     * @param aPointVector the Point/Vector to compare to this.
     *
     * @return true iff this >= aPointVector, false otherwise.
     */
    bool operator>= ( const PointVector<T,N>& aPointVector ) const;
    

    /**
     * Addition operator with assignement.
     *
     * \param v is the Point that gets added to \a *this.
     */
    PointVector<T,N>& operator+= ( const PointVector<T,N>& v );

    /**
     * Addition operator.
     *
     * \param v is the Point that gets added to \a *this.
     */
    PointVector<T,N> operator+ ( const PointVector<T,N>& v ) const;


    /**
     * Substraction operator with assignement.
     *
     * \param v is the Point that gets substracted to \a *this.
     */
    PointVector<T,N>& operator-= ( const PointVector<T,N>& v );

    /**
     * Substraction operator.
     * Point - Vector => Point
     *
     * \param v is the Point that gets added to \a *this.
     */
    PointVector<T,N> operator- ( const PointVector<T,N>& v ) const;

    /**
     * Resets all the values to zero.
     */
    void zero();


    /**
     * Specify the set of norm types
     *
     */
    enum NormType { L_2, L_1, L_infty };

    /**
     * Computes the norm of a point/vector.
     *
     * \param type specifies the type of norm to consider (see \ref NormType)
     * \return the norm of the point/vector
     */
    double norm ( NormType type = L_2 );


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

    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Hidden services ------------------------------
 private:

    ///Internal data-structure: boost/array with constant size.
    boost::array<T,N> myArray;


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class PointVector

/**
 * Overloads 'operator<<' for displaying objects of class 'PointVector'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'PointVector' to write.
 * @return the output stream after the writing.
 */
template<typename T, std::size_t N>
inline
std::ostream&
operator<<( std::ostream & out, const PointVector<T,N> & object )
{
 object.selfDisplay( out );
 return out;
}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/PointVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PointVector_h

#undef PointVector_RECURSES
#endif // else defined(PointVector_RECURSES)
