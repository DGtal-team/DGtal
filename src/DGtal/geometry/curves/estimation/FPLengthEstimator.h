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
 * @file FPLengthEstimator.h
 * @brief Computes the length of a digital curve using its FP (faithful polygon)
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/07
 *
 * Header file for module FPLengthEstimator.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testLengthEstimators.cpp, testL1LengthEstimator.cpp
 */

#if defined(FPLengthEstimator_RECURSES)
#error Recursive header files inclusion detected in FPLengthEstimator.h
#else // defined(FPLengthEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FPLengthEstimator_RECURSES

#if !defined FPLengthEstimator_h
/** Prevents repeated inclusion of headers. */
#define FPLengthEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/FP.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class FPLengthEstimator
  /**
   * Description of template class 'FPLengthEstimator' <p>
   * \brief Aim: a model of CGlobalCurveEstimator that computes
   * the length of a digital curve using its FP (faithful polygon)
   *
   * Model of CGlobalCurveGeometricEstimator

   * @tparam TConstIterator a model of CConstIteratorOnPoints.
   */
  template <typename TConstIterator>
  class FPLengthEstimator
  {
    // ----------------------- Standard services ------------------------------
  public:


    ///@todo CONCEPT CHECK sur ConstIterator
    typedef TConstIterator ConstIterator;

    typedef double Quantity;

    typedef FP<ConstIterator,int,4> FaithfulPolygon;
    typedef typename FaithfulPolygon::Point Point;
    typedef typename FaithfulPolygon::Vector Vector;

    /**
     * Default Constructor.
     */
    FPLengthEstimator() = default;


    /**
     * Destructor.
     */
    ~FPLengthEstimator() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    FPLengthEstimator ( const FPLengthEstimator & other ) = delete;

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    FPLengthEstimator & operator= ( const FPLengthEstimator & other ) = delete;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Computation of the l1 length of the curve.
     * Complexity: O(|Range|)
     * @param itb begin iterator
     * @param ite end iterator
     * @param h grid size (must be > 0).
     *
     * @return the curve length.
     */
    Quantity eval( const ConstIterator& itb,
        const ConstIterator& ite,
        const double h = 1. ) const;

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

  }; // end of class FPLengthEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'FPLengthEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FPLengthEstimator' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const FPLengthEstimator<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/estimation/FPLengthEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FPLengthEstimator_h

#undef FPLengthEstimator_RECURSES
#endif // else defined(FPLengthEstimator_RECURSES)
