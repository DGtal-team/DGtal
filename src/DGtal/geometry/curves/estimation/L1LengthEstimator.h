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
 * @file L1LengthEstimator.h
 * @brief Compute the length of a curve using the l_1 metric
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/27
 *
 * Header file for module L1LengthEstimator.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testL1LengthEstimator.cpp, testLengthEstimator.cpp
 */

#if defined(L1LengthEstimator_RECURSES)
#error Recursive header files inclusion detected in L1LengthEstimator.h
#else // defined(L1LengthEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define L1LengthEstimator_RECURSES

#if !defined L1LengthEstimator_h
/** Prevents repeated inclusion of headers. */
#define L1LengthEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class L1LengthEstimator
  /**
   * Description of template class 'L1LengthEstimator' <p>
   * \brief Aim: a simple model of CGlobalCurveEstimator that compute
   * the length of a curve using the l_1 metric (just add 1/h for
   * every step).
   *
   * Model of  CGlobalCurveGeometricEstimator
   *
   * @tparam TConstIterator a model of CConstIteratorOnArrows.
   */
  template <typename TConstIterator>
  class L1LengthEstimator
  {
    // ----------------------- Standard services ------------------------------
  public:


    ///@todo CONCEPT CHECK sur ConstIterator
    typedef TConstIterator ConstIterator;

    typedef double Quantity;


    /**
     * Default Constructor.
     */
    L1LengthEstimator() = default;


    /**
     * Destructor.
     */
    ~L1LengthEstimator() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    L1LengthEstimator ( const L1LengthEstimator & other ) = delete;

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    L1LengthEstimator & operator= ( const L1LengthEstimator & other ) = delete;


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
        const double h ) const;

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

  }; // end of class L1LengthEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'L1LengthEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'L1LengthEstimator' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const L1LengthEstimator<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/estimation/L1LengthEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined L1LengthEstimator_h

#undef L1LengthEstimator_RECURSES
#endif // else defined(L1LengthEstimator_RECURSES)
