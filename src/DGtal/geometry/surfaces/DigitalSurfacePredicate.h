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
 * @file
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/07
 *
 * Header file for module DigitalSurfacePredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurfacePredicate_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfacePredicate.h
#else // defined(DigitalSurfacePredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfacePredicate_RECURSES

#if !defined DigitalSurfacePredicate_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfacePredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/UnorderedSetByBlock.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSurfacePredicate
  /**
   * Description of template class 'DigitalSurfacePredicate' <p>
   * \brief Aim: A point predicate which tells whether a point belongs to the
   * set of pointels of a given digital surface or not.
   *
   * Internally the set of pointels is stored in a DGtal::UnorderedSetByBlock.
   *
   * @tparam TSurface any digital surface type.
   *
     \b Models: A DigitalSurfacePredicate is a model of concepts::CPointPredicate.
   */
  template <typename TSurface>
  class DigitalSurfacePredicate
  {
    // ----------------------- Public types ------------------------------
  public:
      using Surface = TSurface;
      using Point   = typename Surface::Point;
      using Integer = typename Point::Coordinate;
      using KSpace  = typename Surface::KSpace;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     */
    DigitalSurfacePredicate();

    /**
     * Constructor.
     *
     * @param aSurface a digital surface.
     */
    DigitalSurfacePredicate(ConstAlias<Surface> aSurface);

    /**
     * Destructor.
     */
    ~DigitalSurfacePredicate();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalSurfacePredicate ( const DigitalSurfacePredicate & other );

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DigitalSurfacePredicate & operator= ( const DigitalSurfacePredicate & other );

    //-------------------- model of concepts::CPointPredicate -----------------------------
  public:
    /**
     * Test whether a point is a pointel of a digital surface or not.
     *
     * @param aPoint any digital point.
     * @return 'true' if the point is a pointel of the digital surface, false otherwise.
     */
    bool operator() (Point const& aPoint) const;

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

    // ------------------------- Protected Data ------------------------------
  protected:

    // ------------------------- Private Data --------------------------------
  private:
    CountedConstPtrOrConstPtr<Surface> mySurface; /**< A pointer on the digital surface. */
    UnorderedSetByBlock<Point> myPointSet; /**< The set of pointels. */

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:
    /**
     * Get the Khalimsky space associated with the digital surface.
     */
    KSpace const& space () const;

    /**
     * Computes the set of pointels, fills the myPointSet object.
     */
    void buildPointSet();

  }; // end of class DigitalSurfacePredicate


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfacePredicate'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfacePredicate' to write.
   * @return the output stream after the writing.
   */
  template <typename TSurface>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSurfacePredicate<TSurface> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/DigitalSurfacePredicate.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfacePredicate_h

#undef DigitalSurfacePredicate_RECURSES
#endif // else defined(DigitalSurfacePredicate_RECURSES)
