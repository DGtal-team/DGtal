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
 * @file COBANaivePlane.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 * @author Emilie Charrier
 * @author Lilian Buzer
 *
 * @date 2012/09/20
 *
 * Header file for module COBANaivePlane.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(COBANaivePlane_RECURSES)
#error Recursive header files inclusion detected in COBANaivePlane.h
#else // defined(COBANaivePlane_RECURSES)
/** Prevents recursive inclusion of headers. */
#define COBANaivePlane_RECURSES

#if !defined COBANaivePlane_h
/** Prevents repeated inclusion of headers. */
#define COBANaivePlane_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/arithmetec/ConvexIntegerPolygon.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class COBANaivePlane
  /**
   * Description of template class 'COBANaivePlane' <p> \brief Aim: A
   * class that contains the COBA algorithm (Emilie Charrier, Lilian
   * Buzer, DGCI2008) for recognizing pieces of digital planes of given axis
   * width. When the width is 1, it corresponds to naive planes. The
   * axis is specified at initialization of the object.
   *
   * As a (3D) geometric primitive, it obeys to a subset of the
   * concept CSegmentComputer. It is copy constructible,
   * assignable. It is iterable (inner type ConstIterator, begin(),
   * end()). It has methods extend(Point), extend( Iterator, Iterator)
   * and isExtendable(Point), isExtendable(Iterator, Iterator).
   *
   * @tparam TSpace specifies the type of digital space in which lies
   * input digital points. A model of CSpace.
   *
   * @tparam TInternalInteger specifies the type of integer used in internal
   * computations. The type should be able to hold integers of order
   * D^3 if D is the diameter of the set of digital points.
   */
  template < typename TSpace, 
             typename TInternalInteger = DGtal::BigInteger >
  class COBANaivePlane
  {

    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( CInteger< TInternalInteger > ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 3 ));

    // ----------------------- public types ------------------------------
  public:
    typedef TSpace Space;
    typedef typename Space::Point Point;
    typedef std::set< Point > PointSet;
    typedef PointSet::const_iterator ConstIterator;

    // ----------------------- internal types ------------------------------
  private:
    typedef TInternalInteger InternalInteger;
    typedef PointVector< 3, InternalInteger > InternalPoint3;
    typedef SpaceND< 2, InternalInteger > InternalSpace2;
    typedef InternalSpace::Point InternalPoint2;
    typedef ConvexIntegerPolygon< InternalSpace2 > ConvexPolygonZ2;

    /**
       Defines the state of the algorithm, the part of the data that
       may change after initialization of the COBANaivePlane object.
    */
    struct State {
      PointSet pointSet;       /**< the set of points within the plane. */ 
      ConstIterator indMax;    /**< 3D point giving the max dot product. */
      ConstIterator indMin;    /**< 3D point giving the min dot product. */
      ConvexPolygonZ2 cip;     /**< current constraint integer polygon. */
      InternalPoint3 centroid; /**< current centroid of cip. */
      InternalPoint3 N;        /**< current normal vector. */
      InternalInteger max;     /**< current max dot product. */
      InternalInteger min;     /**< current min dot product. */
      InternalPoint2 grad;     /**< the current gradient. */
    };

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~COBANaivePlane();

    /**
     * Constructor. The object is not valid and should be initialized.
     * @see init
     */
    COBANaivePlane();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    COBANaivePlane ( const COBANaivePlane & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    COBANaivePlane & operator= ( const COBANaivePlane & other );

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

    // ------------------------- Private Datas --------------------------------
  private:
    Dimension myAxis;          /**< the main axis used in all subsequent computations. */
    Integer myG;               /**< the grid step used in all subsequent computations. */
    InternalPoint2 myWidth;    /**< the plane width as a positive rational number myWidth[0]/myWidth[1] */
    State myState;             /**< the current state that defines the plane being recognized. */

    // ------------------------- Hidden services ------------------------------
  protected:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class COBANaivePlane


  /**
   * Overloads 'operator<<' for displaying objects of class 'COBANaivePlane'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'COBANaivePlane' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace, typename TInternalInteger>
  std::ostream&
  operator<< ( std::ostream & out, const COBANaivePlane<TSpace, TInternalInteger> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/arithmetic/COBANaivePlane.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined COBANaivePlane_h

#undef COBANaivePlane_RECURSES
#endif // else defined(COBANaivePlane_RECURSES)
