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
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/arithmetic/ConvexIntegerPolygon.h"
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
   * end()). It has methods extend(Point), extend( InputIterator, InputIterator)
   * and isExtendable(Point), isExtendable(InputIterator, InputIterator).
   * It is also a model of CPointPredicate.
   *
   * @tparam TSpace specifies the type of digital space in which lies
   * input digital points. A model of CSpace.
   *
   * @tparam TInternalInteger specifies the type of integer used in internal
   * computations. The type should be able to hold integers of order
   * D^3 if D is the diameter of the set of digital points.
   *
   * Essentially a backport from [ImaGene](https://gforge.liris.cnrs.fr/projects/imagene).
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
    typedef typename Space::Size Size;
    typedef std::set< Point > PointSet;
    typedef typename PointSet::const_iterator ConstIterator;
    typedef typename PointSet::iterator Iterator;
    typedef TInternalInteger InternalInteger;
    typedef IntegerComputer< InternalInteger > MyIntegerComputer;

    // ----------------------- internal types ------------------------------
  private:
    typedef PointVector< 3, InternalInteger > InternalPoint3;
    typedef SpaceND< 2, InternalInteger > InternalSpace2;
    typedef typename InternalSpace2::Point InternalPoint2;
    typedef ConvexIntegerPolygon< InternalSpace2 > ConvexPolygonZ2;
    typedef typename ConvexPolygonZ2::HalfSpace HalfSpace;

    /**
       Defines the state of the algorithm, the part of the data that
       may change after initialization of the COBANaivePlane
       object. Only the set of points is not stored here.
    */
    struct State {
      InternalInteger max;     /**< current max dot product. */
      InternalInteger min;     /**< current min dot product. */
      ConstIterator indMax;    /**< 3D point giving the max dot product. */
      ConstIterator indMin;    /**< 3D point giving the min dot product. */
      ConvexPolygonZ2 cip;     /**< current constraint integer polygon. */
      InternalPoint3 centroid; /**< current centroid of cip. */
      InternalPoint3 N;        /**< current normal vector. */
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

    /**
       @return the object that performs integer calculation.
    */
    MyIntegerComputer & ic() const;

    /**
     * Clear the object, free memory. It is no more valid.
     */
    void clear();

    /**
     * All these parameters cannot be changed during the process.
     * After this call, the object is in a consistent state and can
     * accept new points for recognition. Call clear before
     * initializing everything.
     *
     * @param axis the main axis (0,1,2) for x, y or z.
     *
     * @param diameter the diameter for the set of points (maximum
     * distance between the given points)
     *
     * @param firstPoint the first point for initializing the plane
     * recognition algorithm.
     *
     * @param widthNumerator the maximal axis-width (x,y,or z) for the
     * plane is defined as the rational number \a widthNumerator / \a
     * widthDenominator (default is 1/1, i.e. naive plane).
     *
     * @param widthDenominator the maximal axis-width (x,y,or z) for
     * the plane is defined as the rational number \a widthNumerator /
     * \a widthDenominator (default is 1/1, i.e. naive plane).
     */
    void init( Dimension axis, InternalInteger diameter, 
               const Point & firstPoint,
               InternalInteger widthNumerator = NumberTraits< InternalInteger >::ONE, 
               InternalInteger widthDenominator = NumberTraits< InternalInteger >::ONE );


    /**
     * @return the number of distinct points in the current naive plane.
     */
    Size size() const;

    /**
     * @return the number of vertices/edges of the convex integer polygon of solutions.
     */
    Size complexity() const;

    /**
     * Checks if the point \a p is in the current digital
     * plane. Therefore, a COBANaivePlane is a model of
     * CPointPredicate.
     *
     * @param p any 3D point.
     *
     * @return 'true' if it is in the current plane, false otherwise.
     */
    bool operator()( const Point & p ) const;

    /**
     * Adds the point \a p to this plane if it is within the current
     * bounds. The plane parameters are not updated.
     *
     * @param p any 3D point (in the specified diameter).
     *
     * @return 'true' if \a p is in the plane, 'false' otherwise (the
     * object is then in its original state).
     */
    bool extendAsIs( const Point & p );

    /**
     * Adds the point \a p and checks if we have still a digital plane
     * of specified width. The plane parameters may be updated so as
     * to include the new point.
     *
     * @param p any 3D point (in the specified diameter).
     *
     * @return 'true' if it is still a plane, 'false' otherwise (the
     * object is then in its original state).
     */
    bool extend( const Point & p );


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
    InternalInteger myG;       /**< the grid step used in all subsequent computations. */
    InternalPoint2 myWidth;    /**< the plane width as a positive rational number myWidth[0]/myWidth[1] */
    PointSet myPointSet;       /**< the set of points within the plane. */ 
    State myState;             /**< the current state that defines the plane being recognized. */
    InternalInteger myCst1;    /**<  ( (int) ceil( get_si( myG ) * myWidth ) + 1 ). */
    InternalInteger myCst2;    /**<  ( (int) floor( get_si( myG ) * myWidth ) - 1 ). */
    mutable InternalInteger _v;/**< temporary variable used in computations. */
    mutable State _state;      /**< Temporary state used in computations. */
    mutable InternalPoint2 _grad; /**< temporary variable to store the current gradient. */
    // ------------------------- Hidden services ------------------------------
  protected:


    // ------------------------- Internals ------------------------------------
  private:

    /**
     * Recompute centroid of polygon of solution and deduce the
     * current normal vector.  It is called after any modification of
     * the convex polygon representing the set of solution.
     *
     * @param state (modified) the state where the fields state.cip are used in computation and where
     * fields state.centroid and state.N are updated.
     */
    void computeCentroidAndNormal( State & state );

    /**
     * Performs the double cut in parameter space according to the
     * current gradient and width. The centroid and normals are no
     * more valid (computeCentroidAndNormal should be called
     * afterwards).
     *
     * @param grad (altered, but not modified) the gradient used to
     * update the polygon of solutions state.cip.
     *
     * @param state (modified) the state where the fields
     * state.indMin, state.indMax, state.cip are used in computation
     * and where field state.cip is updated.
     */
    void doubleCut( InternalPoint2 & grad, State & state );

    /**
     * Computes the min and max values/arguments of the scalar product
     * between the normal state.N and the points in the range
     * [itB,itE). Overwrites state.min, state.max at the start.
     *
     * @tparam TInputIterator a model of boost::InputIterator on Point.
     *
     * @param state (modified) the state where the normal N is used in
     * computation and where fields state.min, state.max,
     * state.indMin, state.indMax are updated.
     *
     * @param itB an input iterator on the first point of the range.
     * @param itE an input iterator after the last point of the range.
     */
    template <typename TInputIterator>
    void computeMinMax( State & state, TInputIterator itB, TInputIterator itE );

    /**
     * Updates the min and max values/arguments of the scalar product
     * between the normal state.N and the points in the range
     * [itB,itE). Do not overwrite state.min, state.max at the start.
     *
     * @tparam TInputIterator a model of boost::InputIterator on Point.
     *
     * @param state (modified) the state where the normal N is used in
     * computation and where fields state.min, state.max,
     * state.indMin, state.indMax are updated.
     *
     * @param itB an input iterator on the first point of the range.
     * @param itE an input iterator after the last point of the range.
     * @return 'true' if any of the fields state.min, state.max,
     * state.indMin, state.indMax have been updated, 'false'
     * otherwise.
     */
    template <typename TInputIterator>
    bool updateMinMax( State & state, TInputIterator itB, TInputIterator itE );

    /**
     * @param state the state where the normal state.N, the scalars state.min and state.max are used in
     * computations.
     *
     * @return 'true' if the current width along state.N (computed
     * from the difference of state.max and state.min) is strictly
     * inferior to the maximal specified width (in myWidth), 'false'
     * otherwise.
     */
    bool checkPlaneWidth( const State & state ) const;

    /**
     * @param grad (updated) the value of a gradient used to cut the
     * polygon of solutions.
     *
     * @param state the state where the iterators state.indMin and
     * state.indMax are used in computations.
     */
    void computeGradient( InternalPoint2 & grad, const State & state ) const;

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
