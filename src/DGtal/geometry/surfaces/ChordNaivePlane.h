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
 * @file ChordNaivePlane.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 * @author Yan Gérard
 * @author Isabelle Debled-Rennesson
 * @author Paul Zimmermann
 *
 * @date 2012/09/20
 *
 * Header file for module ChordNaivePlane.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ChordNaivePlane_RECURSES)
#error Recursive header files inclusion detected in ChordNaivePlane.h
#else // defined(ChordNaivePlane_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ChordNaivePlane_RECURSES

#if !defined ChordNaivePlane_h
/** Prevents repeated inclusion of headers. */
#define ChordNaivePlane_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ChordNaivePlane
  /**
   * Description of template class 'ChordNaivePlane'. \brief Aim: A
   * class that contains the chord-based algorithm for recognizing
   * pieces of digital planes of given axis width [ Gerard,
   * Debled-Rennesson, Zimmermann, 2005 ]. When the width is 1, it
   * corresponds to naive planes. The axis is specified at
   * initialization of the object. 
   *
   * This class is an implementation of Gerard, Debled-Rennesson,
   * Zimmermann, 2005: An elementary digital plane recognition
   * algorithm, @cite Gerard_2005_dam.
   *
   * As a (3D) geometric primitive, it obeys to a subset of the
   * concept CSegmentComputer. It is copy constructible,
   * assignable. It is iterable (inner type ConstIterator, begin(),
   * end()). It has methods \ref extend(), extend( InputIterator,
   * InputIterator) and \ref isExtendable(),
   * isExtendable(InputIterator, InputIterator).  The object stores
   * all the distinct points \c p such that 'extend( \c p )' was
   * successful. It is thus a model of boost::ForwardContainer (non
   * mutable).
   *
   * It is also a model of CPointPredicate (returns 'true' iff a point
   * is within the current bounds).
   *
   * \par Note on complexity: 
   * According to the paper, the worst-case complexity is \f$ O(n^7)
   * \f$ (in its non-incremental form). However, the observed complexity is quasi-linear. 
   *
   * \par Note on execution times: 
   * The user should favor int32_t or int64_t instead of BigInteger
   * whenever possible. When the point components are smaller than
   * 14000, int32_t are sufficient. For point components smaller than
   * 440000000, int64_t are sufficient. For greater diameters, it is
   * necessary to use BigInteger.
   *
   * @tparam TPoint specifies the type of input points (digital or not). 
   *
   * @tparam TInternalScalar specifies the type of scalar used in
   * internal computations, generally a more precise type than
   * TPoint::Component. For instance, for digital points, the type
   * should be able to hold integers of order (2*D^3) if D is the
   * diameter of the set of digital points.
   *
   @code
   typedef SpaceND<3,int> Z3;
   typedef ChordNaivePlane< Z3::Point, int64_t > NaivePlane;
   NaivePlane plane;
   plane.init( 2, 1, 1 ); // axis is z, width is 1/1 => naive 
   plane.extend( Point( 10, 0, 0 ) ); // return 'true'
   plane.extend( Point( 0, 8, 0 ) );  // return 'true'
   plane.extend( Point( 0, 0, 6 ) );  // return 'true'
   plane.extend( Point( 5, 5, 5 ) );  // return 'false'
   // There is no naive plane going through the 3 first points and the last one.
   @endcode
   *
   * Model of boost::DefaultConstructible, boost::CopyConstructible,
   * boost::Assignable, boost::ForwardContainer, CPointPredicate.
   */
  template < typename TPoint, 
             typename TInternalScalar >
  class ChordNaivePlane
  {

    // BOOST_CONCEPT_ASSERT(( CPoint< TPoint > ));
    BOOST_CONCEPT_ASSERT(( CSignedNumber< TInternalScalar > ));
    BOOST_STATIC_ASSERT(( TPoint::dimension == 3 ));

    // ----------------------- public types ------------------------------
  public:
    typedef TPoint Point;
    typedef TInternalScalar InternalScalar;
    typedef Point Vector;
    typedef typename Vector::Component Component;
    typedef typename Point::Coordinate Coordinate;
    typedef InternalScalar InternalVector[ 3 ];

    typedef std::set< Point > PointSet;
    typedef typename PointSet::size_type Size;
    typedef typename PointSet::const_iterator ConstIterator;
    typedef typename PointSet::iterator Iterator;

    // ----------------------- std public types ------------------------------
  public:
    typedef typename PointSet::const_iterator const_iterator;
    typedef typename PointSet::const_pointer const_pointer;
    typedef typename PointSet::const_reference const_reference;
    typedef typename PointSet::value_type value_type;
    typedef typename PointSet::difference_type difference_type;
    typedef typename PointSet::size_type size_type;

    // ----------------------- internal types ------------------------------
  private:
    /**
       Defines the state of the algorithm, the part of the data that
       may change after initialization of the ChordNaivePlane
       object. Only the set of points is not stored here.
    */
    struct State {
      InternalScalar height;   /**< current height of the topmost point. */
      Point A,B,C;             /**< current triangle, A topmost. */
      InternalVector N;        /**< current normal vector, i.e. (a,b,c). */
      Point ptMax;             /**< 3D point giving the max dot product. */
      Point ptMin;             /**< 3D point giving the min dot product. */
      InternalScalar max;      /**< current max dot product value, i.e. <= mu + c. */
      InternalScalar min;      /**< current min dot product value, i.e. mu by convention. */
      unsigned int nbValid;    /**< 0 when object is initialized, 1 when points are aligned with main axis, 2 when points form a triangle containing the main axis direction, 3 when there are at least 3 points that form a triangle not aligned with the main axis direction. */
    };

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~ChordNaivePlane();

    /**
     * Constructor. The object is not valid and should be initialized.
     * @see init
     */
    ChordNaivePlane();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ChordNaivePlane ( const ChordNaivePlane & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ChordNaivePlane & operator= ( const ChordNaivePlane & other );

    /**
     * Clear the object, free memory. The plane keeps its main axis,
     * diameter and width, but contains no point.
     */
    void clear();

    /**
     * All these parameters cannot be changed during the process.
     * After this call, the object is in a consistent state and can
     * accept new points for recognition. Calls clear so that the
     * object is ready to be extended.
     *
     * @param axis the main axis (0,1,2) for x, y or z.
     *
     * @param widthNumerator the maximal axis-width (x,y,or z) for the
     * plane is defined as the rational number \a widthNumerator / \a
     * widthDenominator (default is 1/1, i.e. naive plane).
     *
     * @param widthDenominator the maximal axis-width (x,y,or z) for
     * the plane is defined as the rational number \a widthNumerator /
     * \a widthDenominator (default is 1/1, i.e. naive plane).
     */
    void init( Dimension axis,
               InternalScalar widthNumerator = NumberTraits< InternalScalar >::ONE, 
               InternalScalar widthDenominator = NumberTraits< InternalScalar >::ONE );

    /**
       Useful to compute the axis width of a given range of points (public static version).

       @tparam TInputIterator any model of InputIterator on Point.
       
       @param[in] axis the main axis (0,1,2) for x, y or z.
       @param[in] itB an iterator on the first element of the range of 3D points.
       @param[in] itE an iterator after the last element of the range of 3D points.
       @return the axis width as a pair numerator/denominator.
    */
    template <typename TInputIterator>
    static 
    std::pair<InternalScalar, InternalScalar>
    computeAxisWidth( Dimension axis, TInputIterator itB, TInputIterator itE );


    //-------------------- model of ForwardContainer -----------------------------
  public:

    /**
     * @return the number of distinct points in the current naive plane.
     */
    Size size() const;

    /**
     * @return 'true' if and only if this object contains no point.
     */
    bool empty() const;

    /**
     * @return a const iterator pointing on the first point stored in the current naive plane.
     */
    ConstIterator begin() const;

    /**
     * @return a const iterator pointing after the last point stored in the current naive plane.
     */
    ConstIterator end() const;

    /**
     * NB: std version.
     * @return the maximal allowed number of points in the current naive plane.
     * @see maxSize
     */
    Size max_size() const;

    /**
     * same as max_size
     * @return the maximal allowed number of points in the current naive plane.
     */
    Size maxSize() const;


    //-------------------- model of CPointPredicate -----------------------------
  public:

    /**
     * Checks if the point \a p is in the current digital
     * plane. Therefore, a ChordNaivePlane is a model of
     * CPointPredicate.
     *
     * @param p any 3D point.
     *
     * @return 'true' if it is in the current plane, false otherwise.
     */
    bool operator()( const Point & p ) const;

    //-------------------- model of CIncrementalPrimitiveComputer -----------------------------
  public:

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

    /**
     * Checks if we have still a digital plane of specified width when
     * adding point \a p. The object is left unchanged whatever the
     * returned value. The invariant is 'this->isExtendable( p ) ==
     * true <=> this->extend( p ) == true'.
     *
     * @param p any 3D point (in the specified diameter).
     *
     * @return 'true' if this is still a plane, 'false' otherwise.
     */
    bool isExtendable( const Point & p ) const;

    //-------------------- model of CAdditivePrimitiveComputer -----------------------------
  public:

    /**
     * Adds the range of points [\a it, \a itE) and checks if we have
     * still a digital plane of specified width. The plane parameters
     * may be updated so as to include all the new points. All points
     * pointed by iterators should be in the diameter of this object.
     *
     * @tparam TInputIterator any model of InputIterator on Point.
     * @param[in] itB an iterator on the first element of the range of 3D points.
     * @param[in] itE an iterator after the last element of the range of 3D points.
     *
     * @return 'true' if it is still a plane, 'false' otherwise (the
     * object is then in its original state).
     */
    template <typename TInputIterator>
    bool extend( TInputIterator itB, TInputIterator itE );

    /**
     * Checks if we have still a digital plane of specified width when
     * adding the range of points [\a it, \a itE). The object is left
     * unchanged whatever the returned value.  All points pointed by
     * iterators should be in the diameter of this object. The
     * invariant is 'this->isExtendable( it, itE ) == true <=>
     * this->extend( it, itE ) == true'.
     *
     * @tparam TInputIterator any model of InputIterator on Point.
     * @param[in] itB an iterator on the first element of the range of 3D points.
     * @param[in] itE an iterator after the last element of the range of 3D points.
     *
     * @return 'true' if this is still a plane, 'false' otherwise.
     */
    template <typename TInputIterator>
    bool isExtendable( TInputIterator itB, TInputIterator itE ) const;

    /**
       Useful to check if a given set of points has a valid axis width (public version).

       @tparam TInputIterator any model of InputIterator on Point.
       
       @param[in] itB an iterator on the first element of the range of 3D points.
       @param[in] itE an iterator after the last element of the range of 3D points.
       @return 'true' iff the set of points given by range [itB,itE) has a valid width.
    */
    template <typename TInputIterator>
    bool
    satisfies( TInputIterator itB, TInputIterator itE ) const;

    /**
       Useful to compute the axis width of a given range of points (public version).

       @tparam TInputIterator any model of InputIterator on Point.
       
       @param[in] itB an iterator on the first element of the range of 3D points.
       @param[in] itE an iterator after the last element of the range of 3D points.
       @return the axis width as a pair numerator/denominator.
    */
    template <typename TInputIterator>
    std::pair<InternalScalar, InternalScalar>
    axisWidth( TInputIterator itB, TInputIterator itE ) const;


    //-------------------- Parameters services -----------------------------
  public:

    /**
     * @tparam Vector3D any type T such that T.operator[](int i)
     * returns a reference to a double. i ranges in 0,1,2.
     *
     * @param[in,out] normal the current normal vector 
     */
    template <typename Vector3D>
    void getNormal( Vector3D & normal ) const;

    /**
     * @tparam Vector3D any type T such that T.operator[](int i)
     * returns a reference to a double. i ranges in 0,1,2.
     *
     * @param[in,out] normal the current unit normal vector 
     */
    template <typename Vector3D>
    void getUnitNormal( Vector3D & normal ) const;

    /**
     * If n is the unit normal to the current plane, then n.x >= min
     * and n.x <= max are the two half-planes defining it.
     *
     * @param min the lower bound (corresponding to the unit vector).
     * @param max the upper bound (corresponding to the unit vector).
     */
    void getBounds( double & min, double & max ) const;

    /**
     * @pre ! empty()
     * @return the current minimal point of the plane, i.e. the one
     * with the smallest scalar product with the current normal
     * vector. Note that other points may also have a minimum value.
     */
    const Point & minimalPoint() const;

    /**
     * @pre ! empty()
     * @return the current maximal point of the plane, i.e. the one
     * with the highest scalar product with the current normal
     * vector. Note that other points may also have a maximum value.
     */
    const Point & maximalPoint() const;

    // ----------------------- Utilities --------------------------------------
  public:

    /**
       Inner product with potentially better precision.
       @param u any vector
       @param v any vector
     */
    template <typename TVector1, typename TVector2>
    static InternalScalar internalDot( const TVector1 & u, const TVector2 & v );

    /**
       Cross product with potentially better precision.
       @param[out] n the vector that stores the cross product of u and v.
       @param u any vector
       @param v any vector
     */
    template <typename TVector1, typename TVector2>
    static void internalCross( InternalVector & n, const TVector1 & u, const TVector2 & v );

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
    Dimension z;               /**< the main axis used in all subsequent computations. */
    Dimension x,y;             /**< the two other axes used in all subsequent computations. */
    InternalScalar myWidth0;   /**< the plane width as a positive rational number myWidth0/myWidth1 */
    InternalScalar myWidth1;   /**< the plane width as a positive rational number myWidth0/myWidth1 */
    mutable PointSet myPointSet;/**< the set of points within the plane, mutable since its state may temporarily be changed during some computations. */ 
    State myState;             /**< the current state that defines the plane being recognized. */
    mutable State _state;      /**< Temporary state used in computations. */
    mutable InternalScalar _d; /**< temporary variable used in some computations. */
    // ------------------------- Hidden services ------------------------------
  protected:


    // ------------------------- Internals ------------------------------------
  private:

    /**
     * Computes the min and max values/arguments of the scalar product
     * between the normal state.N and the points in the range
     * [itB,itE). Overwrites state.min, state.max at the start.
     *
     * @tparam TInputIterator any model of InputIterator.
     * @param state (modified) the state where the normal N is used in
     * computation and where fields state.min, state.max,
     * state.ptMin, state.ptMax are updated.
     *
     * @param itB an input iterator on the first point of the range.
     * @param itE an input iterator after the last point of the range.
     */
    template <typename TInputIterator>
    void computeMinMax( State & state, TInputIterator itB, TInputIterator itE ) const;

    /**
     * Updates the min and max values/arguments of the scalar product
     * between the normal state.N and the points in the range
     * [itB,itE). Do not overwrite state.min, state.max at the start.
     *
     * @tparam TInputIterator any model of InputIterator.
     *
     * @param state (modified) the state where the normal N is used in
     * computation and where fields state.min, state.max,
     * state.ptMin, state.ptMax are updated.
     *
     * @param itB an input iterator on the first point of the range.
     * @param itE an input iterator after the last point of the range.
     * @return 'true' if any of the fields state.min, state.max,
     * state.ptMin, state.ptMax have been updated, 'false'
     * otherwise.
     */
    template <typename TInputIterator>
    bool updateMinMax( State & state, TInputIterator itB, TInputIterator itE ) const;

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
       Sets up a consistent state with only one point.
       @param p1 any point.
    */
    bool setUp1( const Point & p1 );

    /**
       Sets up a consistent initial normal direction given the output
       of findTriangle (state.nbValid, state.A, state.B, state.C). 

       @param[in,out] the modified state.
    */
    void setUpNormal( State & state ) const;

    /**
       Sets up a consistent initial normal direction given the output
       of findTriangle (state.nbValid == 1).

       @param[in,out] the modified state.
    */
    void setUpNormal1( State & state ) const;

    /**
       Sets up a consistent initial normal direction given the output
       of findTriangle (state.nbValid == 2, state.A).

       @param[in,out] the modified state.
    */
    void setUpNormal2( State & state ) const;

    /**
       Sets up a consistent initial normal direction given the output
       of findTriangle (state.nbValid == 3, state.A, state.B, state.C). 

       @param[in,out] the modified state.
    */
    void setUpNormal3( State & state ) const;

    /**
       Computes the orientation of vectors AB and AC viewed from the
       main axis vector. It is the sign of the 2D determinant of AB
       and AC.
       @param A any point.
       @param B any point.
       @param C any point.

       @return the sign of det(ab,ac), where a,b,c are the 2d
       projections of A,B,C along main axis.
    */
    int signDelta( const Point & A, const Point & B, const Point & C ) const;

    /**
       Computes the orientation of vectors AO and AC viewed from the
       main axis vector. It is the sign of the 2D determinant of AO
       and AC, where O is the origin.
       @param A any point.
       @param C any point.

       @return the sign of det(ao,ac), where a,o,c are the 2d
       projections of A,O,C along main axis.
    */
    int signDelta( const Point & A, const Point & C ) const;

    /**
       Puts in (A,B,C) the new current triangle. We choose the
       triangle of the tetrahedron A,B,C,M having the highest
       intersection with Oz. 

       @param state the current state of the algorithm, state.A,
       state.B and state.C should represent the current triangle.

       @param M should be the vector P2-P1, where P2 is the point
       maximizing the dot product with the normal and P1 is the point
       minimizing the same dot product.

       @return 'true' if a consistent new triangle was found, 'false'
       if error (M was not correct in this case).

       @pre state.nbValid >= 2

    */
    bool newCurrentTriangle( State & state, const Point & M ) const;

    /**
       @tparam TInputIterator any model of boost::InputIterator on Point.

       Initializes the Chord algorithm by determining an initial
       triangle of the convex chord set (S+(-S)) if S is the set of
       points. If points have special alignment with the main axis, it
       may not return a triangle.

       @param[in] itB the beginning of the range of input points.
       @param[in] itE the end of the range of input points.

       @param[out] state contains the points forming a triangle not
       containing the main axis (state.A, state.B, state.C) if one was
       found, otherwise a segment not containing the main axis
       (state.A, state.B), otherwise the first point (state.A).

       @return 0 if object is initialized and there is no points, 1
       when all points are aligned with main axis, 2 when all points
       form a triangle containing the main axis direction, 3 when
       there are at least 3 points that form a triangle not aligned
       with the main axis direction.
    */
    template <typename TInputIterator>
    unsigned int findTriangle( State & state, TInputIterator itB, TInputIterator itE ) const;

    template <typename TInputIterator>
    unsigned int findTriangle1( State & state, TInputIterator itB, TInputIterator itE ) const;
    template <typename TInputIterator>
    unsigned int findTriangle2( State & state, TInputIterator itB, TInputIterator itE ) const;
    template <typename TInputIterator1, typename TInputIterator2>
    unsigned int findMixedTriangle( State & state, 
                                    TInputIterator1 itB1, TInputIterator1 itE1,
                                    TInputIterator2 itB2, TInputIterator2 itE2 ) const;

    /**
       @param p1 any point.
       @param p2 any point.
       @return 'true' iff the two points form a vector aligned with the main axis.
     */
    bool alignedAlongAxis( const Point & p1, const Point & p2 ) const;

    /**
       @param[in,out] state the field state.height is updated and
       contains the main axis height of the current triangle.
    */
    void computeHeight( State & state ) const;
    
    /**
       Computes a new normal according to the given values of state.A, state.B, state.C.

       @param[in,out] state the field state.normal is updated and
       contains the new normal.

       @pre state.nbValid >= 2
     */
    void computeNormal( State & state ) const;

    /**
       Useful to check if a given set of points has a valid axis width.

       @tparam TInputIterator any model of InputIterator on Point.
       
       @param[out] the resulting state, if you need to have a look.
       @param[in] itB an iterator on the first element of the range of 3D points.
       @param[in] itE an iterator after the last element of the range of 3D points.
       @return 'true' iff the set of points given by range [itB,itE) has a valid width.
    */
    template <typename TInputIterator>
    bool
    satisfies( State & state, TInputIterator itB, TInputIterator itE ) const;


    /**
       Useful to compute the axis width of a given range of points.

       @tparam TInputIterator any model of InputIterator on Point.
       
       @param[out] the resulting state, if you need to have a look.
       @param[in] itB an iterator on the first element of the range of 3D points.
       @param[in] itE an iterator after the last element of the range of 3D points.
       @return the axis width as a pair numerator/denominator.
    */
    template <typename TInputIterator>
    std::pair<InternalScalar, InternalScalar>
    axisWidth( State & state, TInputIterator itB, TInputIterator itE ) const;


    /**
       @param[in,out] state the current state of the algorithm (fields
       state.min, state.max, state.height are used)

       @return 'true' iff the current state indicates that the set of
       points is included between two planes of axis width smaller
       than the width specified at initialization. It is the value:
       state.max - state.min <= state.height.

       @see init
     */
    bool checkWidth( const State & state ) const;

    /**
     * Writes/Displays the object on an output stream (debug purposes).
     * @param out the output stream where the object is written.
     * @param state a given state for the display.
     */
    void selfDisplay ( std::ostream & out, const State & state ) const;


  }; // end of class ChordNaivePlane


  /**
   * Overloads 'operator<<' for displaying objects of class 'ChordNaivePlane'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ChordNaivePlane' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint, typename TInternalScalar>
  std::ostream&
  operator<< ( std::ostream & out, const ChordNaivePlane<TPoint, TInternalScalar> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/ChordNaivePlane.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ChordNaivePlane_h

#undef ChordNaivePlane_RECURSES
#endif // else defined(ChordNaivePlane_RECURSES)
