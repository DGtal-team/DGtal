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
 * @file ConvexIntegerPolygon.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/19
 *
 * Header file for module ConvexIntegerPolygon.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConvexIntegerPolygon_RECURSES)
#error Recursive header files inclusion detected in ConvexIntegerPolygon.h
#else // defined(ConvexIntegerPolygon_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConvexIntegerPolygon_RECURSES

#if !defined ConvexIntegerPolygon_h
/** Prevents repeated inclusion of headers. */
#define ConvexIntegerPolygon_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
#include "DGtal/math/arithmetic/ClosedIntegerHalfPlane.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexIntegerPolygon
  /**
     Description of template class 'ConvexIntegerPolygon' <p> \brief
     Aim: Represents a convex polygon in the two-dimensional digital
     plane.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable. It is also a
     model of boost::Container since it is a std::list of points. It
     is also a model of CDrawableWithBoard2D, and is displayable on a
     Board2D object.

     It contains no more data than a list of points except mutable
     data for intermediate computations.

     It is a backport of <a
     href="https://gforge.liris.cnrs.fr/projects/imagene">ImaGene</a>.

     @tparam TSpace an arbitrary 2-dimensional model of CSpace.
     @tparam TSequence a model of boost::Sequence whose elements are points (TSpace::Point). Default is list of points.
   */
  template < typename TSpace, 
             typename TSequence = std::list< typename TSpace::Point > >
  class ConvexIntegerPolygon 
    : public TSequence
  {
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
    BOOST_CONCEPT_ASSERT(( boost::Sequence< TSequence > ));

  public:
    typedef ConvexIntegerPolygon<TSpace,TSequence> Self;
    typedef TSequence Base;

    typedef TSpace Space;
    typedef typename Space::Integer Integer;
    typedef typename Space::Point Point;
    typedef typename Space::Vector Vector;
    typedef IntegerComputer<Integer> MyIntegerComputer;
    typedef HyperRectDomain< Space > Domain; 
    typedef ClosedIntegerHalfPlane< Space > HalfSpace;

    typedef typename Base::value_type Value;
    typedef typename Base::iterator Iterator;
    typedef typename Base::const_iterator ConstIterator;

    // The sequence must contain points.
    BOOST_STATIC_ASSERT
    (( ConceptUtils::SameType< Value, Point >::value ));
    
    // Point2I and Point should be the same type.
    typedef typename MyIntegerComputer::Point2I Point2I;
    typedef typename MyIntegerComputer::Vector2I Vector2I;
    typedef typename MyIntegerComputer::Point3I Point3I;
    typedef typename MyIntegerComputer::Vector3I Vector3I;
    BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Point2I, Point >::value ));
    BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Vector2I, Vector >::value ));

  public:
    using Base::size;
    using Base::empty;
    using Base::clear;
    using Base::insert;
    using Base::erase;
    using Base::front;
    using Base::back;
    using Base::push_front;
    using Base::push_back;
    using Base::begin;
    using Base::end;
    using Base::rbegin;
    using Base::rend;

  public:
    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor.
     */
    ~ConvexIntegerPolygon();

    /**
     * Constructor.
     */
    ConvexIntegerPolygon();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ConvexIntegerPolygon ( const Base & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Base & other );

    /**
       @return the bounding domain of this polygon, i.e. the smallest
       bounding box containing all the points of this polygon.
    */
    Domain boundingBoxDomain() const;

    /**
     * Removes (duplicate) consecutive vertices.
     */
    void purge();

    /**
     * Inserts the point K to the convex polygon before position "pos".
     * @param pos any iterator
     * @param K the point to add
     * @return an iterator on the newly created element.
     */
    Iterator insertBefore( const Iterator & pos, const Point & K );

    /**
       adds the point K to the end of the polygon.
       @param K the point to add
     */
    void pushBack( const Point & K );

    /**
     * @return 2*area of polygon.
     */
    const Integer & twiceArea() const;

    /**
     * if the area of this polygon is not 0, computes centroid, else,
     * computes the middle of the straight line segment.
     *
     * The centroid is a 2D rational point but it is represented as a
     * 3D integer point (a/d,c/d) corresponds to (a,b,d).
     *
     * @return the centroid. The centroid is \b not in reduced form.
     *
     * @see centroid( const Integer & ) const
     */
    Point3I centroid() const;
    
    /**
       This form is faster than centroid if you have already computed the area.

       if \e area is not 0, computes centroid, else, computes the middle
       of the straight line segment.
     
       The centroid is a 2D rational point but it is represented as a
       3D integer point (a/d,c/d) corresponds to (a,b,d).

       @param twice_area the area*2 of this polygon.

       @see centroid() const
     */
    Point3I centroid( const Integer & twice_area ) const;

    // ----------------------- halfspace services -------------------------------
  public:

    /**
       Cuts the convex polygon with the given half-space constraint.
       
       @param hs any half-space constraint.
       @return 'true' if the polygon was modified, 'false' otherwise.
     */
    bool cut( const HalfSpace & hs );

    /**
       Computes the constraint of the form N.P<=c whose supporting
       line passes through point *it and *(it+1), such that the other
       points of the polygon are inside.

       @param it an iterator on a point of this polygon.
       @return the corresponding half-space.
     */
    HalfSpace halfSpace( ConstIterator it ) const;

    /**
       Computes the constraint of the form N.P<=c whose supporting
       line passes through A and B such that the point \a inP
       satisfies the constraint.
       
       @param A any point.
       @param B any point different from A.
       @param inP any point not on the straight line (AB).
       @return  the corresponding half-space.
     */
    HalfSpace halfSpace( const Point & A, const Point & B, const Point & inP ) const;

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

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /// A utility object to perform computation on integers. Need not
    /// to be copied when cloning this object. Avoids many dynamic
    /// allocations when using big integers.
    mutable MyIntegerComputer _ic;
    mutable Integer _a, _b, _c, _c1, _c3, _den, _g;
    mutable Point _A, _B, _A1, _B1, _A2, _B2;
    mutable Vector _N;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ConvexIntegerPolygon


  /**
   * Overloads 'operator<<' for displaying objects of class 'ConvexIntegerPolygon'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ConvexIntegerPolygon' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace, typename TSequence>
  std::ostream&
  operator<< ( std::ostream & out, 
               const ConvexIntegerPolygon<TSpace,TSequence> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/arithmetic/ConvexIntegerPolygon.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConvexIntegerPolygon_h

#undef ConvexIntegerPolygon_RECURSES
#endif // else defined(ConvexIntegerPolygon_RECURSES)
