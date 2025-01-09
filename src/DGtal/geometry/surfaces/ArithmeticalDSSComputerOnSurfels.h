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
 * @file ArithmeticalDSSComputerOnSurfels.h
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/01/22
 *
 * @brief Header file for module ArithmeticalDSSComputerOnSurfels.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithmeticalDSSComputerOnSurfels_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSSComputerOnSurfels.h
#else // defined(ArithmeticalDSSComputerOnSurfels_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSSComputerOnSurfels_RECURSES

#if !defined ArithmeticalDSSComputerOnSurfels_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSSComputerOnSurfels_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // class ArithmeticalDSSComputerOnSurfels
  /**
   * \brief Aim: This class is a wrapper around ArithmeticalDSS that is devoted
   * to the dynamic recognition of digital straight segments (DSS) along a 
   * sequence of surfels lying on a slice of the digital surface (i.e., the 
   * orthogonal direction of all surfels belong to a same plane, most pairs
   * of consecutive surfels share a common linel).
   *
   * @tparam TKSpace type of Khalimsky space
   * @tparam TIterator type of iterator on 3d surfels,
   * at least readable and forward.
   * @tparam TInteger type of integers used for the computation of remainders,
   * which is a model of concepts::CInteger.
   * @tparam adjacency an unsigned integer equal to 4 for standard
   * (simply 4-connected) DSS or 8 for naive (simply 8-connected) DSS (default).
   *
   * This class is a model of concepts::CDynamicBidirectionalSegmentComputer.
   * It is also default constructible, copy constructible, assignable and equality comparable.
   *
   * @see ArithmeticalDSS NaiveDSS8 StandardDSS4
   */
  template <typename TKSpace, typename TIterator,
    typename TInteger = typename TKSpace::Space::Integer>
  class ArithmeticalDSSComputerOnSurfels
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT(( TKSpace::dimension == 3 ));

    // ----------------------- inner types ------------------------------
  public:

    /**
     * Type of Khalimsky space
     */
    typedef TKSpace KSpace;

    /**
     * Type of signed cell
     */
    typedef typename KSpace::SCell  SCell; 

    /**
     * Type of iterator, at least readable and forward
     */
    typedef TIterator ConstIterator;
    BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<ConstIterator> ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::ForwardTraversalConcept<ConstIterator> ));
    BOOST_STATIC_ASSERT(( concepts::ConceptUtils::SameType< SCell, 
                          typename IteratorCirculatorTraits<ConstIterator>::Value >::value ));
    /**
     * Type of 3d digital point
     */
    typedef typename KSpace::Space::Point Point3;

    /**
     * Type of 2d digital point
     */
    typedef PointVector<2, TInteger> Point;

    /**
     * Type of 3d to 2d projector 
     */
    typedef functors::Projector<SpaceND<2,TInteger> > Projector; 
    
    /**
     * Type of coordinate
     */
    typedef typename Point::Coordinate Coordinate;
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<Coordinate> ));

    /**
     * Type of integer, devoted to remainders (and intercepts)
     */
    typedef TInteger Integer;
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<Integer> ));

    /**
     * Type of objects that represents DSSs
     */
    typedef ArithmeticalDSS<Coordinate, Integer, 4> DSS;
    //we expect that the iterator type returned DGtal points, used in the DSS representation
    BOOST_STATIC_ASSERT(( concepts::ConceptUtils::SameType< Point, typename DSS::Point >::value ));

    /**
     * Type of primitive representation, defined as an alias of DSS
     */
    typedef DSS Primitive;

    /**
     * Type of vector, defined as an alias of point
     */
    typedef Point Vector;

    /**
     * Alias of this class
     */
    typedef ArithmeticalDSSComputerOnSurfels<KSpace,ConstIterator,TInteger> Self;

    /**
     * Helpers used to extract relevant points from a pair of points
     */
    struct DirectPairExtractor {

      virtual Point first(const std::pair<Point,Point>& aPair) const { return aPair.first; }
      virtual Point second(const std::pair<Point,Point>& aPair) const { return aPair.second; }
      
    };
    struct IndirectPairExtractor : public DirectPairExtractor {
      
      Point first(const std::pair<Point,Point>& aPair) const { return  aPair.second; }
      Point second(const std::pair<Point,Point>& aPair) const { return aPair.first; }

    }; 
    typedef std::shared_ptr<DirectPairExtractor> PairExtractor;  
    
    /**
     * Reversed version of this class (using reverse iterators)
     */
    typedef ArithmeticalDSSComputerOnSurfels<KSpace,ReverseIterator<ConstIterator>,TInteger> Reverse;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     * not valid
     */
    ArithmeticalDSSComputerOnSurfels();

    /**
     * Constructor.
     * @param aKSpace a Khalimsky space
     * @param aDim1 a first direction that describes the projection plane
     * @param aDim2 a second direction that describes the projection plane
     * @param aFlagToReverse a boolean telling whether one has to reverse 
     * the orientation of the points associated to a surfel or not 
     * ('false' by default)
     */
    ArithmeticalDSSComputerOnSurfels(const KSpace& aKSpace, Dimension aDim1, Dimension aDim2, bool aFlagToReverse = false);
    
    /**
     * Initialisation.
     * @param it an iterator on 3D surfels
     */
    void init(const ConstIterator& it);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ArithmeticalDSSComputerOnSurfels ( const ArithmeticalDSSComputerOnSurfels & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSSComputerOnSurfels& operator= ( const ArithmeticalDSSComputerOnSurfels & other );

    /**
     * @return a default-constructed instance of Self
     */
    Self getSelf() const;

    /**
     * @return a default-constructed instance of Reverse
     */
    Reverse getReverse() const;

    /**
     * Equality operator.
     * @param other the object to compare with.
     * @return 'true' if the DSS representations
     * and the ranges of the two objects match,
     * 'false' otherwise
     */
    bool operator==( const ArithmeticalDSSComputerOnSurfels & other ) const;

    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
    bool operator!=( const ArithmeticalDSSComputerOnSurfels & other ) const;

    /**
     * Destructor.
     */
    ~ArithmeticalDSSComputerOnSurfels(){};

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Tests whether the current DSS can be extended at the front.
     *
     * @return 'true' if yes, 'false' otherwise.
     *
     * @warning the caller must be sure that the iterator returned  
     * by 'end()' can be safely dereferenced. 
     */
    bool isExtendableFront();

    /**
     * Tests whether the current DSS can be extended at the back.
     *
     * @return 'true' if yes, 'false' otherwise.
     */
    bool isExtendableBack();

    /**
     * Tests whether the current DSS can be extended at the front.
     * Computes the parameters of the extended DSS if yes.
     *
     * @return 'true' if yes, 'false' otherwise.
     *
     * @warning the caller must be sure that the iterator returned  
     * by 'end()' can be safely dereferenced. 
     */
    bool extendFront();

    /**
     * Tests whether the current DSS can be extended at the back.
     * Computes the parameters of the extended DSS if yes.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendBack();

    /**
     * Removes the front point of the DSS
     * if it has more than two points
     * @return 'true' if the front point is removed, 'false' otherwise.
     */
    bool retractFront();

    /**
     * Removes the back point of the DSS
     * if it has more than two points
     * @return 'true' if the back point is removed, 'false' otherwise.
     */
    bool retractBack();

    /**
     * Returns the ends of a unit segment corresponding 
     * to the projection of a given signed surfel. 
     *
     * @param aSurfel any signed surfel.
     * @return a pair of 2D points. 
     */
    std::pair<Point,Point> getProjectedPointsFromSurfel(SCell const& aSurfel) const; 

    /**
     * Front end of the projection of a given surfel. 
     *
     * @param aSurfel any signed surfel.
     * @return the second 2D point.
     * @see getProjectedPointsFromSurfel
     */
    Point getNextProjectedPoint(SCell const& aSurfel) const;
    
    /**
     * Back end of the projection of a given surfel. 
     *
     * @param aSurfel any signed surfel.
     * @return the second 2D point.
     * @see getProjectedPointsFromSurfel
     */
    Point getPreviousProjectedPoint(SCell const& aSurfel) const; 

    // ------------------------- Accessors ------------------------------
    /**
     * @return the current DSS representation.
     * NB: since we return a const reference, you must
     * copy the result, if you want to keep it beyond
     * the object's existence.
     */
    const Primitive& primitive() const;
    /**
     * @return a-parameter of the DSS
     */
    Integer a() const;
    /**
     * @return b-parameter of the DSS
     */
    Integer b() const;
    /**
     * @return mu-parameter of the DSS
     */
    Integer mu() const;
    /**
     * @return omega-parameter of the DSS
     */
    Integer omega() const;
    /**
     * @return first upper leaning point.
     */
    Point Uf() const;
    /**
     * @return last upper leaning point.
     */
    Point Ul() const;
    /**
     * @return first lower leaning point.
     */
    Point Lf() const;
    /**
     * @return last lower leaning point.
     */
    Point Ll() const;
    /**
     * @return the first point of the DSS.
     */
    Point back() const ;
    /**
     * @return the last point of the DSS.
     */
    Point front() const ;
    /**
     * @return begin iterator of the DSS range.
     */
    ConstIterator begin() const;
    /**
     * @return end iterator of the DSS range.
     */
    ConstIterator end() const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;


    // ------------------------- Hidden services ------------------------------
  private:

    /**
     * Returns the ends of a unit segment corresponding 
     * to the projection of a given signed linel. 
     *
     * @param aLinel any signed linel.
     * @return a pair of 2D points. 
     */
    std::pair<Point,Point> getProjectedPointsFromLinel(SCell const& aLinel) const; 
    
    /**
     * Returns the unique dimension in {0,1,2} \ {aDim1, aDim2}. 
     *
     * @param aDim1 a dimension
     * @param aDim2 a dimension
     */
    Dimension dimNotIn(Dimension const& aDim1, Dimension const& aDim2) const; 
    
    // ------------------------- Protected Datas ------------------------------
  protected:

    /**
     * (Pointer to) Khalimsky space
     */
    const KSpace* myKSpace;

    /**
     * A first direction that describes the projection plane
     */
    Dimension mySliceAxis1;
    
    /**
     * A second direction that describes the projection plane
     */
    Dimension mySliceAxis2;
    
    /**
     * A direction along which the points are projected
     * (and orthogonal to the projection plane) 
     */
    Dimension myProjectionAxis;

    /**
     * Functor that projects a 3D point to a 2D point along myProjectionAxis
     */
    Projector my2DProjector; 

    /**
     * Smart pointer on an object used to extract relevant points 
     * from a pair of points
     */
    PairExtractor myExtractor; 
    
    /**
    * DSS representation
    */
    DSS myDSS;
    
    /**
    * begin iterator
    */
    ConstIterator myBegin;
    
    /**
    * end iterator
    *
    * @warning the user must be sure that it can be safely dereferenced
    * before calling 'isExtendableFront' and 'extendFront'. 
    */
    ConstIterator myEnd;

    // ------------------ Display ------------------------------------------

  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;


  }; // end of class ArithmeticalDSSComputerOnSurfels


/**
 * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSSComputerOnSurfels'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithmeticalDSSComputerOnSurfels' to write.
 * @return the output stream after the writing.
 */
template <typename TKSpace, typename TIterator, typename TInteger>
std::ostream&
operator<< ( std::ostream & out,  const ArithmeticalDSSComputerOnSurfels<TKSpace,TIterator,TInteger> & object )
{
  object.selfDisplay( out);
  return out;
}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/surfaces/ArithmeticalDSSComputerOnSurfels.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSSComputerOnSurfels_h

#undef ArithmeticalDSSComputerOnSurfels_RECURSES
#endif // else defined(ArithmeticalDSSComputerOnSurfels_RECURSES)
