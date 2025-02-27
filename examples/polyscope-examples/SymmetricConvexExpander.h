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
 * @file SymmetricConvexExpander.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2022/09/02
 *
 * This file is part of the DGtal library.
 */

#if defined(SymmetricConvexExpander_RECURSES)
#error Recursive header files inclusion detected in SymmetricConvexExpander.h
#else // defined(SymmetricConvexExpander_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SymmetricConvexExpander_RECURSES

#if !defined SymmetricConvexExpander_h
/** Prevents repeated inclusion of headers. */
#define SymmetricConvexExpander_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class SymmetricConvexExpander
  /**
   * Description of class 'SymmetricConvexExpander' <p>
   *
   * \brief Aim: SymmetricConvexExpander computes symmetric fully
   * convex subsets of a given digital set.
   *
   * @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   * @tparam TPointPredicate an arbitrary model of predicate Point -> bool
   **/

  template < concepts::CCellularGridSpaceND TKSpace,
             typename TPointPredicate >
  class SymmetricConvexExpander
  {
  public:
    typedef DigitalConvexity<TKSpace>       Self;
    typedef TKSpace                         KSpace;
    typedef TPointPredicate                 PointPredicate;
    typedef typename KSpace::Integer        Integer;
    typedef typename KSpace::Point          Point;
    typedef typename KSpace::Vector         Vector;
    typedef typename KSpace::Space          Space;
    typedef std::size_t                     Size;
    typedef DGtal::BoundedLatticePolytope < Space > LatticePolytope;
    typedef DGtal::BoundedRationalPolytope< Space > RationalPolytope;
    typedef std::vector<Point>              PointRange;
    typedef std::vector<Vector>             VectorRange;
    typedef std::unordered_set<Point>       PointSet;
    
    static const Dimension dimension = KSpace::dimension;

    typedef std::pair< Point, Integer >     Node;

    // Inversion order since priority queue output max element.
    struct NodeComparator {
      /// Default constructor. 
      NodeComparator() = default;
      // p < q iff p.second < q.second
      bool operator()( const Node& p, const Node& q ) const
      {
        return p.second > q.second;
      }
    };

    typedef std::priority_queue< Node, std::vector<Node>, NodeComparator > NodeQueue;

    /// Constructor from predicate and symmetry center point.
    SymmetricConvexExpander( const PointPredicate& predicate,
                             const Point& kcenter,
                             const Point& lo, 
                             const Point& hi )
      : myPredicate( &predicate ), myConvexity( lo, hi )
    {
      init( kcenter );
    }

    bool predicate( const Point& p ) const
    {
      return (*myPredicate)( p );
    }
    
    void init( const Point& kcenter )
    {
      myKCenter = kcenter;
      myPoints.clear();
      myQ = NodeQueue();
      myM.clear();
      myPerfectSymmetry       = true;
      myPerfectSymmetryRadius = 0;
      // The starting points depend on the parity of the coordinates of the center.
      // There are from 1 to 2^d starting points.
      PointRange points;
      const auto x = myKCenter[ 0 ];
      if ( x % 2 == 0 )
        points.push_back( Point::base( 0, x / 2 ) );
      else
        {
          points.push_back( Point::base( 0, (x-1) / 2 ) );
          points.push_back( Point::base( 0, (x+1) / 2 ) );
        }
      for ( Dimension k = 1; k < dimension; k++ )
        {
          const auto n = points.size();
          const auto y = myKCenter[ k ];
          if ( y % 2 == 0 )
            {
              for ( auto i = 0; i < n; i++ )
                points[ i ][ k ] = y / 2;
            }
          else
            {
              points.resize( 2*n );
              const auto z  = (y-1)/2;
              const auto z1 = z + 1;
              for ( auto i = 0; i < n; i++ )
                {
                  points[ i ][ k ] = z;
                  Point q = points[ i ];
                  q[ k ]  = z1;
                  points[ i+n ]    = q;
                }                  
            }
        }
      // Keep only the points that satisfy the predicate.
      for ( auto&& p : points )
        {
          const Point sp = symmetric( p );
          if ( ! myM.count( p )
               && predicate( p ) && predicate( sp ) )
            {
              Node n( p, (2*p - myKCenter).squaredNorm() );
              myQ.push( n );
              myM.insert( p );
              myM.insert( sp );
            }
        }
    }

    /// Advance of one symmetric point.
    bool advance( bool enforce_full_convexity )
    {
      while ( ! finished() )
        {
          const auto p = current().first;
          //NOT USED const auto d = current().second; // current ring distance
          const auto sp = symmetric( p );
          myPoints.insert( p );
          myPoints.insert( sp );
          PointRange X( myPoints.cbegin(), myPoints.cend() );
          if ( enforce_full_convexity && ! myConvexity.isFullyConvex( X ) )
            {
              myPoints.erase( p );
              myPoints.erase( sp );
              ignore();
            }
          else
            {
              expand();
              return true;
            }
        }
      return false;
    }
    
    /**
       @return a const reference on the current visited vertex. The
       node is a pair <Vertex,Data> where the second term is the
       topological distance to the start vertex or set.

       NB: valid only if not 'finished()'.
     */
    const Node& current() const
    {
      return myQ.top();
    }

    /**
       Goes to the next vertex but ignores the current vertex for
       determining the future visited vertices. Otherwise said, no
       future visited vertex will have this vertex as a father.

       NB: valid only if not 'finished()'.
     */
    void ignore()
    {
      myQ.pop();
    }

    /**
       Goes to the next vertex and take into account the current
       vertex for determining the future visited vertices.
       NB: valid only if not 'finished()'.
     */
    void expand()
    {
      const Point p = current().first;
      myQ.pop();
      myPoints.insert( p );
      myPoints.insert( symmetric( p ) );
      const auto next_points = next( p );
      for ( auto&& q : next_points )
        {
          if ( ! myM.count( q ) )
            {
              const auto  sq   = symmetric( q );
              const bool  q_in = predicate( q );
              const bool sq_in = predicate( sq );
              if ( q_in && sq_in )
                {
                  Node n( q, (2*q - myKCenter).squaredNorm() );
                  myQ.push( n );
                  myM.insert(  q );
                  myM.insert( sq );
                  if ( myPerfectSymmetry )
                    myPerfectSymmetryRadius = std::max( myPerfectSymmetryRadius,
                                                        n.second );
                }
              else if ( ( q_in && ! sq_in ) || ( ! q_in && sq_in ) )
                {
                  myPerfectSymmetry = false;
                }
            }
        }
    }

    /**
       @return 'true' if all possible elements have been visited.
     */
    bool finished() const
    {
      return myQ.empty();
    }
    
    
    Point symmetric( const Point& p ) const
    {
      return myKCenter - p;
    }

    PointRange next( const Point& p ) const
    {
      PointRange N;
      Point d = 2*p - myKCenter;
      for ( Dimension i = 0; i < dimension; i++ )
        {
          if ( d[ i ] >= 0 ) N.push_back( p + Point::base( i, 1 ) );
          if ( d[ i ] <= 0 ) N.push_back( p - Point::base( i, 1 ) );
        }
      return N;
    }

    /// The predicate that every point must satisfy
    const PointPredicate* myPredicate;

    /// The digital convexity object
    DigitalConvexity< KSpace > myConvexity;
    
    /// Symmetry center (with doubled coordinates to represent half-integers).
    Point myKCenter;

    /// Symmetric range of lattice points, sorted.
    PointSet myPoints;

    /// Queue of points (only one point is inserted in the queue for a
    /// symmetric pair of point).
    NodeQueue  myQ;
    /// Marked points, i.e. points already in the queue or in the object.
    PointSet   myM;

    /// True iff the set and its local complement are symmetric.
    bool myPerfectSymmetry;
    /// Upper bound on the max distance of perfect symmetry.
    Integer myPerfectSymmetryRadius;
    
  };

  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SymmetricConvexExpander_h

#endif // !defined SymmetricConvexExpander_RECURSES
