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
 * @file TangencyComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/07/16
 *
 * Header file for module TangencyComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TangencyComputer_RECURSES)
#error Recursive header files inclusion detected in TangencyComputer.h
#else // defined(TangencyComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TangencyComputer_RECURSES

#if !defined TangencyComputer_h
/** Prevents repeated inclusion of headers. */
#define TangencyComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clone.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/kernel/LatticeSetByIntervals.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class TangencyComputer
  /**
     Description of template class 'TangencyComputer' <p> \brief Aim:
     A class that computes tangency to a given digital set. It
     provides services to compute all the cotangent points to a given
     point, or to compute shortest paths.

     @see moduleDigitalConvexityApplications

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   */
  template < concepts::CCellularGridSpaceND TKSpace >
  class TangencyComputer
  {
  public:
    typedef TangencyComputer< TKSpace > Self;
    typedef TKSpace                     KSpace;
    typedef typename KSpace::Space      Space;
    typedef typename KSpace::Point      Point;
    typedef typename KSpace::Vector     Vector;
    typedef HyperRectDomain< Space >    Domain;
    typedef std::size_t                 Index;
    typedef std::size_t                 Size;
    typedef std::vector< Index >        Path;
    typedef CellGeometry< KSpace >      CellCover;
    typedef LatticeSetByIntervals< Space > LatticeCellCover;
    
    // ------------------------- Shortest path services --------------------------------
  public:

    /// This structure is a state machine that computes shortest paths in a digital
    /// set. Internally, it references a TangencyComputer.
    struct ShortestPaths {
      /// Type used for Dijkstra's algorithm queue (point, ancestor, distance).
      typedef std::tuple< Index, Index, double > Node;

      /// Allows to compare two nodes (closest is popped first) by
      /// modeling a `greater than` relation (which is the relation
      /// used for `std::priority_queue` when you wish to have the
      /// smallest one outputed first).
      struct Comparator {
        /// @param p1 the first node
        /// @param p2 the second node
        /// @return 'true' iff node \a p1 is further away than node \a p2.
        bool operator()  ( const Node& p1,
                           const Node& p2 ) const
        {
          return std::get<2>( p1 ) > std::get<2>( p2 );
        }
      };

      /// Default constructor. The object is not valid.
      ShortestPaths()
        : myTgcyComputer( nullptr ), mySecure( 0.0 )
      {}

      /// Copy constructor
      /// @param other the object to clone.
      ShortestPaths( const ShortestPaths& other ) = default;
      /// Move constructor
      /// @param other the object to clone.
      ShortestPaths( ShortestPaths&& other ) = default;
      /// Assignment
      /// @param other the object to clone.
      /// @return a reference to 'this'
      ShortestPaths& operator=( const ShortestPaths& other ) = default;
      /// Move assignment
      /// @param other the object to clone.
      /// @return a reference to 'this'
      ShortestPaths& operator=( ShortestPaths&& other ) = default;
      
      /// Constructs a ShorthestPaths object and references the given
      /// TangencyComputer \a tgcy_computer. It initialized the object
      /// so that it can start a new computation.
      ///
      /// @param[in] tgcy_computer the tangency computer where
      /// shortest paths are computed.
      ///
      /// @param[in] secure This value is used to prune vertices in
      /// the bft. If it is greater or equal to \f$ \sqrt{d} \f$ where
      /// \a d is the dimension, the shortest path algorithm is
      /// guaranteed to output the correct result. If the value is
      /// smaller (down to 0.0), the algorithm is much faster but a
      /// few shortest path may be missed.
      ///
      /// @note Takes O(n) time complexity, where n is the number of
      /// points in the TangencyComputer object.
      ShortestPaths( ConstAlias< TangencyComputer > tgcy_computer,
                     double secure = sqrt( KSpace::dimension ) )
        : myTgcyComputer( &tgcy_computer ),
          mySecure( std::max( secure, 0.0 ) )
      {
        clear();
      }

      /// @return a pointer on the tangency computer.
      const TangencyComputer* tangencyComputerPtr() const
      {
        return myTgcyComputer;
      }
      
      /// @return the number of points in the associated tangency computer
      Index size() const
      {
        return myTgcyComputer->size();
      }
      
      /// Clears the object and prepares it for a shortest path
      /// computation.
      void clear()
      {
        const auto nb = size();
        myAncestor = std::vector< Index > ( nb, nb );
        myDistance = std::vector< double >( nb, std::numeric_limits<double>::infinity() );
        myVisited  = std::vector< bool >  ( nb, false );
        myQ        = std::priority_queue< Node, std::vector< Node >, Comparator >();
  }

      /// Adds the point with index \a i as a source point
      /// @param[in] i any valid index
      /// @note Must be done before starting computations.
      void init( Index i )
      {
        ASSERT( i < size() );
        myQ.push( std::make_tuple( i, i, 0.0 ) );
        myAncestor[ i ] = i;
        myDistance[ i ] = 0.0;
        myVisited [ i ] = true;
      }
                     
      /// Adds a range of indices as source points
      ///
      /// @param[in] it,itE a range of valid indices, which are the
      /// indices of source points.
      ///
      /// @note Must be done before starting computations.
      template < typename IndexFwdIterator >
      void init( IndexFwdIterator it, IndexFwdIterator itE )
      {
        for ( ; it != itE; ++it )
          {
            const auto i = *it;
            ASSERT( i < size() );
            myQ.push( std::make_tuple( i, i, 0.0 ) );
          }
        const auto elem = myQ.top();
        const auto i    = std::get<0>( elem );
        myAncestor[ i ] = i;
        myDistance[ i ] = 0.0;
        myVisited [ i ] = true;
      }

      /// @return 'true' if the traversal is finished, i.e. when the
      /// queue is empty.
      bool finished() const
      {
        return myQ.empty();
      }

      /// @return a const reference to the current node on top of the
      /// queue of bft, a triplet '(i,a,d)' where \a i is the index of
      /// the point, \a a is the index of its ancestor, \a d is the
      /// distance of \a i to the closest source.
      ///
      /// @pre valid only if not 'finished()'.
      const Node& current() const
      {
        ASSERT( ! finished() );
        return myQ.top();
      }

      /// Goes to the next point in the bft.
      ///
      /// @pre valid only if not already 'finished()'.
      /// @note The core method of shortest paths algorithm.
      void expand();

      
      /// @return 'true' if the object is valid, i.e. when its tangency computer exists.
      bool isValid() const
      {
        return myTgcyComputer != nullptr;
      }

      /// @param[in] i any valid index
      /// @return the point with index \a i.
      const Point& point( Index i ) const
      {
        ASSERT( i < size() );
        return myTgcyComputer->point( i );
      }

      /// @param[in] i any valid index
      /// @return the index of an ancestor to \a i.
      ///
      /// @pre The ancestor is valid only when `isVisited(i)` is true,
      /// so after it was a `current()` node and `expand()` has been
      /// called.
      Index ancestor( Index i ) const
      {
        ASSERT( i < size() );
        return myAncestor[ i ];
      }

      /// @param[in] i any valid index
      /// @return the distance of point \a i to the closest source.
      ///
      /// @pre The distance is correct only when `isVisited(i)` is true,
      /// so after it was a `current()` node and `expand()` has been
      /// called.
      double distance( Index i ) const
      {
        ASSERT( i < size() );
        return myDistance[ i ];
      }

      /// @param[in] i any valid index
      ///
      /// @return 'true' iff point is already visited by the bft,
      /// i.e. after it was a `current()` node and `expand()` has been
      /// called.
      bool isVisited( Index i ) const
      {
        return ancestor( i ) < size();
      }

      /// @return the infinity distance (point is not computed or unreachable)
      static double infinity()
      {
        return std::numeric_limits<double>::infinity();
      }

      /// @param[in] i any valid point index
      ///
      /// @return the path from this point to its closest source, or
      /// an empty path if it cannot be computed (for instance, the
      /// point was not visited yet).
      Path pathToSource( Index i ) const
      {
        Path P;
        if ( ! isVisited( i ) ) return P;
        P.push_back( i );
        while ( ancestor( i ) != i )
          {
            i = ancestor( i );
            P.push_back( i );
          }
        return P;
      }

      /// @return a const reference to the array storing for each
      /// point its ancestor in the shortest path, or itself if it was
      /// a source.
      const std::vector< Index >& ancestors() const
      { return myAncestor; }
      
      /// @return a const reference to the array storing for each
      /// point its distance to the closest source.
      const std::vector< double >& distances() const
      { return myDistance; }
      
      /// @return a const reference to the array storing for each
      /// point if it is already visited.
      const std::vector< bool >& visitedPoints() const
      { return myVisited; }
      
    protected:
      /// A pointer toward the tangency computer.
      const TangencyComputer* myTgcyComputer;
      /// This value is used to prune vertices in the bft. If it is
      /// greater or equal to \f$ \sqrt{d} \f$ where \a d is the
      /// dimension, the shortest path algorithm is guaranteed to
      /// output the correct result. If the value is smaller (down to
      /// 0.0), the algorithm is much faster but a few shortest path
      /// may be missed.
      double                  mySecure;
      /// Stores for each point its ancestor in the shortest path, or
      /// itself if it was a source.
      std::vector< Index >    myAncestor;
      /// Stores for each point its distance to the closest source.
      std::vector< double >   myDistance;
      /// Remembers for each point if it is already visited.
      std::vector< bool >     myVisited;
      /// The queue of points being currently processed.
      std::priority_queue< Node, std::vector< Node >, Comparator > myQ;

    protected:

      /// Updates the queue with the cotangent points of the point given in parameter.
      ///
      /// @param current the index of the point where we determine its
      /// adjacent (here cotangent) to update the queue of the bft.
      void propagate( Index current );
      
      /// Extracts a subset of cotangent points by a breadth-first
      /// traversal. Used to update the queue when computing shortest paths.
      ///
      /// @param[in] i the index of a point
      ///
      /// @return the indices of the other points of the shape that are cotangent to \a a.
      std::vector< Index >
      getCotangentPoints( Index i ) const;

    };

    /// ShortestPaths may access to datas of TangencyComputer.
    friend struct ShortestPaths;
    
    // ------------------------- Standard services --------------------------------
  public:
    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /// Constructor. The object is invalid.
    TangencyComputer() = default;

    /// Copy constructor.
    /// @param other the object to clone.
    TangencyComputer( const Self& other ) = default;

    /// Move constructor.
    /// @param other the object to move
    TangencyComputer( Self&& other ) = default;

    /// Assigment
    /// @param other the object to clone
    /// @return a reference to 'this'
    Self& operator=( const Self& other ) = default;

    /// Move assigment
    /// @param other the object to clone
    /// @return a reference to 'this'
    Self& operator=( Self&& other ) = default;
    
    /// Constructor from digital space.
    /// @param aK the input Khalimsky space, which is cloned.
    TangencyComputer( Clone< KSpace > aK );

    /// Init the object with the points of the range itB, itE
    /// Points within this range are indexed in the same order.
    ///
    /// @tparam PointIterator any model of ForwardIterator on Point.
    /// @param[in] itB an iterator pointing at the beginning of the range.
    /// @param[in] itE an iterator pointing after the end of the range.
    ///
    /// @param[in] use_lattice_cell_cover if 'true' uses
    /// LatticeSetByIntervals to represent the cell geometry instead
    /// of CellGeometry. Generally a little bit slower for digital
    /// surfaces, but may be faster for volumetric objects.
    template < typename PointIterator >
    void init( PointIterator itB, PointIterator itE,
               bool use_lattice_cell_cover = false );

    /// @}

    // ------------------------- Accessors services --------------------------------
  public:
    /// @name Accessors services
    /// @{
    
    /// @return a const reference to the Khalimsky space.
    const KSpace& space() const
    { return myK; }

    /// @return the number of points in the digital set.
    Size size() const
    { return myX.size(); }
    
    /// @return a const reference to the points defining the digital set.
    const std::vector< Point >& points() const
      { return myX; }
      
    /// @param i any valid point index (between 0 included and 'size()' excluded)
    /// @return a const reference to the corresponding point.
    const Point& point( Index i ) const
    { return myX[ i ]; }
      
    /// @param a any point
    /// @return its index or `size()` if the point is not in the object
    /// @note Time complexity is amortized constant.
    Size index( const Point& a ) const
    {
      const auto p = myPt2Index.find( a );
      return p == myPt2Index.cend() ? size() : p->second;
    }
      
    /// @return a const reference to the cell geometry of the current digital set.
    const CellCover& cellCover() const
    { return myCellCover; }

    /// @return a const reference to the lattice cell geometry of the
    /// current digital set.
    const LatticeCellCover& latticeCellCover() const
    { return myLatticeCellCover; }

    /// @param[in] path a sequence of point indices describing a valid path.
    /// @return its Euclidean length.
    double length( const Path& path ) const
    {
      auto eucl_d = [] ( const Point& p, const Point& q )
      { return ( p - q ).norm(); };
      double l = 0.0;
      for ( auto i = 1; i < path.size(); i++ )
        l += eucl_d( point( path[ i-1 ] ), point( path[ i ] ) );
      return l;
    }
    
    /// @}

    // ------------------------- Tangency services --------------------------------
  public:
    /// @name Tangency services
    /// @{
    
    /// Tells if two points are cotangent with respect to the current digital set.
    /// @param[in] a any point
    /// @param[in] b any point
    /// @return 'true' if and only if \a a and \a b are cotangent in this set.
    bool arePointsCotangent( const Point& a, const Point& b ) const;

    /// Tells if three points are cotangent with respect to the current digital set.
    /// @param[in] a any point
    /// @param[in] b any point
    /// @param[in] c any point
    /// @return 'true' if and only if \a a and \a b are cotangent in this set.
    bool arePointsCotangent( const Point& a, const Point& b, const Point& c ) const;

    /// Extracts cotangent points by a breadth-first traversal.
    /// @param[in] a any point
    /// @return the indices of the other points of the shape that are cotangent to \a a.
    std::vector< Index >
    getCotangentPoints( const Point& a ) const;
    
    /// Extracts a subset of cotangent points by a breadth-first traversal.
    ///
    /// @param[in] a any point
    /// @param[in] to_avoid if 'to_avoid[ i ]' is true, then the point
    /// of index \a i is not visited by the bft.
    ///
    /// @return the indices of the other points of the shape that are cotangent to \a a.
    std::vector< Index >
    getCotangentPoints( const Point& a,
                        const std::vector< bool > & to_avoid ) const;
    
    /// @}
    
    // ------------------------- Shortest paths services --------------------------------
  public:
    /// @name Shortest paths services
    /// @{
    
    /// Returns a ShortestPaths object that gives a lot of control
    /// when computing shortest paths. You should use it instead of
    /// TangencyComputer::shortestPaths or
    /// TangencyComputer::shortestPath when (1) you wish to compute
    /// distances to several sources, (2) you wish to store the result
    /// for further use, (4) and more generally if you wish to have
    /// more control on distance computations.
    ///
    /// @param secure This value is used to prune vertices in the
    /// bft. If it is greater or equal to \f$ \sqrt{d} \f$ where \a d
    /// is the dimension, the shortest path algorithm is guaranteed to
    /// output the correct result. If the value is smaller (down to
    /// 0.0), the algorithm is much faster but a few shortest path may
    /// be missed.
    ///
    /// @return a ShortestPaths object that allows shortest path computations.
    ShortestPaths
    makeShortestPaths( double secure = sqrt( KSpace::dimension ) ) const;
        
    /// This function can be used to compute directly several shortest
    /// paths from given sources to a set of targets. Each
    /// returned path starts from the source and ends at the closest
    /// target. The path is empty if there is no path between
    /// them.
    ///
    /// @param[in] sources the indices of the `n` source points.
    /// @param[in] targets the indices of the possible target points.
    ///
    /// @param secure This value is used to prune vertices in the
    /// bft. If it is greater or equal to \f$ \sqrt{d} \f$ where \a d
    /// is the dimension, the shortest path algorithm is guaranteed to
    /// output the correct result. If the value is smaller (down to
    /// 0.0), the algorithm is much faster but a few shortest path may
    /// be missed.
    ///
    /// @param[in] verbose when 'true' some information are displayed
    /// during computation.
    ///
    /// @return the `n` shortest paths from each source point to the
    /// closest target point.
    ///
    /// @note Builds one ShortestPaths object and stops when the bft
    /// is finished.
    std::vector< Path >
    shortestPaths( const std::vector< Index >& sources,
                   const std::vector< Index >& targets,
                   double secure = sqrt( KSpace::dimension ),
                   bool verbose = false ) const;
    
    /// This function can be used to compute directly a shortest path
    /// from a source to a target, returned as a sequence of
    /// point indices, where the first is the source and the last is
    /// the target. It returns an empty sequence if there is no
    /// path between them.
    ///
    /// @param[in] source the index of the source point.
    /// @param[in] target the index of the target point.
    ///
    /// @param secure This value is used to prune vertices in the
    /// bft. If it is greater or equal to \f$ \sqrt{d} \f$ where \a d
    /// is the dimension, the shortest path algorithm is guaranteed to
    /// output the correct result. If the value is smaller (down to
    /// 0.0), the algorithm is much faster but a few shortest path may
    /// be missed.
    ///
    /// @param[in] verbose when 'true' some information are displayed
    /// during computation.
    ///
    /// @return the sequence of point indices from \a source to \a
    /// target, i.e. `[source, ..., target]`, which form a
    /// valid path in the object.
    ///
    /// @note Builds two ShortestPaths objects and stops when they
    /// meet.
    Path
    shortestPath( Index source, Index target,
                  double secure = sqrt( KSpace::dimension ),
                  bool verbose = false ) const;
    
    /// @}
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// The cellular grid space where computations are done.
    KSpace myK;
    /// The digital convexity object used to check full convexity.
    DigitalConvexity< KSpace > myDConv;
    /// The vector of all vectors to neighbors (8 in 2D, 26 in 3D, etc).
    std::vector< Vector > myN;
    /// The vector of all distances to neighbors (8 in 2D, 26 in 3D,
    /// etc), that is the norm of each value of \ref myN.
    std::vector< double > myDN;
    /// The vector of points defining the digital shape under study.
    std::vector< Point >  myX;
    /// Tells if one must use CellCover or LatticeCellCover for computations.
    bool myUseLatticeCellCover;
    /// The cell geometry representing all the cells touching the
    /// digital shape (uses UnorderedSetByBlock behind)
    CellCover myCellCover;
    /// The lattice cell geometry representing all the cells touching
    /// the digital shape (uses LatticeSetByIntervals behind).
    LatticeCellCover myLatticeCellCover;
    
    /// A map giving for each point its index.
    std::unordered_map< Point, Index > myPt2Index;
    
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

    /// Precomputes some neighborhood tables at construction.
    void setUp();
    
  }; // end of class TangencyComputer

  /// @name Functions related to TangencyComputer (output)
  /// @{

  /**
   * Overloads 'operator<<' for displaying objects of class 'TangencyComputer'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'TangencyComputer' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out,
               const TangencyComputer<TKSpace> & object );

  /// @}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "TangencyComputer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TangencyComputer_h

#undef TangencyComputer_RECURSES
#endif // else defined(TangencyComputer_RECURSES)

  
