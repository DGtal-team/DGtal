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
 * @file BoundedLatticePolytope.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/01/02
 *
 * Header file for module BoundedLatticePolytope.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(BoundedLatticePolytope_RECURSES)
#error Recursive header files inclusion detected in BoundedLatticePolytope.h
#else // defined(BoundedLatticePolytope_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BoundedLatticePolytope_RECURSES

#if !defined BoundedLatticePolytope_h
/** Prevents repeated inclusion of headers. */
#define BoundedLatticePolytope_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/arithmetic/ClosedIntegerHalfPlane.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class BoundedLatticePolytope
  /**
     Description of template class 'BoundedLatticePolytope' <p> \brief
     Aim: Represents an nD lattice polytope, i.e. a convex polyhedron
     bounded with vertices with integer coordinates, as a set of
     inequalities. Otherwise said, it is a H-representation of a
     polytope (as an intersection of half-spaces). A limitation is
     that we model only bounded polytopes, i.e. polytopes that can be
     included in a finite bounding box.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TSpace an arbitrary model of CSpace.
   */
  template < typename TSpace >
  class BoundedLatticePolytope
  {
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));

  public:
    typedef BoundedLatticePolytope<TSpace>  Self;
    typedef TSpace                          Space;
    typedef typename Space::Integer         Integer;
    typedef typename Space::Point           Point;
    typedef typename Space::Vector          Vector;
    typedef std::vector<Vector>             InequalityMatrix;
    typedef std::vector<Integer>            InequalityVector;
    typedef HyperRectDomain< Space >        Domain;
    typedef ClosedIntegerHalfPlane< Space > HalfSpace;
    typedef DGtal::BigInteger               BigInteger;
    static const Dimension dimension = Space::dimension;

    /**
     * Represents the unit segment from (0,...,0) (included) to
     * (0,...,1,...,0) (included) with the 1 at position \a k.
     */
    struct UnitSegment {
      Dimension k;
      UnitSegment( Dimension d ) : k( d ) {}
    };

    /**
     * Represents the unit segment from (0,...,0) (excluded) to
     * (0,...,1,...,0) (excluded) with the 1 at position \a k.
     */
    struct StrictUnitSegment {
      Dimension k;
      StrictUnitSegment( Dimension d ) : k( d ) {}
    };

    /**
     * Represents the unit segment from (0,...,0) (included) to
     * (0,...,1,...,0) (excluded) with the 1 at position \a k.
     */
    struct RightStrictUnitSegment {
      Dimension k;
      RightStrictUnitSegment( Dimension d ) : k( d ) {}
    };

    /**
     * Represents the unit segment from (0,...,0) (excluded) to
     * (0,...,1,...,0) (included) with the 1 at position \a k.
     */
    struct LeftStrictUnitSegment {
      Dimension k;
      LeftStrictUnitSegment( Dimension d ) : k( d ) {}
    };

    /**
     * Represents the unit cell obtained by successive Minkowski sum
     * of UnitSegment whose dimensions are stored in \a dims. When \a
     * dims is empty, it is only the point (0,...,0).
     */
    struct UnitCell {
      std::vector<Dimension> dims;
      UnitCell( std::initializer_list<Dimension> l )
        : dims( l.begin(), l.end() ) {}

      /**
      * Overloads 'operator<<' for displaying objects of class 'BoundedLatticePolytope::UnitCell'.
      * @param out the output stream where the object is written.
      * @param object the object of class 'BoundedLatticePolytope::UnitCell' to write.
      * @return the output stream after the writing.
      */
      friend std::ostream&
      operator<< ( std::ostream & out,
                   const UnitCell & object )
      {
        out << "{";
        for ( Dimension i = 0; i < object.dims.size(); ++i ) out << object.dims[ i ];
        out << "}";
        return out;
      }
    };

    /**
     * Represents the unit cell obtained by successive Minkowski sum
     * of RightStrictUnitSegment whose dimensions are stored in \a dims. When \a
     * dims is empty, it is only the point (0,...,0).
     */
    struct RightStrictUnitCell {
      std::vector<Dimension> dims;
      RightStrictUnitCell( std::initializer_list<Dimension> l )
        : dims( l.begin(), l.end() ) {}

      /**
      * Overloads 'operator<<' for displaying objects of class 'BoundedLatticePolytope::UnitCell'.
      * @param out the output stream where the object is written.
      * @param object the object of class 'BoundedLatticePolytope::UnitCell' to write.
      * @return the output stream after the writing.
      */
      friend std::ostream&
      operator<< ( std::ostream & out,
                   const RightStrictUnitCell & object )
      {
        out << "{";
        for ( Dimension i = 0; i < object.dims.size(); ++i ) out << object.dims[ i ];
        out << "}";
        return out;
      }
    };

    /**
     * Represents the unit cell obtained by successive Minkowski sum
     * of RightStrictUnitSegment whose dimensions are stored in \a dims. When \a
     * dims is empty, it is only the point (0,...,0).
     */
    struct LeftStrictUnitCell {
      std::vector<Dimension> dims;
      LeftStrictUnitCell( std::initializer_list<Dimension> l )
        : dims( l.begin(), l.end() ) {}

      /**
      * Overloads 'operator<<' for displaying objects of class 'BoundedLatticePolytope::UnitCell'.
      * @param out the output stream where the object is written.
      * @param object the object of class 'BoundedLatticePolytope::UnitCell' to write.
      * @return the output stream after the writing.
      */
      friend std::ostream&
      operator<< ( std::ostream & out,
                   const LeftStrictUnitCell & object )
      {
        out << "{";
        for ( Dimension i = 0; i < object.dims.size(); ++i ) out << object.dims[ i ];
        out << "}";
        return out;
      }
    };

    /**
     * Represents the unit cell obtained by successive Minkowski sum
     * of StrictUnitSegment whose dimensions are stored in \a dims. When \a
     * dims is empty, it is only the point (0,...,0).
     */
    struct StrictUnitCell {
      std::vector<Dimension> dims;
      StrictUnitCell( std::initializer_list<Dimension> l )
        : dims( l.begin(), l.end() ) {}

      /**
      * Overloads 'operator<<' for displaying objects of class 'BoundedLatticePolytope::UnitCell'.
      * @param out the output stream where the object is written.
      * @param object the object of class 'BoundedLatticePolytope::UnitCell' to write.
      * @return the output stream after the writing.
      */
      friend std::ostream&
      operator<< ( std::ostream & out,
                   const StrictUnitCell & object )
      {
        out << "{";
        for ( Dimension i = 0; i < object.dims.size(); ++i ) out << object.dims[ i ];
        out << "}";
        return out;
      }
    };

    /// @name Standard services (construction, initialization, assignment, interior, closure)
    /// @{

    /**
     * Destructor.
     */
    ~BoundedLatticePolytope() = default;

    /**
     * Constructor.
     */
    BoundedLatticePolytope() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    BoundedLatticePolytope ( const Self & other ) = default;


    /**
     * Constructs the polytope from a simplex given as an initializer_list.
     *
     * @param l any list of no more than d+1 points in general positions.
     *
     */
    BoundedLatticePolytope( std::initializer_list<Point> l );

    /**
     * Constructs the polytope from a simplex given as a range
     * [itB,itE) of lattice points.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param itB the start of the range of no more than n+1 points defining the simplex.
     * @param itE past the end the range of no more than n+1 points defining the simplex.
     */
    template <typename PointIterator>
    BoundedLatticePolytope( PointIterator itB, PointIterator itE );

    /**
     * Constructs a polytope from a domain and a collection of half-spaces.
     *
     * @tparam HalfSpaceIterator any model of forward iterator on HalfSpace.
     * @param domain a bounded lattice domain.
     * @param itB the start of the range of half-spaces.
     * @param itE past the end of the range of half-spaces.
     *
     * @param valid_edge_constraints when 'true', tells that there are
     * half-spaces that represents th constraints on edges (n-2 cells)
     * lying between two faces (n-1 cells) pointing to different
     * orthants.
     *
     * @param check_duplicate_constraints when 'true', the
     * initialization checks if the given range of half-spaces
     * contains axis-aligned half-space constraints already defined by
     * the domain and if so it merges the duplicated constraints,
     * otherwise it accepts and stores the constraints as is.
     */
    template <typename HalfSpaceIterator>
    BoundedLatticePolytope( const Domain& domain,
			    HalfSpaceIterator itB, HalfSpaceIterator itE,
                            bool valid_edge_constraints = false,
                            bool check_duplicate_constraints = false );

    /**
     * Initializes a polytope from a domain and a vector of half-spaces.
     *
     * @tparam HalfSpaceIterator any model of forward iterator on HalfSpace.
     * @param domain a bounded lattice domain.
     * @param itB the start of the range of half-spaces.
     * @param itE past the end of the range of half-spaces.
     *
     * @param valid_edge_constraints when 'true', tells that there are
     * half-spaces that represents the constraints on edges (n-2 cells)
     * lying between two faces (n-1 cells) pointing to different
     * orthants.
     *
     * @param check_duplicate_constraints when 'true', the
     * initialization checks if the given range of half-spaces
     * contains axis-aligned half-space constraints already defined by
     * the domain and if so it merges the duplicated constraints,
     * otherwise it accepts and stores the constraints as is.
     */
    template <typename HalfSpaceIterator>
    void init( const Domain& domain,
	       HalfSpaceIterator itB, HalfSpaceIterator itE,
               bool valid_edge_constraints = false,
               bool check_duplicate_constraints = false );


    /**
     * Initializes the polytope from a simplex given as a range [itB,itE) of points.
     *
     * @param itB the start of the range of no more than n+1 points defining the simplex.
     * @param itE past the end the range of no more than n+1 points defining the simplex.
     *
     * @return 'true' if [itB,itE) was a valid simplex, otherwise
     * return 'false' and the polytope is empty.
     *
     * @note Use DGtal SimpleMatrix::determinant which is not efficient when the
     * dimension is high. Does not use Eigen to avoid dependency.
     */
    template <typename PointIterator>
    bool init( PointIterator itB, PointIterator itE );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// Clears the polytope.
    void clear();

    /// @return the interior (in the topological sense) of this
    /// polytope, by making all constraints strict.
    BoundedLatticePolytope interiorPolytope() const;

    /// @return the closure (in the topological sense) of this
    /// polytope, by making all constraints large.
    BoundedLatticePolytope closurePolytope() const;

    /// @}

    // ----------------------- Accessor services ------------------------------
  public:
    /// @name Accessor services
    /// @{

    /// @return the domain of the current polytope.
    const Domain& getDomain() const;

    /// @return the number of half-space constraints.
    unsigned int nbHalfSpaces() const;

    /// @param i the index of the half-space constraint between 0 and
    /// `nbHalfSpaces()` (excluded).
    ///
    /// @return the normal vector of the \a i-th half space
    /// constraint (i.e. `A` in constraint `Ax <= b`).
    const Vector& getA( unsigned int i ) const;

    /// @param i the index of the half-space constraint between 0 and
    /// `nbHalfSpaces()` (excluded).
    ///
    /// @return the offset of the \a i-th half space
    /// constraint (i.e. `b` in constraint `Ax <= b`).
    Integer getB( unsigned int i ) const;

    /// @param i the index of the half-space constraint between 0 and
    /// `nbHalfSpaces()` (excluded).
    ///
    /// @return 'true' if the \a i-th half space constraint is of the
    /// form `Ax <= b`, 'false' if it is of the form `Ax < b`.
    bool isLarge( unsigned int i ) const;

    /// Sets the \a i-th half space constraint to the form `Ax <= b`.
    ///
    /// @param i the index of the half-space constraint between 0 and
    /// `nbHalfSpaces()` (excluded).
    void setLarge( unsigned int i );

    /// Sets the \a i-th half space constraint to the form `Ax < b`.
    ///
    /// @param i the index of the half-space constraint between 0 and
    /// `nbHalfSpaces()` (excluded).
    void setStrict( unsigned int i );

    /// @return the matrix A in the polytope representation \f$ Ax \le B \f$.
    const InequalityMatrix& getA() const;

    /// @return the vector B in the polytope representation \f$ Ax \le B \f$.
    const InequalityVector& getB() const;
    /// @return the vector I telling if inequalities are large in the
    /// polytope representation \f$ Ax \prec B \f$, with \f$ \prec \in
    /// \{ <, \le \} \f$.
    const std::vector<bool>& getI() const;

    /// @return 'true' if we can perform exact Minkowksi sums on this
    /// polytope. This is related to the presence of valid edge
    /// constraints (n-k cells for k >= 2) in-between face constraints
    /// (n-1 cells) that change orthants.
    bool canBeSummed() const;

    /// @}

    // ----------------------- Check point services ------------------------------
  public:

    /// @name Check point services (is inside test)
    /// @{

    /**
     * @param p any point of the space.
     * @return 'true' if and only if \a p is inside this polytope.
     */
    bool isInside( const Point& p ) const;

    /**
     * @param p any point inside the domain of this polytope.
     * @return 'true' if and only if \a p is inside this polytope.
     *
     * @note Slightly faster than isInside.
     */
    bool isDomainPointInside( const Point& p ) const;

    /**
     * @param p any point of the space.
     * @return 'true' if and only if \a p is strictly inside this polytope.
     */
    bool isInterior( const Point& p ) const;

    /**
     * @param p any point of the space.
     * @return 'true' if and only if \a p lies on the boundary of this polytope.
     */
    bool isBoundary( const Point& p ) const;

    /// @}

    // ----------------------- Modification services ------------------------------
  public:

    /// @name Global modification services (cut, swap, Minkowski sum)
    /// @{


    /**
       Cut the polytope by the given half space `a.x <= b` or `a.x <
       b` where `a` is some axis vector.

       @param k the dimension of the axis vector \f$ +/- e_k \f$
       @param pos 'true' is positive, 'false' is negative for the axis vector \f$ +/- e_k \f$
       @param b any integer number
       @param large tells if the inequality is large (true) or strict (false).

       @return the index of the constraint in the polytope.

    */
    unsigned int cut( Dimension k, bool pos, Integer b, bool large = true );

    /**
       Cut the polytope by the given half space `a.x <= b` or `a.x < b`.

       @param a any integer vector
       @param b any integer number
       @param large tells if the inequality is large (true) or strict (false).
       @param valid_edge_constraint when 'true', tells that the
       half-spaces that represents constraints on edges (n-2 cells)
       lying between two faces (n-1 cells) pointing to different
       orthants are still valid after this operation.

       @return the index of the constraint in the polytope.

       @note For now complexity is O(n) where n=A.rows() because it
       checks if a parallel closed half-space defines already a face
       of the polytope.
    */
    unsigned int cut( const Vector& a, Integer b, bool large = true,
		      bool valid_edge_constraint = false );

    /**
       Cuts the lattice polytope with the given half-space constraint.

       @param hs any half-space constraint.
       @param large tells if the inequality is large (true) or strict (false).
       @param valid_edge_constraint when 'true', tells that the
       half-spaces that represents constraints on edges (n-2 cells)
       lying between two faces (n-1 cells) pointing to different
       orthants are still valid after this operation.

       @return the index of the constraint in the polytope.

       @note For now complexity is O(n) where n=A.rows() because it
       checks if a parallel closed half-space defines already a face
       of the polytope.
     */
    unsigned int cut( const HalfSpace & hs, bool large = true,
		      bool valid_edge_constraint = false );

    /**
       Swaps the content of this object with other. O(1) complexity.
       @param other any other BoundedLatticePolytope.
    */
    void swap( BoundedLatticePolytope & other );


    /**
     * Dilates this polytope P by t.
     * @param t any integer.
     * @return a reference to 'this', which is now the polytope tP.
     */
    Self& operator*=( Integer t );

    /**
     * Minkowski sum of this polytope with a unit segment aligned with some axis.
     *
     * @param s any unit segment.
     * @return a reference to 'this'.
     */
    Self& operator+=( UnitSegment s );

    /**
     * Minkowski sum of this polytope with an axis-aligned unit cell.
     *
     * @param c any unit cell.
     * @return a reference to 'this'.
     */
    Self& operator+=( UnitCell c );

    /**
     * Minkowski sum of this polytope with a strict unit segment aligned with some axis.
     *
     * @param s any strict unit segment.
     * @return a reference to 'this'.
     */
    Self& operator+=( StrictUnitSegment s );

    /**
     * Minkowski sum of this polytope with an axis-aligned strict unit cell.
     *
     * @param c any unit cell.
     * @return a reference to 'this'.
     */
    Self& operator+=( StrictUnitCell c );

    /**
     * Minkowski sum of this polytope with a right strict unit segment aligned with some axis.
     *
     * @param s any strict unit segment.
     * @return a reference to 'this'.
     */
    Self& operator+=( RightStrictUnitSegment s );

    /**
     * Minkowski sum of this polytope with an axis-aligned right strict unit cell.
     *
     * @param c any strict unit cell.
     * @return a reference to 'this'.
     */
    Self& operator+=( RightStrictUnitCell c );

    /**
     * Minkowski sum of this polytope with a left strict unit segment aligned with some axis.
     *
     * @param s any strict unit segment.
     * @return a reference to 'this'.
     */
    Self& operator+=( LeftStrictUnitSegment s );

    /**
     * Minkowski sum of this polytope with an axis-aligned left strict unit cell.
     *
     * @param c any strict unit cell.
     * @return a reference to 'this'.
     */
    Self& operator+=( LeftStrictUnitCell c );
    /// @}

    // ----------------------- Enumeration services ------------------------------
  public:

    /// @name Enumeration services (counting, get points in polytope)
    /// @{

    /**
     * Computes the number of integer points lying within the polytope.
     *
     * @return the number of integer points lying within the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     */
    Integer count() const;

    /**
     * Computes the number of integer points lying within the interior of the polytope.
     *
     * @return the number of integer points lying within the interior of the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     *
     * @note `count() <= countInterior() + countBoundary()` with
     * equality when the polytope is closed.
     */
    Integer countInterior() const;

    /**
     * Computes the number of integer points lying on the boundary of the polytope.
     *
     * @return the number of integer points lying on the boundary of the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     *
     * @note `count() <= countInterior() + countBoundary()` with
     * equality when the polytope is closed.
     */
    Integer countBoundary() const;

    /**
     * Computes the number of integer points within the polytope and
     * the domain bounded by \a low and \a hi.
     *
     * @param[in] low the lowest point of the domain.
     * @param[in] hi the highest point of the domain.
     * @return the number of integer points within the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     */
    Integer countWithin( Point low, Point hi ) const;

    /**
     * Computes the number of integer points within the polytope up to
     * some maximum number \a max.
     *
     * @note For instance, a d-dimensional simplex that contains no
     * integer points in its interior contains only d+1 points. If
     * there is more, you know that the simplex has a non empty
     * interior.
     *
     * @param[in] max the maximum number of points that are counted,
     * the method exists when this number of reached.
     *
     * @return the number of integer points within the polytope up to .
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     */
    Integer countUpTo( Integer max ) const;

    /**
     * Computes the integer points within the polytope.
     *
     * @param[out] pts the integer points within the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     * @note At output, pts.size() == this->count()
     */
    void getPoints( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points within the polytope and converts
     * them to cells represented with their Khalimsky coordinates.
     *
     * @param[out] pts the integer points within the polytope, with
     * coordinates equal to `2*p - alpha_shift`, for `p` in the polytope.
     *
     * @param[in] alpha_shift the translation applied to each point.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     * @note At output, pts.size() == this->count()
     * @note Useful for full convexity checks.
     */
    void getKPoints( std::vector<Point>& pts, const Point& alpha_shift ) const;

    /**
     * Computes the integer points interior to the polytope.
     *
     * @param[out] pts the integer points interior to the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     * @note At output, pts.size() == this->countInterior()
     */
    void getInteriorPoints( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points boundary to the polytope.
     *
     * @param[out] pts the integer points boundary to the polytope.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     * @note At output, pts.size() == this->countBoundary()
     */
    void getBoundaryPoints( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points within the polytope.
     *
     * @tparam PointSet any model of set with a method `insert( Point )`.
     *
     * @param[in,out] pts_set the set of points where points within
     * this polytope are inserted.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     */
    template <typename PointSet>
    void insertPoints( PointSet& pts_set ) const;

    /**
     * Computes the integer points within the polytope and converts
     * them to cells represented with their Khalimsky coordinates.
     *
     * @tparam PointSet any model of set with a method `insert( Point )`.
     *
     * @param[in,out] pts_set the set of points where points within
     * this polytope are inserted. Each point is inserted with
     * coordinates equal to `2*p - alpha_shift`, for `p` in the
     * polytope.
     *
     * @param[in] alpha_shift the translation applied to each point.
     *
     * @note Quite fast: obtained by line intersection, see
     * BoundedLatticePolytopeCounter
     */
    template <typename PointSet>
    void insertKPoints( PointSet& pts_set, const Point& alpha_shift ) const;

    /// @}

    // -------------- Enumeration services (old methods by scanning ) --------------
  public:

    /// @name Enumeration services by scanning (counting, get points in polytope)
    /// @{

    /**
     * Computes the number of integer points lying within the polytope.
     *
     * @return the number of integer points lying within the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     */
    Integer countByScanning() const;

    /**
     * Computes the number of integer points lying within the interior of the polytope.
     *
     * @return the number of integer points lying within the interior of the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     *
     * @note `count() <= countInterior() + countBoundary()` with
     * equality when the polytope is closed.
     */
    Integer countInteriorByScanning() const;

    /**
     * Computes the number of integer points lying on the boundary of the polytope.
     *
     * @return the number of integer points lying on the boundary of the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     *
     * @note `count() <= countInterior() + countBoundary()` with
     * equality when the polytope is closed.
     */
    Integer countBoundaryByScanning() const;

    /**
     * Computes the number of integer points within the polytope and
     * the domain bounded by \a low and \a hi.
     *
     * @param[in] low the lowest point of the domain.
     * @param[in] hi the highest point of the domain.
     * @return the number of integer points within the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     */
    Integer countWithinByScanning( Point low, Point hi ) const;

    /**
     * Computes the number of integer points within the polytope up to
     * some maximum number \a max.
     *
     * @note For instance, a d-dimensional simplex that contains no
     * integer points in its interior contains only d+1 points. If
     * there is more, you know that the simplex has a non empty
     * interior.
     *
     * @param[in] max the maximum number of points that are counted,
     * the method exists when this number of reached.
     *
     * @return the number of integer points within the polytope up to .
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     */
    Integer countUpToByScanning( Integer max ) const;

    /**
     * Computes the integer points within the polytope.
     *
     * @param[out] pts the integer points within the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     * @note At output, pts.size() == this->count()
     */
    void getPointsByScanning( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points interior to the polytope.
     *
     * @param[out] pts the integer points interior to the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     * @note At output, pts.size() == this->countInterior()
     */
    void getInteriorPointsByScanning( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points boundary to the polytope.
     *
     * @param[out] pts the integer points boundary to the polytope.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     * @note At output, pts.size() == this->countBoundary()
     */
    void getBoundaryPointsByScanning( std::vector<Point>& pts ) const;

    /**
     * Computes the integer points within the polytope.
     *
     * @tparam PointSet any model of set with a method `insert( Point )`.
     *
     * @param[in,out] pts_set the set of points where points within
     * this polytope are inserted.
     *
     * @note Quite slow: obtained by checking every point of the polytope domain.
     */
    template <typename PointSet>
    void insertPointsByScanning( PointSet& pts_set ) const;

    /// @}


    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface services
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object. If the polytope
     * has been default constructed, it is invalid.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * @return the class name. It is notably used for drawing this object.
     */
    std::string className() const;

    /// @}

    // ------------------------- Protected Data ------------------------------
  protected:
    /// The matrix A in the polytope representation \f$ Ax \le B \f$.
    InequalityMatrix  A;
    /// The vector B in the polytope representation \f$ Ax \le B \f$.
    InequalityVector  B;
    /// Tight bounded box
    Domain            D;
    /// Are inequalities large ?
    std::vector<bool> I;
    /// Indicates if Minkowski sums with segments will be valid
    bool myValidEdgeConstraints;

    // ------------------------- Private Data --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:
    /// In 3D, builds a valid lattice polytope with empty interior
    /// from 3 non-colinear points.
    /// @param a any point such that a, b, and c are not colinear.
    /// @param b any point such that a, b, and c are not colinear.
    /// @param c any point such that a, b, and c are not colinear.
    /// @return 'true' if they were not colinear, false otherwise.
    bool internalInitFromTriangle3D( Point a, Point b, Point c );

    /// In 3D, builds a valid lattice polytope with empty interior
    /// from 2 points.
    /// @param a any point
    /// @param b any point
    /// @return 'true'
    bool internalInitFromSegment3D( Point a, Point b );

    /// In 2D, builds a valid lattice polytope with empty interior
    /// from 2 points.
    /// @param a any point
    /// @param b any point
    /// @return 'true'
    bool internalInitFromSegment2D( Point a, Point b );

  }; // end of class BoundedLatticePolytope

  namespace detail {
  /**
     Description of template class 'BoundedLatticePolytopeSpecializer'
     <p> \brief Aim: It is just a helper class for
     BoundedLatticePolytope to add dimension specific static methods.

     @tparam N the dimension of the polytope.
     @tparam TInteger any model of integer.
   */
    template <DGtal::Dimension N, typename TInteger>
    struct BoundedLatticePolytopeSpecializer {
      typedef TInteger                        Integer;
      typedef SpaceND< N, Integer>            Space;
      typedef typename Space::Point           Point;
      typedef typename Space::Vector          Vector;
      typedef BoundedLatticePolytope< Space > Polytope;
      static const Dimension dimension = Space::dimension;

      /// Not implemented generic method.
      ///
      /// This method add extremal constraints for simplex edges, faces, etc. Each
      /// constraint is a half-space bounded by the edge, face, etc, and one
      /// axis. Such constraints are useful when computing the
      /// Minkowski sum.
      ///
      /// @note This method is useful starting from 3D, where it is
      /// implemented in the specialization of this class.
      ///
      /// @todo For higher dimensions, one should add constraint for
      /// extremal faces, etc.
      static void
      addEdgeConstraint( Polytope& , unsigned int , unsigned int ,
			 const std::vector<Point>& )
      {
	trace.error() << "[BoundedLatticePolytopeHelper::addEdgeConstraint]"
		      << " this method is only implemented in 3D." << std::endl;
      }

      /// Generic method for cross product, only implemented in 3D.
      /// @return their cross product.
      static
      Vector crossProduct( const Vector& , const Vector& )
      {
	trace.error() << "[BoundedLatticePolytopeHelper::crossProduct]"
		      << " this method is only implemented in 3D." << std::endl;
	return Vector::zero;
      }
    };

    /**
       Description of template class 'BoundedLatticePolytopeSpecializer'
       <p> \brief Aim: 3D specialization for
       BoundedLatticePolytope to add dimension specific static methods.

       @tparam TInteger any model of integer.
    */
    template <typename TInteger>
    struct BoundedLatticePolytopeSpecializer<3, TInteger> {
      typedef TInteger                        Integer;
      typedef SpaceND< 3, Integer>            Space;
      typedef typename Space::Point           Point;
      typedef typename Space::Vector          Vector;
      typedef BoundedLatticePolytope< Space > Polytope;
      static const Dimension dimension = Space::dimension;

      /// This method add extremal constraints for simplex edges. Each
      /// constraint is a half-space bounded by the edge and one
      /// axis. Such constraints are useful when computing the
      /// Minkowski sum.
      ///
      /// @param[in,out] P any polytope.
      /// @param[in] i any index in the vector of points \a pts.
      /// @param[in] j any index in the vector of points \a pts.
      /// @param[in] pts a vector of points defining a simplex.
      static void
      addEdgeConstraint( Polytope& P, unsigned int i, unsigned int j,
			 const std::vector<Point>& pts )
      {
	Vector ab = pts[ i ] - pts[ j ];
	for ( int s = 0; s < 2; s++ )
	  for ( Dimension k = 0; k < dimension; ++k )
	    {
	      Vector   n = ab.crossProduct( Point::base( k, (s == 0) ? 1 : -1 ) );
	      Integer  b = n.dot( pts[ i ] );
              std::size_t nb_in = 0;
	      for ( auto p : pts ) {
		Integer v = n.dot( p );
		if ( v < b )  nb_in++;
	      }
	      if ( nb_in == pts.size() - 2 ) {
		P.cut( n, b, true, true );
	      }
	    }
      }
      /// Generic method for cross product, only implemented in 3D.
      /// @param v1 any vector
      /// @param v2 any vector
      /// @return their cross product.
      static
      Vector crossProduct( const Vector& v1, const Vector& v2 )
      {
	return v1.crossProduct( v2 );
      }
    };
  }

  /// @name Functions related to BoundedLatticePolytope (output, dilation, Minkowski sum)
  /// @{

  /**
   * Overloads 'operator<<' for displaying objects of class 'BoundedLatticePolytope'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'BoundedLatticePolytope' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out,
               const BoundedLatticePolytope<TSpace> & object );


  /**
   * @param t any integer.
   * @param P any polytope.
   * @return the polytope tP.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator* ( typename BoundedLatticePolytope<TSpace>::Integer t,
              const BoundedLatticePolytope<TSpace> & P );


  /**
   * Minkowski sum of polytope \a P with unit segment \a s aligned with some axis.
   *
   * @param P any polytope.
   * @param s any unit segment.
   * @return the Polytope P + s.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::UnitSegment s );

  /**
   * Minkowski sum of polytope \a P with an axis-aligned unit cell \a c.
   *
   * @param P any polytope.
   * @param c any unit cell.
   * @return the Polytope P + c.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::UnitCell c );

  /**
   * Minkowski sum of polytope \a P with strict unit segment \a s aligned with some axis.
   *
   * @param P any polytope.
   * @param s any strict unit segment.
   * @return the Polytope P + s.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::StrictUnitSegment s );

  /**
   * Minkowski sum of polytope \a P with an axis-aligned strict unit cell \a c.
   *
   * @param P any polytope.
   * @param c any strict unit cell.
   * @return the Polytope P + c.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::StrictUnitCell c );

  /**
   * Minkowski sum of polytope \a P with strict unit segment \a s aligned with some axis.
   *
   * @param P any polytope.
   * @param s any strict unit segment.
   * @return the Polytope P + s.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::RightStrictUnitSegment s );

  /**
   * Minkowski sum of polytope \a P with an axis-aligned strict unit cell \a c.
   *
   * @param P any polytope.
   * @param c any strict unit cell.
   * @return the Polytope P + c.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::RightStrictUnitCell c );

  /**
   * Minkowski sum of polytope \a P with strict unit segment \a s aligned with some axis.
   *
   * @param P any polytope.
   * @param s any strict unit segment.
   * @return the Polytope P + s.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::LeftStrictUnitSegment s );

  /**
   * Minkowski sum of polytope \a P with an axis-aligned strict unit cell \a c.
   *
   * @param P any polytope.
   * @param c any strict unit cell.
   * @return the Polytope P + c.
   */
  template <typename TSpace>
  BoundedLatticePolytope<TSpace>
  operator+ ( const BoundedLatticePolytope<TSpace> & P,
              typename BoundedLatticePolytope<TSpace>::LeftStrictUnitCell c );

  /// @}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "BoundedLatticePolytope.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BoundedLatticePolytope_h

#undef BoundedLatticePolytope_RECURSES
#endif // else defined(BoundedLatticePolytope_RECURSES)
