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
 * @file GJK.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Laurent Provot <provot.research@gmail.com>
 * @authoer Yan Gerard <yan.gerard@free.fr>
 * @author Fabien Feschet <research@feschet.fr>
 * All rights reserved.
 *
 * @date 2014/07/02
 *
 * Header file for module GJK.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GJK_RECURSES)
#error Recursive header files inclusion detected in GJK.h
#else // defined(GJK_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GJK_RECURSES

#if !defined GJK_h
/** Prevents repeated inclusion of headers. */
#define GJK_h

#include <vector>
#include <stdexcept>
#include "Eigen/Dense"



/**
 \exception SingularSystem
 Raised during the recognition process when a linear system is singular.
 */
class SingularSystem : public std::runtime_error
{
public:
  SingularSystem()
  : std::runtime_error("The system's matrix is not invertible.")
  {}
};


/**
 \exception SuspiciousLoop
 Raised during the recognition process when a loop seems suspicious (too long).
 Due to double type, rounding errors may occur and the halting condition may
 not be reach.
 */
class SuspiciousLoop : public std::runtime_error
{
public:
  SuspiciousLoop()
  : std::runtime_error("The convergence loop for this instance seems "
                       "suspicious (too many iterations).")
  {}
};

namespace DGtal
{
  /**
   @class GJK
   @brief An implementation of our extension of the classical GJK algorithm.
   
   Th GJKnD algorithm allows us to separate 3 sets of integer points with a
   couple of parallel hyperplanes describing a digital hyperplane, with one set
   of point inside the digital hyperplane, and the two other ones lying on their
   own opposite part of the digital hyperplane.
   @code
   typedef GJK::Point    Point;
   typedef GJK::Vector   Vector;
   
   std::vector<Point> setIn, setUp, setDown;
   // initialize the 3 pointsets with your nD data
   ...
   
   Vector orthoDir;
   double lower_bound, upper_bound;
   bool dhpOK;
   try {
   dhpOK = isDigitalHyperplane(setIn, setUp, setDown,
   orthoDir, lower_bound, upper_bound);
   }
   catch (const SingularSystem & e) {
   std::cout << "Robustness problem: " << e.what()
   << " Unable to decide whether the sets are separable."
   << std::endl;
   return;
   }
   
   if (dhpOK)
   std::cout << "The sets describe a digital hyperplane: "
   << lower_bound << " < " << orthoDir.transpose() << " . X < "
   << upper_bound << std::endl;
   else
   std::cout << "The sets ares not separable by a pair of parallel "
   << "hyperplanes" << std::endl;
   @endcode
   
   *
   */
  class GJK
  {
  public:
    
    /// The vector type used in the GJKnD algorithm.
    typedef Eigen::VectorXd   Vector;
    /// The point type used in the GJKnD algorithm.
    typedef Vector            Point;
    
  private:
    /// The underlying matrix type used in the GJKnD algorithm.
    typedef Eigen::MatrixXd   Matrix;
    
  public:
    /**
     The isDigitalHyperplane function looks for a couple of hyperplanes that
     separtes the 3 sets of input points.
     \param[in]  setIn    the set of points that shall lie between the couple of hyperplanes
     \param[in]  setAbove the set of points that shall lie above the upper hyperplane
     \param[in]  setBelow the set of points that shall lie below the lower hyperplane
     \param[out] normal   the normal vector of the separting hyperplanes
     \param[out] h        the shift parameter of the lower hyperplanes
     \param[out] H        the shift parameter of the upper hyperplanes
     \return true if the couple of hyperplanes exists, false otherwise
     
     \warning Due to the underlying \a double type, robustness problems may occur
     and this function may throw a SingularSystem exception. The user should
     catch this exception and be aware that the fact the system is singular does
     not mean the sets are not separable.
     */
    template <typename ConstIteratorOnPoints>
    bool isDigitalHyperplane(ConstIteratorOnPoints & setInBegin,
                             ConstIteratorOnPoints & setInEnd,
                             ConstIteratorOnPoints & setAboveBegin,
                             ConstIteratorOnPoints & setAboveEnd,
                             ConstIteratorOnPoints & setBelowBegin,
                             ConstIteratorOnPoints & setBelowEnd,
                             Vector & normal,
                             double & h, double & H);
    
  private:
    /**
     A utility function to compute the barycentric coordinates of the origin
     wrt. to a given k-simplex. The simplex vertices' coordinates are given in
     the canonical basis of Z^d, and we assume k <= d.
     \param simplex the k+1 vertices of a k-simplex embedded in Z^d
     \return the barycentric coordinates of the origin wrt. simplex
     */
    Vector barycentricCoordinatesOrigin(const std::vector<Point> & simplex);
    
    /**
     An inductive function to compute the closest k-simplex to the origin in Z^d (k <= d)
     \param[in] newPoint        the furthest point of the set according to
     our search direction
     \param[in] previousSimplex the previous closest k-simplex (k<d) to the origin.
     The function is incremental and we assume that the
     origin stricly lies in the positive half-space
     delimited by previousSimplex
     \param[out] closestSimplex the result i.e. the closest simplex to the origin
     \param[out] towardOrigin   the new search direction to find points
     */
    void closestSimplexToOrigin(const Point & newPoint,
                                const std::vector<Point> & previousSimplex,
                                std::vector<Point> & closestSimplex,
                                Point & towardOrigin);
    
    /**
     This function looks for extreme points (support points) in the different
     pointsets according to a given direction.
     \param[in] searchDirection well... the search direction
     \param[in] setIn           the set of points that belong to the (potential)
     hyperplan. We assume that the set is not empty.
     \param[in] setAbove        the set of points (that don't belong to the
     (potential) hyperplan) lying above setIn
     \param[in] setBelow        the set of points (that don't belong to the
     (potential) hyperplan) lying below setIn
     \param[out] result         the furthest point wrt. searchDirection
     \param[out] distance       the distance to the origin
     */
    template <typename ConstIteratorOnPoints>
    void findSupportPoint(const Vector & searchDirection,
                          ConstIteratorOnPoints & setInBegin,
                          ConstIteratorOnPoints & setInEnd,
                          ConstIteratorOnPoints & setAboveBegin,
                          ConstIteratorOnPoints & setAboveEnd,
                          ConstIteratorOnPoints & setBelowBegin,
                          ConstIteratorOnPoints & setBelowEnd,
                          Point & result,
                          double & distance);
  };
  
  
  /**
   * Overloads 'operator<<' for displaying objects of class 'GJK'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GJK' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const GJK & object );
  
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////////////////////////////////
  // Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/tools/GJK.ih"
#endif
  
#endif // !defined GJK_h
  
#undef GJK_RECURSES
#endif // else defined(GJK_RECURSES)
  
  
} //end namespace
