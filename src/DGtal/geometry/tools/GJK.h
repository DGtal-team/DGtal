/*
 * Copyright (c) 2012   Laurent Provot <provot.research@gmail.com>,
 * Yan Gerard <yan.gerard@free.fr> and Fabien Feschet <research@feschet.fr>
 * All rights reserved.
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GJK_ND_HPP
#define GJK_ND_HPP

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


/**
  \class GJKnD
  \brief An implementation of our extension of the classical GJK algorithm.

  Th GJKnD algorithm allows us to separate 3 sets of integer points with a
  couple of parallel hyperplanes describing a digital hyperplane, with one set
  of point inside the digital hyperplane, and the two other ones lying on their
  own opposite part of the digital hyperplane.
  \code
    typedef GJKnD::Point    Point;
    typedef GJKnD::Vector   Vector;

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
  \endcode
 */
class GJKnD
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
  static bool isDigitalHyperplane(const std::vector<Point> & setIn,
                                  const std::vector<Point> & setAbove,
                                  const std::vector<Point> & setBelow,
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
  static Vector barycentricCoordinatesOrigin(const std::vector<Point> & simplex);

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
  static void closestSimplexToOrigin(const Point & newPoint,
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
  static void findSupportPoint(const Vector & searchDirection,
                               const std::vector<Point> & setIn,
                               const std::vector<Point> & setAbove,
                               const std::vector<Point> & setBelow,
                               Point & result,
                               double & distance);
};

#endif // GJK_ND_HPP
