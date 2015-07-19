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

#include "GJK_nD.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iterator>
#include "Eigen/Core"

/**
  A utility function to compute the barycentric coordinates of the origin
  wrt. to a given k-simplex. The simplex vertices' coordinates are given in
  the canonical basis of Z^d, and we assume k <= d.
  \param simplex the k+1 vertices of a k-simplex embedded in Z^d
  \return the barycentric coordinates of the origin wrt. simplex
 */
GJKnD::Vector
GJKnD::barycentricCoordinatesOrigin(const std::vector<Point> & simplex)
{
  assert(simplex.size() > 1);

  const size_t dim = simplex.size() - 1;

  // we compute some "basis vectors" wrt. simplex
  std::vector<Vector> basisVectors;
  for (size_t i = 0; i < dim; ++i)
    basisVectors.push_back(simplex[i] - simplex[dim]);

  // we project each vertex of the simplex onto each basis vector
  Matrix A(dim + 1, dim + 1);
  for (size_t j = 0; j < dim; ++j)
    for (size_t i = 0; i < dim + 1; ++i)
      A(j,i) = simplex[i].dot(basisVectors[j]);

  for (size_t i = 0; i < dim + 1; ++i)
    A(dim,i) = 1;

  Vector origin(dim + 1);
  for (size_t i = 0; i < dim + 1; ++i)
    origin(i) = 0.0;
  origin(dim) = 1;

  // we solve the linear system
  Eigen::ColPivHouseholderQR<Matrix> decomposition(A);

  if (!decomposition.isInvertible())
    throw SingularSystem();

  return decomposition.solve(origin);
}


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
void
GJKnD::closestSimplexToOrigin(const Point & newPoint,
                              const std::vector<Point> & previousSimplex,
                              std::vector<Point> & closestSimplex,
                              Vector & towardOrigin)
{
  if (previousSimplex.size() == 0) {
    // base case : newPoint is the closest point to the origin
    if (closestSimplex.size() == 0) {
      closestSimplex.push_back(newPoint);
      // update search direction
      towardOrigin = newPoint;
    }
    return;
  }
  else {
    // inductive step
    // test whether the origin is inside [previousSimplex U newPoint]...
    std::vector<Point> currentSimplex = previousSimplex;
    currentSimplex.push_back(newPoint);
    Vector bc = barycentricCoordinatesOrigin(currentSimplex);
    bool allPositive = true;
    for (int i = 0; allPositive && i < bc.size() - 1; ++i)
      allPositive = allPositive && (bc(i) > 0.0);

    if (allPositive) {
      // ... yes it is : we stop the induction
      if (currentSimplex.size() > closestSimplex.size()) {
        closestSimplex = currentSimplex;
        // update search direction
        towardOrigin = bc(0) * currentSimplex[0];
        for (int i = 1; i < bc.size(); ++i)
          towardOrigin += bc(i) * currentSimplex[i];
      }
    }
    else {
      // ... no it's not : we test each sub-simplex newPoint belongs to
      for (int i = 0; i < bc.size() - 1; ++i)
        if (bc(i) <= 0) {
          currentSimplex.clear();
          for (int j = 0; j < static_cast<int>(previousSimplex.size()); ++j)
            if (j != i)
              currentSimplex.push_back(previousSimplex[j]);
          closestSimplexToOrigin(newPoint, currentSimplex,
                                 closestSimplex, towardOrigin);
        }
      // else nothing to do : the origin cannot be both, outside the simplex
      // AND closer to the sub-simplex associated to bc[i] than any other
      // sub-simplices, since b[i] > 0.

    }
  }
}


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
void
GJKnD::findSupportPoint(const Vector & searchDirection,
                        const std::vector<Point> & setIn,
                        const std::vector<Point> & setAbove,
                        const std::vector<Point> & setBelow,
                        Point & result,
                        double & distance)
{
  size_t miniIn, maxiIn, miniAbove, maxiBelow, i;
  double minIn, maxIn, minAbove, maxBelow;

  // look for extreme points in setIn according to searchDirection
  double d = searchDirection.dot(setIn[0]);
  minIn = maxIn = d;
  miniIn = maxiIn = 0;
  for (i = 1; i < setIn.size(); ++i) {
    d = searchDirection.dot(setIn[i]);
    if (d < minIn) {
      minIn = d;
      miniIn = i;
    }
    else if (d > maxIn) {
      maxIn = d;
      maxiIn = i;
    }
  }

  // look for a minimal point in setUp according to searchDirection
  if (!setAbove.empty()) {
    d = searchDirection.dot(setAbove[0]);
    minAbove = d;
    miniAbove = 0;
    for (i = 1; i < setAbove.size(); ++i) {
      d = searchDirection.dot(setAbove[i]);
      if (d < minAbove) {
        minAbove = d;
        miniAbove = i;
      }
    }
  }

  // look for a maximal point in setBelow according to searchDirection
  if (!setBelow.empty()) {
    d = searchDirection.dot(setBelow[0]);
    maxBelow = d;
    maxiBelow = 0;
    for (i = 1; i < setBelow.size(); ++i) {
      d = searchDirection.dot(setBelow[i]);
      if (d > maxBelow) {
        maxBelow = d;
        maxiBelow = i;
      }
    }
  }

  assert(!(setBelow.empty() && setAbove.empty()));

  if (setBelow.empty()) {
    distance = minAbove - maxIn;
    result = setAbove[miniAbove] - setIn[maxiIn];
    return;
  }
  else if (setAbove.empty()) {
    distance = minIn - maxBelow;
    result = setIn[miniIn] - setBelow[maxiBelow];
    return;
  }

  // only keep the smallest difference
  if (minAbove - maxIn < minIn - maxBelow) {
    distance = minAbove - maxIn;
    result = setAbove[miniAbove] - setIn[maxiIn];
    return;
  }
  else {
    distance = minIn - maxBelow;
    result = setIn[miniIn] - setBelow[maxiBelow];
    return;
  }
}



bool GJKnD::isDigitalHyperplane(const std::vector<Point> & setIn,
                                const std::vector<Point> & setAbove,
                                const std::vector<Point> & setBelow,
                                Vector & normal,
                                double & h, double & H)
{
  assert(!setIn.empty());
  assert(!(setAbove.empty() && setBelow.empty()));

  size_t dim = setIn[0].size();
  std::vector<Point> previousSimplex;
  Point supportPoint;
  double distance;

  // initialization (the first point is OK)
  if (setAbove.empty())
    normal = setIn[0] - setBelow[0];
  else
    normal = setAbove[0] - setIn[0];
  previousSimplex.push_back(normal);


  // loop to find the smallest distance between pointsets
  std::vector<Point> newSimplex;
  Vector newDir;
  int nbIter = 0;
  for (;;) {
    findSupportPoint(normal, setIn, setAbove, setBelow, supportPoint, distance);

    if (previousSimplex.size() == dim + 1  ||
        normal.norm() < 1e-12 ||
        Eigen::internal::isApprox(normal.dot(supportPoint),
                                  normal.dot(previousSimplex[0]),
                                  1.0))
      break;

    closestSimplexToOrigin(supportPoint, previousSimplex, newSimplex, newDir);
    normal = newDir;
    previousSimplex = newSimplex;
    newSimplex.clear();

    // check that the iteration is not too long
    if (++nbIter > 1000)
      throw SuspiciousLoop();
  }

  if (previousSimplex.size() == dim + 1 || normal.norm() < 1e-12)
    return false;

  // compute the bounds of the digital hyperplane
  double d = normal.dot(setIn[0]);
  h = d;
  H = d;
  for (size_t i = 1; i < setIn.size(); ++i) {
    d = normal.dot(setIn[i]);
    if (d < h)
      h = d;
    else if (d > H)
      H = d;
  }

  return true;
}
