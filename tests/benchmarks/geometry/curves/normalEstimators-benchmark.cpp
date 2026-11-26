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

/**
 * @file geometry/curves/normalEstimators-benchmark.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Benchmark of the multigrid convergence and of the running times of
 * normal estimators based on maximal segments, and of the length and
 * curvature estimators derived from them.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/parametric/Ellipse2D.h"
#include "DGtal/shapes/parametric/Flower2D.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputerOnArrows.h"
#include "DGtal/geometry/curves/StabbingLineComputer.h"
#include "DGtal/geometry/curves/StabbingCircleComputer.h"
#include "DGtal/geometry/curves/estimation/SegmentComputerEstimators.h"
#include "DGtal/geometry/curves/estimation/MostCenteredMaximalSegmentEstimator.h"
#include "DGtal/geometry/curves/estimation/LambdaMaximalSegmentEstimator.h"

/**
   This benchmark studies the multigrid convergence and the running
   times of normal estimators based on maximal segments along the boundary
   of digitized shapes: a convex shape (an ellipse) and a non-convex
   shape (a flower). Four kinds of maximal segments are compared:
   - maximal DSSs (ArithmeticalDSSComputer), on the points of the curve;
   - maximal DSSs (ArithmeticalDSSComputerOnArrows), on the steps of the
     curve (arrows);
   - maximal segments of StabbingLineComputer, on the steps of the curve
     (pairs of inner and outer points);
   - maximal segments of StabbingCircleComputer (DCA), on the steps of the
     curve.
   For each kind of maximal segments, two estimators are compared:
   - the most centered maximal segment estimator
     (MostCenteredMaximalSegmentEstimator);
   - the lambda-maximal segment estimator (LambdaMaximalSegmentEstimator,
     with the lambda function functors::Lambda64Function and the
     eccentricity functors::PointEccentricity for the points or
     functors::StepEccentricity for the steps; see also
     \ref modulePrimitiveBasedEstimators).

\verbatim
normalEstimators-benchmark
\endverbatim

   For each shape and each grid step h, the shape is digitized with a
   Gauss digitization, its boundary is tracked and the normal vectors are
   estimated at each element of the closed curve. The estimated normals
   are compared with the true normals of the shape at the points obtained
   by radial projection of the elements (points or middles of the steps)
   onto the boundary of the shape, the error being the angle between the
   estimated and the true normal directions (in radian, regardless of the
   orientation of the normals).

   The length of the curve is moreover estimated by integration of each of
   these normal estimators: each step of the curve, of length h,
   contributes \f$ h |n \cdot u| \f$, where \f$ n \f$ is the estimated
   unit normal vector at the step (the normalized sum of the normal vectors
   at its two points for the estimators on points) and \f$ u \f$ the
   trivial normal vector of the step. The estimated length is compared
   with the length of the boundary of the shape (relative error).

   Finally, the curvature is estimated with CurvatureFromDCAEstimator, i.e.
   from the maximal segments of StabbingCircleComputer, with both
   estimators, and compared with the true curvature of the shape at the
   middles of the steps (absolute error).

   The results are written on the standard output and in the files:
   - "normals-convergence-[shape].txt", each line containing the grid step,
     the number of points of the curve, then, for each normal estimator,
     the mean error, the maximal error and the running time (ms);
   - "length-convergence-[shape].txt", each line containing the grid step,
     the number of points of the curve, the true length, then, for each
     normal estimator, the relative error of the estimated length;
   - "curvature-convergence-[shape].txt", each line containing the grid
     step, the number of points of the curve, then, for each curvature
     estimator, the mean error, the maximal error and the running time (ms).

   \image html normals-convergence-ellipse.png "Mean angular error (radian) of the normal estimators along the boundary of a digitized ellipse as a function of the grid step h."
   \image html normals-max-error-ellipse.png "Maximal angular error (radian) of the normal estimators along the boundary of a digitized ellipse as a function of the grid step h."
   \image html normals-convergence-flower.png "Mean angular error (radian) of the normal estimators along the boundary of a digitized flower as a function of the grid step h."
   \image html normals-max-error-flower.png "Maximal angular error (radian) of the normal estimators along the boundary of a digitized flower as a function of the grid step h."
   \image html normals-timings.png "Running times (ms) of the normal estimators along the boundary of a digitized flower as a function of the number of points."
   \image html length-convergence-ellipse.png "Relative error of the length of the boundary of a digitized ellipse, estimated by integration of the normal vectors, as a function of the grid step h."
   \image html length-convergence-flower.png "Relative error of the length of the boundary of a digitized flower, estimated by integration of the normal vectors, as a function of the grid step h."
   \image html curvature-convergence-ellipse.png "Mean and maximal absolute errors of the curvature estimated from DCAs along the boundary of a digitized ellipse as a function of the grid step h."
   \image html curvature-convergence-flower.png "Mean and maximal absolute errors of the curvature estimated from DCAs along the boundary of a digitized flower as a function of the grid step h."

   \example benchmarks/geometry/curves/normalEstimators-benchmark.cpp
*/

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Normal vectors from maximal segments
///////////////////////////////////////////////////////////////////////////////

/**
 * Unit normal vector of a DSS of characteristics (a, b, mu),
 * i.e. the unit vector of direction (a, -b).
 */
struct NormalFromDSS
{
  typedef RealVector Value;
  template <typename DSS>
  Value operator() (const DSS& aDSS) const
  {
    Value v( double( aDSS.a() ), -double( aDSS.b() ) );
    return v / v.norm();
  }
};

/**
 * Unit normal vector of the straight line alpha.x + beta.y + gamma = 0
 * separating the inner and outer points of a StabbingLineComputer,
 * i.e. the unit vector of direction (alpha, beta).
 */
struct NormalFromStabbingLine
{
  typedef RealVector Value;
  template <typename StabbingLine>
  Value operator() (const StabbingLine& aSegment) const
  {
    double alpha, beta, gamma;
    aSegment.getParameters( alpha, beta, gamma );
    Value v( alpha, beta );
    return v / v.norm();
  }
};

template <typename DSSComputer>
using NormalFromDSSEstimator =
  DGtal::detail::PosIndepScaleIndepSCEstimator<DSSComputer, NormalFromDSS>;

template <typename StabbingLineComputer>
using NormalFromStabbingLineEstimator =
  DGtal::detail::PosIndepScaleIndepSCEstimator<StabbingLineComputer, NormalFromStabbingLine>;

///////////////////////////////////////////////////////////////////////////////
// Measures
///////////////////////////////////////////////////////////////////////////////

/// Errors and running time of an estimator
struct Measure
{
  double meanError;
  double maxError;
  double time; //ms
};

/// Angle (in [0, pi/2]) between the directions of two vectors
double angle(RealVector u, RealVector v)
{
  double c = std::abs( u.dot(v) ) / ( u.norm() * v.norm() );
  return std::acos( std::min( 1.0, c ) );
}

/**
 * Estimates a quantity along a whole closed curve.
 *
 * @param estimator an estimator (model of CCurveLocalGeometricEstimator)
 * @param c a circulator on the elements of the curve
 * @param h the grid step
 * @param values (returns) the estimated quantities
 * @return the running time (ms)
 */
template <typename Estimator, typename Circulator, typename Quantity>
double estimate(Estimator& estimator, const Circulator& c, double h,
                std::vector<Quantity>& values)
{
  values.clear();
  auto t1 = std::chrono::high_resolution_clock::now();
  estimator.init( c, c );
  estimator.eval( c, c, std::back_inserter( values ), h );
  auto t2 = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
}

/**
 * Estimates the normals along a whole closed curve and compares them
 * with the true normals.
 *
 * @param estimator a normal estimator (model of CCurveLocalGeometricEstimator)
 * @param c a circulator on the elements of the curve
 * @param h the grid step
 * @param trueNormals the true normals at the elements of the curve
 * @param normals (returns) the estimated normals
 */
template <typename Estimator, typename Circulator>
Measure measure(Estimator& estimator, const Circulator& c, double h,
                const std::vector<RealVector>& trueNormals,
                std::vector<RealVector>& normals)
{
  double time = estimate( estimator, c, h, normals );
  ASSERT( normals.size() == trueNormals.size() );
  double sum = 0.0, max = 0.0;
  for (std::size_t i = 0; i < normals.size(); ++i)
    {
      double e = angle( normals[i], trueNormals[i] );
      sum += e;
      max = std::max( max, e );
    }
  return { sum / double( normals.size() ), max, time };
}

/**
 * Estimates the curvature along a whole closed curve and compares it
 * with the true curvature (absolute errors).
 */
template <typename Estimator, typename Circulator>
Measure measureCurvature(Estimator& estimator, const Circulator& c, double h,
                         const std::vector<double>& trueCurvatures)
{
  std::vector<double> curvatures;
  double time = estimate( estimator, c, h, curvatures );
  ASSERT( curvatures.size() == trueCurvatures.size() );
  double sum = 0.0, max = 0.0;
  for (std::size_t i = 0; i < curvatures.size(); ++i)
    {
      double e = std::abs( curvatures[i] - trueCurvatures[i] );
      sum += e;
      max = std::max( max, e );
    }
  return { sum / double( curvatures.size() ), max, time };
}

/**
 * Length of a curve estimated by integration of the normal vectors at
 * its steps: each step, of length h, contributes
 * \f$ h |n \cdot u| \f$, where \f$ n \f$ is the estimated unit normal
 * vector and \f$ u \f$ the trivial normal vector of the step.
 *
 * @param stepNormals the estimated normal vectors at the steps
 * @param trivialNormals the trivial normal vectors of the steps
 * @param h the grid step
 */
double length(const std::vector<RealVector>& stepNormals,
              const std::vector<RealVector>& trivialNormals, double h)
{
  double l = 0.0;
  for (std::size_t k = 0; k < stepNormals.size(); ++k)
    l += h * std::abs( stepNormals[k].dot( trivialNormals[k] ) ) / stepNormals[k].norm();
  return l;
}

/// Results of the estimators for a given shape and a given grid step
struct Results
{
  std::size_t nbPoints;                 ///< number of points of the curve
  double trueLength;                    ///< length of the boundary of the shape
  std::vector<Measure> normals;         ///< measures of the normal estimators
  std::vector<double> lengthErrors;     ///< relative errors of the lengths
  std::vector<Measure> curvatures;      ///< measures of the curvature estimators
};

/**
 * Digitizes a shape with a grid step h, tracks its boundary, estimates
 * the normals with the eight estimators, the lengths by integration of
 * these normals, the curvature with the two estimators based on DCAs,
 * and returns their measures.
 *
 * @param shape a parametric shape
 * @param h the grid step
 */
template <typename Shape>
Results measures(const Shape& shape, double h)
{
  Results results;

  // digitization and boundary tracking
  GaussDigitizer<Space, Shape> dig;
  dig.attach( shape );
  dig.init( shape.getLowerBound() + RealVector( -1.0, -1.0 ),
            shape.getUpperBound() + RealVector( 1.0, 1.0 ), h );
  KSpace K;
  K.init( dig.getLowerBound(), dig.getUpperBound(), true );
  SurfelAdjacency<KSpace::dimension> sAdj( true );
  SCell bel = Surfaces<KSpace>::findABel( K, dig, 1000000 );
  std::vector<Point> boundary;
  Surfaces<KSpace>::track2DBoundaryPoints( boundary, K, sAdj, dig, bel );
  GridCurve<KSpace> curve( K );
  curve.initFromPointsVector( boundary );

  // points and steps of the curve
  typedef GridCurve<KSpace>::PointsRange PointsRange;
  typedef GridCurve<KSpace>::IncidentPointsRange StepsRange;
  typedef GridCurve<KSpace>::ArrowsRange ArrowsRange;
  PointsRange points = curve.getPointsRange();
  StepsRange steps = curve.getIncidentPointsRange();
  ArrowsRange arrows = curve.getArrowsRange();
  results.nbPoints = points.size();
  const std::size_t n = steps.size();

  // true normals at the points and at the middles of the steps,
  // true curvatures at the middles of the steps,
  // trivial normals of the steps
  auto trueNormal = [&shape] (const RealPoint& p)
    { return RealVector( shape.normal( shape.parameter( p ) ) ); };
  std::vector<RealVector> pointNormals, stepNormals, trivialNormals;
  std::vector<double> stepCurvatures;
  for (auto it = points.begin(); it != points.end(); ++it)
    pointNormals.push_back( trueNormal( h * RealPoint( *it ) ) );
  for (auto it = steps.begin(); it != steps.end(); ++it)
    {
      auto pair = *it;
      RealPoint middle = h * 0.5 * ( RealPoint( pair.first ) + RealPoint( pair.second ) );
      stepNormals.push_back( trueNormal( middle ) );
      stepCurvatures.push_back( shape.curvature( shape.parameter( middle ) ) );
      trivialNormals.push_back( RealVector( pair.second ) - RealVector( pair.first ) );
    }
  results.trueLength = shape.arclength( 0.0, 2.0 * M_PI, 1000000 );

  // the k-th step goes from the k-th point to the next one
  std::vector<Point> pointVector( points.begin(), points.end() );
  std::size_t k = 0;
  for (auto it = arrows.begin(); it != arrows.end(); ++it, ++k)
    if ( (*it).first != pointVector[k] )
      trace.warning() << "the " << k << "-th arrow does not begin at the "
                      << k << "-th point" << std::endl;
  ASSERT( points.size() == n );

  // orientation of the curve (sign of its area)
  double area = 0.0;
  for (std::size_t i = 0; i < pointVector.size(); ++i)
    {
      Point p = pointVector[i], q = pointVector[(i+1) % pointVector.size()];
      area += double( p[0] ) * q[1] - double( q[0] ) * p[1];
    }
  const bool isCCW = ( area > 0.0 );

  typedef PointsRange::ConstCirculator PointCirculator;
  typedef StepsRange::ConstCirculator StepCirculator;
  typedef ArrowsRange::ConstCirculator ArrowCirculator;
  PointCirculator pc = points.c();
  StepCirculator sc = steps.c();
  ArrowCirculator ac = arrows.c();

  typedef ArithmeticalDSSComputer<PointCirculator, int, 4> DSSComputer;
  typedef StabbingLineComputer<StepCirculator> LineComputer;
  typedef StabbingCircleComputer<StepCirculator> CircleComputer;
  typedef ArithmeticalDSSComputerOnArrows<ArrowCirculator, int> ArrowDSSComputer;
  typedef NormalFromDSSEstimator<DSSComputer> DSSNormal;
  typedef NormalFromDSSEstimator<ArrowDSSComputer> ArrowDSSNormal;
  typedef NormalFromStabbingLineEstimator<LineComputer> LineNormal;
  typedef NormalFromDCAEstimator<CircleComputer> CircleNormal;

  // normals estimated at the points: the normal at a step is the
  // normalized sum of the normals at its two points
  auto pointsToSteps = [n] (const std::vector<RealVector>& normals)
    {
      std::vector<RealVector> res;
      for (std::size_t i = 0; i < n; ++i)
        {
          RealVector v = normals[i] + normals[(i+1) % n];
          res.push_back( v / v.norm() );
        }
      return res;
    };
  std::vector<RealVector> normals;
  auto addPointResults = [&] (const Measure& m)
    {
      results.normals.push_back( m );
      double l = length( pointsToSteps( normals ), trivialNormals, h );
      results.lengthErrors.push_back( std::abs( l - results.trueLength ) / results.trueLength );
    };
  auto addStepResults = [&] (const Measure& m)
    {
      results.normals.push_back( m );
      double l = length( normals, trivialNormals, h );
      results.lengthErrors.push_back( std::abs( l - results.trueLength ) / results.trueLength );
    };

  {
    MostCenteredMaximalSegmentEstimator<DSSComputer, DSSNormal> e{ DSSComputer(), DSSNormal() };
    addPointResults( measure( e, pc, h, pointNormals, normals ) );
  }
  {
    LambdaMaximalSegmentEstimator<DSSComputer, DSSNormal> e{ DSSComputer(), DSSNormal() };
    addPointResults( measure( e, pc, h, pointNormals, normals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<ArrowDSSComputer, ArrowDSSNormal> e{ ArrowDSSComputer(), ArrowDSSNormal() };
    addStepResults( measure( e, ac, h, stepNormals, normals ) );
  }
  {
    LambdaMaximalSegmentEstimator<ArrowDSSComputer, ArrowDSSNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ ArrowDSSComputer(), ArrowDSSNormal() };
    addStepResults( measure( e, ac, h, stepNormals, normals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<LineComputer, LineNormal> e{ LineComputer(), LineNormal() };
    addStepResults( measure( e, sc, h, stepNormals, normals ) );
  }
  {
    LambdaMaximalSegmentEstimator<LineComputer, LineNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ LineComputer(), LineNormal() };
    addStepResults( measure( e, sc, h, stepNormals, normals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<CircleComputer, CircleNormal> e{ CircleComputer(), CircleNormal() };
    addStepResults( measure( e, sc, h, stepNormals, normals ) );
  }
  {
    LambdaMaximalSegmentEstimator<CircleComputer, CircleNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ CircleComputer(), CircleNormal() };
    addStepResults( measure( e, sc, h, stepNormals, normals ) );
  }

  // curvature, whose sign depends on the orientation of the curve
  auto curvatures = [&] (auto isCCWTag)
    {
      constexpr bool ccw = decltype(isCCWTag)::value;
      typedef CurvatureFromDCAEstimator<CircleComputer, ccw> CircleCurvature;
      {
        MostCenteredMaximalSegmentEstimator<CircleComputer, CircleCurvature> e{ CircleComputer(), CircleCurvature() };
        results.curvatures.push_back( measureCurvature( e, sc, h, stepCurvatures ) );
      }
      {
        LambdaMaximalSegmentEstimator<CircleComputer, CircleCurvature,
          functors::Lambda64Function, functors::StepEccentricity> e{ CircleComputer(), CircleCurvature() };
        results.curvatures.push_back( measureCurvature( e, sc, h, stepCurvatures ) );
      }
    };
  if ( isCCW )
    curvatures( std::true_type() );
  else
    curvatures( std::false_type() );

  return results;
}

/**
 * Multigrid convergence of the estimators on a shape, for several grid
 * steps, written in the files "normals-convergence-[name].txt",
 * "length-convergence-[name].txt" and "curvature-convergence-[name].txt".
 */
template <typename Shape>
void convergence(const Shape& shape, const std::string& name,
                 const std::vector<double>& steps)
{
  const std::vector<std::string> normalEstimators =
    { "DSS-MostCentered", "DSS-LambdaMST",
      "DSSOnArrows-MostCentered", "DSSOnArrows-LambdaMST",
      "StabbingLine-MostCentered", "StabbingLine-LambdaMST",
      "StabbingCircle-MostCentered", "StabbingCircle-LambdaMST" };
  const std::vector<std::string> curvatureEstimators =
    { "StabbingCircle-MostCentered", "StabbingCircle-LambdaMST" };

  std::ofstream normalsOutput( "normals-convergence-" + name + ".txt" );
  std::ofstream lengthOutput( "length-convergence-" + name + ".txt" );
  std::ofstream curvatureOutput( "curvature-convergence-" + name + ".txt" );
  normalsOutput << "# " << name << std::endl << "# h nbPoints";
  for (const std::string& e : normalEstimators)
    normalsOutput << " " << e << "(mean,max,ms)";
  normalsOutput << std::endl;
  lengthOutput << "# " << name << std::endl << "# h nbPoints trueLength";
  for (const std::string& e : normalEstimators)
    lengthOutput << " " << e << "(relative error)";
  lengthOutput << std::endl;
  curvatureOutput << "# " << name << std::endl << "# h nbPoints";
  for (const std::string& e : curvatureEstimators)
    curvatureOutput << " " << e << "(mean,max,ms)";
  curvatureOutput << std::endl;
  std::cout << "# " << name << std::endl;

  for (double h : steps)
    {
      Results r = measures( shape, h );
      std::stringstream normalsLine, lengthLine, curvatureLine;
      normalsLine << h << " " << r.nbPoints;
      for (const Measure& m : r.normals)
        normalsLine << " " << m.meanError << " " << m.maxError << " " << m.time;
      lengthLine << h << " " << r.nbPoints << " " << r.trueLength;
      for (double e : r.lengthErrors)
        lengthLine << " " << e;
      curvatureLine << h << " " << r.nbPoints;
      for (const Measure& m : r.curvatures)
        curvatureLine << " " << m.meanError << " " << m.maxError << " " << m.time;
      normalsOutput << normalsLine.str() << std::endl;
      lengthOutput << lengthLine.str() << std::endl;
      curvatureOutput << curvatureLine.str() << std::endl;
      std::cout << "normals   " << normalsLine.str() << std::endl
                << "length    " << lengthLine.str() << std::endl
                << "curvature " << curvatureLine.str() << std::endl;
    }
}

/*
  Display results using gnuplot

  set terminal pngcairo size 800,600 font "Helvetica,12"
  set logscale xy
  set grid
  set format x "10^{%L}"
  set format y "10^{%L}"
  set key bottom right font ",10"
  set xlabel "grid step h"
  set ylabel "mean angular error (radian)"
  set output "normals-convergence-ellipse.png"
  plot "normals-convergence-ellipse.txt" using 1:3 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-ellipse.txt" using 1:6 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-ellipse.txt" using 1:9 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-ellipse.txt" using 1:12 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-ellipse.txt" using 1:15 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-ellipse.txt" using 1:18 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-ellipse.txt" using 1:21 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "normals-convergence-ellipse.txt" using 1:24 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.08*x**(2./3.) title "h^{2/3}" w l lw 1 lc rgb "black", 0.03*x title "h" w l lw 1 dt 3 lc rgb "black"
  set output "normals-convergence-flower.png"
  plot "normals-convergence-flower.txt" using 1:3 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 1:6 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 1:9 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 1:12 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 1:15 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 1:18 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 1:21 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "normals-convergence-flower.txt" using 1:24 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.12*x**(2./3.) title "h^{2/3}" w l lw 1 lc rgb "black", 0.04*x title "h" w l lw 1 dt 3 lc rgb "black"
  set ylabel "maximal angular error (radian)"
  set output "normals-max-error-ellipse.png"
  plot "normals-convergence-ellipse.txt" using 1:4 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-ellipse.txt" using 1:7 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-ellipse.txt" using 1:10 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-ellipse.txt" using 1:13 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-ellipse.txt" using 1:16 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-ellipse.txt" using 1:19 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-ellipse.txt" using 1:22 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "normals-convergence-ellipse.txt" using 1:25 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.3*x**(1./3.) title "h^{1/3}" w l lw 1 lc rgb "black", 0.1*x**(2./3.) title "h^{2/3}" w l lw 1 dt 3 lc rgb "black"
  set output "normals-max-error-flower.png"
  plot "normals-convergence-flower.txt" using 1:4 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 1:7 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 1:10 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 1:13 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 1:16 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 1:19 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 1:22 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "normals-convergence-flower.txt" using 1:25 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.5*x**(1./3.) title "h^{1/3}" w l lw 1 lc rgb "black", 0.2*x**(2./3.) title "h^{2/3}" w l lw 1 dt 3 lc rgb "black"
  set ylabel "relative error of the length"
  set output "length-convergence-ellipse.png"
  plot "length-convergence-ellipse.txt" using 1:4 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "length-convergence-ellipse.txt" using 1:5 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "length-convergence-ellipse.txt" using 1:6 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "length-convergence-ellipse.txt" using 1:7 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "length-convergence-ellipse.txt" using 1:8 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "length-convergence-ellipse.txt" using 1:9 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "length-convergence-ellipse.txt" using 1:10 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "length-convergence-ellipse.txt" using 1:11 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.002*x title "h" w l lw 1 lc rgb "black", 0.01*x**2 title "h^2" w l lw 1 dt 3 lc rgb "black"
  set output "length-convergence-flower.png"
  plot "length-convergence-flower.txt" using 1:4 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "length-convergence-flower.txt" using 1:5 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "length-convergence-flower.txt" using 1:6 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "length-convergence-flower.txt" using 1:7 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "length-convergence-flower.txt" using 1:8 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "length-convergence-flower.txt" using 1:9 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "length-convergence-flower.txt" using 1:10 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "length-convergence-flower.txt" using 1:11 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       0.004*x title "h" w l lw 1 lc rgb "black", 0.01*x**2 title "h^2" w l lw 1 dt 3 lc rgb "black"
  set ylabel "absolute error of the curvature"
  set output "curvature-convergence-ellipse.png"
  plot "curvature-convergence-ellipse.txt" using 1:3 title "StabbingCircle-MostCentered (mean)" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "curvature-convergence-ellipse.txt" using 1:4 title "StabbingCircle-MostCentered (max)" w lp pt 13 dt 1 lc rgb "#9467bd" lw 2, \
       "curvature-convergence-ellipse.txt" using 1:6 title "StabbingCircle-LambdaMST (mean)" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       "curvature-convergence-ellipse.txt" using 1:7 title "StabbingCircle-LambdaMST (max)" w lp pt 13 dt 2 lc rgb "#9467bd" lw 2, \
       0.05*x**(1./2.) title "h^{1/2}" w l lw 1 lc rgb "black"
  set output "curvature-convergence-flower.png"
  plot "curvature-convergence-flower.txt" using 1:3 title "StabbingCircle-MostCentered (mean)" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "curvature-convergence-flower.txt" using 1:4 title "StabbingCircle-MostCentered (max)" w lp pt 13 dt 1 lc rgb "#9467bd" lw 2, \
       "curvature-convergence-flower.txt" using 1:6 title "StabbingCircle-LambdaMST (mean)" w lp pt 11 dt 2 lc rgb "#d62728" lw 2, \
       "curvature-convergence-flower.txt" using 1:7 title "StabbingCircle-LambdaMST (max)" w lp pt 13 dt 2 lc rgb "#9467bd" lw 2, \
       0.3*x**(1./2.) title "h^{1/2}" w l lw 1 lc rgb "black"
  set key top left font ",10"
  set xlabel "number of points"
  set ylabel "time (ms)"
  set output "normals-timings.png"
  plot "normals-convergence-flower.txt" using 2:5 title "DSS-MostCentered" w lp pt 5 dt 1 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 2:8 title "DSS-LambdaMST" w lp pt 5 dt 2 lc rgb "#1f77b4" lw 2, \
       "normals-convergence-flower.txt" using 2:11 title "DSSOnArrows-MostCentered" w lp pt 9 dt 1 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 2:14 title "DSSOnArrows-LambdaMST" w lp pt 9 dt 2 lc rgb "#ff7f0e" lw 2, \
       "normals-convergence-flower.txt" using 2:17 title "StabbingLine-MostCentered" w lp pt 7 dt 1 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 2:20 title "StabbingLine-LambdaMST" w lp pt 7 dt 2 lc rgb "#2ca02c" lw 2, \
       "normals-convergence-flower.txt" using 2:23 title "StabbingCircle-MostCentered" w lp pt 11 dt 1 lc rgb "#d62728" lw 2, \
       "normals-convergence-flower.txt" using 2:26 title "StabbingCircle-LambdaMST" w lp pt 11 dt 2 lc rgb "#d62728" lw 2
*/

int main()
{
  const std::vector<double> steps = { 1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.01,
                                      0.005, 0.002, 0.001, 0.0005 };

  // convex shape: ellipse of half axes 5 and 3
  Ellipse2D<Space> ellipse( 0.0, 0.0, 5.0, 3.0, 0.3 );
  convergence( ellipse, "ellipse", steps );

  // non-convex shape: flower with 5 petals of radius between 3 and 7
  Flower2D<Space> flower( 0.0, 0.0, 5.0, 2.0, 5, 0.3 );
  convergence( flower, "flower", steps );

  return 0;
}
