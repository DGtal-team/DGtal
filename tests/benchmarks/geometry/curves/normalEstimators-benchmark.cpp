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
 * normal estimators based on maximal segments.
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
   orientation of the normals). The results are written on the standard
   output and in the files "normals-convergence-ellipse.txt" and
   "normals-convergence-flower.txt", each line containing the grid step,
   the number of points of the curve, then, for each estimator, the mean
   error, the maximal error and the running time (ms).

   \image html normals-convergence-ellipse.png "Mean angular error (radian) of the normal estimators along the boundary of a digitized ellipse as a function of the grid step h."
   \image html normals-max-error-ellipse.png "Maximal angular error (radian) of the normal estimators along the boundary of a digitized ellipse as a function of the grid step h."
   \image html normals-convergence-flower.png "Mean angular error (radian) of the normal estimators along the boundary of a digitized flower as a function of the grid step h."
   \image html normals-max-error-flower.png "Maximal angular error (radian) of the normal estimators along the boundary of a digitized flower as a function of the grid step h."
   \image html normals-timings.png "Running times (ms) of the normal estimators along the boundary of a digitized flower as a function of the number of points."

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
 * Estimates the normals along a whole closed curve and compares them
 * with the true normals.
 *
 * @param estimator a normal estimator (model of CCurveLocalGeometricEstimator)
 * @param c a circulator on the elements of the curve
 * @param h the grid step
 * @param trueNormals the true normals at the elements of the curve
 */
template <typename Estimator, typename Circulator>
Measure measure(Estimator& estimator, const Circulator& c, double h,
                const std::vector<RealVector>& trueNormals)
{
  std::vector<RealVector> normals;
  normals.reserve( trueNormals.size() );
  auto t1 = std::chrono::high_resolution_clock::now();
  estimator.init( c, c );
  estimator.eval( c, c, std::back_inserter( normals ), h );
  auto t2 = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  ASSERT( normals.size() == trueNormals.size() );
  double sum = 0.0, max = 0.0;
  for (std::size_t i = 0; i < normals.size(); ++i)
    {
      double e = angle( normals[i], trueNormals[i] );
      sum += e;
      max = std::max( max, e );
    }
  return { sum / double( normals.size() ), max, dt / 1e6 };
}

/**
 * Digitizes a shape with a grid step h, tracks its boundary, estimates
 * the normals with the eight estimators and returns their measures.
 *
 * @param shape a parametric shape
 * @param h the grid step
 * @param nbPoints (returns) the number of points of the boundary
 */
template <typename Shape>
std::vector<Measure> measures(const Shape& shape, double h, std::size_t& nbPoints)
{
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

  // points and steps of the curve, and true normals
  typedef GridCurve<KSpace>::PointsRange PointsRange;
  typedef GridCurve<KSpace>::IncidentPointsRange StepsRange;
  typedef GridCurve<KSpace>::ArrowsRange ArrowsRange;
  PointsRange points = curve.getPointsRange();
  StepsRange steps = curve.getIncidentPointsRange();
  ArrowsRange arrows = curve.getArrowsRange();
  nbPoints = points.size();

  auto trueNormal = [&shape] (const RealPoint& p)
    { return RealVector( shape.normal( shape.parameter( p ) ) ); };
  std::vector<RealVector> pointNormals, stepNormals;
  for (auto it = points.begin(); it != points.end(); ++it)
    pointNormals.push_back( trueNormal( h * RealPoint( *it ) ) );
  for (auto it = steps.begin(); it != steps.end(); ++it)
    {
      auto pair = *it;
      RealPoint middle = 0.5 * ( RealPoint( pair.first ) + RealPoint( pair.second ) );
      stepNormals.push_back( trueNormal( h * middle ) );
    }

  typedef PointsRange::ConstCirculator PointCirculator;
  typedef StepsRange::ConstCirculator StepCirculator;
  PointCirculator pc = points.c();
  StepCirculator sc = steps.c();
  typedef ArrowsRange::ConstCirculator ArrowCirculator;
  ArrowCirculator ac = arrows.c();

  typedef ArithmeticalDSSComputer<PointCirculator, int, 4> DSSComputer;
  typedef StabbingLineComputer<StepCirculator> LineComputer;
  typedef StabbingCircleComputer<StepCirculator> CircleComputer;
  typedef ArithmeticalDSSComputerOnArrows<ArrowCirculator, int> ArrowDSSComputer;
  typedef NormalFromDSSEstimator<DSSComputer> DSSNormal;
  typedef NormalFromDSSEstimator<ArrowDSSComputer> ArrowDSSNormal;
  typedef NormalFromStabbingLineEstimator<LineComputer> LineNormal;
  typedef NormalFromDCAEstimator<CircleComputer> CircleNormal;

  std::vector<Measure> res;
  {
    MostCenteredMaximalSegmentEstimator<DSSComputer, DSSNormal> e{ DSSComputer(), DSSNormal() };
    res.push_back( measure( e, pc, h, pointNormals ) );
  }
  {
    LambdaMaximalSegmentEstimator<DSSComputer, DSSNormal> e{ DSSComputer(), DSSNormal() };
    res.push_back( measure( e, pc, h, pointNormals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<ArrowDSSComputer, ArrowDSSNormal> e{ ArrowDSSComputer(), ArrowDSSNormal() };
    res.push_back( measure( e, ac, h, stepNormals ) );
  }
  {
    LambdaMaximalSegmentEstimator<ArrowDSSComputer, ArrowDSSNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ ArrowDSSComputer(), ArrowDSSNormal() };
    res.push_back( measure( e, ac, h, stepNormals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<LineComputer, LineNormal> e{ LineComputer(), LineNormal() };
    res.push_back( measure( e, sc, h, stepNormals ) );
  }
  {
    LambdaMaximalSegmentEstimator<LineComputer, LineNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ LineComputer(), LineNormal() };
    res.push_back( measure( e, sc, h, stepNormals ) );
  }
  {
    MostCenteredMaximalSegmentEstimator<CircleComputer, CircleNormal> e{ CircleComputer(), CircleNormal() };
    res.push_back( measure( e, sc, h, stepNormals ) );
  }
  {
    LambdaMaximalSegmentEstimator<CircleComputer, CircleNormal,
      functors::Lambda64Function, functors::StepEccentricity> e{ CircleComputer(), CircleNormal() };
    res.push_back( measure( e, sc, h, stepNormals ) );
  }
  return res;
}

/**
 * Multigrid convergence of the estimators on a shape, for several grid
 * steps, written in the file "normals-convergence-[name].txt".
 */
template <typename Shape>
void convergence(const Shape& shape, const std::string& name,
                 const std::vector<double>& steps)
{
  const std::string fname = "normals-convergence-" + name + ".txt";
  std::ofstream output( fname );
  std::stringstream header;
  header << "# " << name << std::endl
         << "# h nbPoints";
  for (std::string e : { "DSS-MostCentered", "DSS-LambdaMST",
                         "DSSOnArrows-MostCentered", "DSSOnArrows-LambdaMST",
                         "StabbingLine-MostCentered", "StabbingLine-LambdaMST",
                         "StabbingCircle-MostCentered", "StabbingCircle-LambdaMST" })
    header << " " << e << "(mean,max,ms)";
  output << header.str() << std::endl;
  std::cout << header.str() << std::endl;
  for (double h : steps)
    {
      std::size_t nbPoints = 0;
      std::vector<Measure> res = measures( shape, h, nbPoints );
      std::stringstream line;
      line << h << " " << nbPoints;
      for (const Measure& m : res)
        line << " " << m.meanError << " " << m.maxError << " " << m.time;
      output << line.str() << std::endl;
      std::cout << line.str() << std::endl;
    }
  output.close();
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
