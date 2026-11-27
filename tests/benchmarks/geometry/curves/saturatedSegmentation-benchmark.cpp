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
 * @file geometry/curves/saturatedSegmentation-benchmark.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Benchmark of the saturated segmentation of digital circles into
 * maximal DSSs, for the different categories of segment computers.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <limits>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

/**
   This benchmark compares the running times of the saturated
   segmentation (SaturatedSegmentation) of digital circles into maximal
   standard DSSs (ArithmeticalDSSComputer), according to the category of
   the segment computer, which selects the algorithms of
   SegmentComputerHelpers.h used to go from a maximal segment to the next one:
   - ForwardSegmentComputer: forward extensions only, the backward
     extensions being done by forward extensions of the reversed segment
     computer;
   - BidirectionalSegmentComputer: forward and backward extensions;
   - DynamicSegmentComputer: forward extensions and retractions;
   - DynamicBidirectionalSegmentComputer: all operations (default category
     of ArithmeticalDSSComputer).

\verbatim
saturatedSegmentation-benchmark
\endverbatim

   The boundaries of digital disks of increasing radii are segmented on
   closed curves (circulators), in the default mode
   (SaturatedSegmentation::Mode::MostCentered). For each radius, the
   segmentation is computed several times and the shortest running time
   is kept. The results are written on the standard output and in the file
   "timings-saturated-segmentation.txt", each line containing the radius,
   the number of points of the curve, the number of maximal segments and
   the running times (ms) for each category.

   \image html timings-saturated-segmentation.png "Running times (ms) of the saturated segmentation of the boundary of digital disks into maximal standard DSSs, as a function of the number of points of the boundary, for the different categories of ArithmeticalDSSComputer."

   \example benchmarks/geometry/curves/saturatedSegmentation-benchmark.cpp
*/

using namespace std;
using namespace DGtal;

using KSpace = KhalimskySpaceND<2, int>;
using Point = KSpace::Point;
using ConstIterator = std::vector<Point>::const_iterator;
using ConstCirculator = Circulator<ConstIterator>;

/**
 * Predicate returning true for the points of the digital disk of
 * radius r centered at the origin.
 */
struct DiskPredicate
{
  typedef ::Point Point;
  int r;
  bool operator()(const Point& p) const
  { return int64_t(p[0])*p[0] + int64_t(p[1])*p[1] <= int64_t(r)*r; }
};

/**
 * Returns the 4-connected boundary of the digital disk of radius @a r
 * centered at the origin, as a counterclockwise closed sequence of points.
 */
std::vector<Point> circle(int r)
{
  DiskPredicate inDisk{ r };
  KSpace kspace;
  kspace.init( Point::diagonal(-r-2), Point::diagonal(r+2), true );
  SurfelAdjacency<KSpace::dimension> sAdj( true );
  KSpace::SCell bel = Surfaces<KSpace>::findABel( kspace, inDisk, 100000 );
  std::vector<Point> points;
  Surfaces<KSpace>::track2DBoundaryPoints( points, kspace, sAdj, inDisk, bel );
  return points;
}

/**
 * Computes the saturated segmentation of a closed curve several times
 * and returns the shortest running time in ms.
 *
 * @param points the points of the closed curve
 * @param nbSegments (returns) the number of maximal segments
 * @param nbRuns number of computations
 * @tparam TCategory category of the segment computer
 */
template <typename TCategory>
double timings(const std::vector<Point>& points, std::size_t& nbSegments, int nbRuns)
{
  using SegmentComputer = ArithmeticalDSSComputer<ConstCirculator, int, 4, TCategory>;
  using Segmentation = SaturatedSegmentation<SegmentComputer>;

  ConstCirculator c( points.cbegin(), points.cbegin(), points.cend() );
  double best = std::numeric_limits<double>::max();
  for (int k = 0; k < nbRuns; ++k)
    {
      auto t1 = std::chrono::high_resolution_clock::now();
      Segmentation segmentation( c, c, SegmentComputer() );
      std::size_t n = 0;
      for (auto it = segmentation.begin(), itEnd = segmentation.end(); it != itEnd; ++it)
        ++n;
      auto t2 = std::chrono::high_resolution_clock::now();
      double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      best = std::min( best, dt / 1e6 );
      nbSegments = n;
    }
  return best;
}

/*
  Display results using gnuplot

  set terminal pngcairo size 800,560 font "Helvetica,12"
  set output "timings-saturated-segmentation.png"
  set logscale xy
  set key top left
  set grid
  set xlabel "number of points"
  set ylabel "time (ms)"
  set format x "10^{%L}"
  set format y "10^{%L}"
  plot "timings-saturated-segmentation.txt" using 2:4 title "Forward" w lp pt 5 lw 2, \
       "timings-saturated-segmentation.txt" using 2:5 title "Bidirectional" w lp pt 7 lw 2, \
       "timings-saturated-segmentation.txt" using 2:6 title "Dynamic" w lp pt 9 lw 2, \
       "timings-saturated-segmentation.txt" using 2:7 title "DynamicBidirectional" w lp pt 11 lw 2
*/

int main()
{
  const std::vector<int> radii = { 10, 20, 50, 100, 200, 500, 1000, 2000, 5000,
                                   10000, 20000, 50000, 100000 };
  const std::string fname = "timings-saturated-segmentation.txt";
  std::ofstream output( fname );
  const std::string header =
    "# radius nbPoints nbSegments Forward(ms) Bidirectional(ms) Dynamic(ms) DynamicBidirectional(ms)";
  output << header << std::endl;
  std::cout << header << std::endl;

  for (int r : radii)
    {
      std::vector<Point> points = circle( r );
      // more runs for small curves, so that the measures are stable
      int nbRuns = std::max( 3, int( 2000000 / ( points.size() + 1 ) ) );
      nbRuns = std::min( nbRuns, 1000 );
      std::size_t nF = 0, nB = 0, nD = 0, nDB = 0;
      double tF  = timings<ForwardSegmentComputer>( points, nF, nbRuns );
      double tB  = timings<BidirectionalSegmentComputer>( points, nB, nbRuns );
      double tD  = timings<DynamicSegmentComputer>( points, nD, nbRuns );
      double tDB = timings<DynamicBidirectionalSegmentComputer>( points, nDB, nbRuns );
      if ( (nF != nB) || (nF != nD) || (nF != nDB) )
        trace.warning() << "Different numbers of maximal segments for radius " << r
                        << ": " << nF << " " << nB << " " << nD << " " << nDB << std::endl;
      std::stringstream line;
      line << r << " " << points.size() << " " << nF << " "
           << tF << " " << tB << " " << tD << " " << tDB;
      output << line.str() << std::endl;
      std::cout << line.str() << std::endl;
    }
  output.close();
  return 0;
}
