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
 * @file benchmarkHyperRectDomain.cpp
 * @ingroup Tests
 * @author Roland Denis (\c roland.denis@math.univ-lyon1.fr )
 *
 * @date 2019/10/14
 *
 * This file is part of the DGtal library
 */

/**
 * Description of benchmarkHyperRectDomain <p>
 * Aim: benchmark of \ref HyperRectDomain
 */

#include <iostream>
#include <numeric>
#include <chrono>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

#include "DGtalCatch.h"

using namespace DGtal;
using namespace std;

/// Timer used in tic and toc
auto tic_timer = std::chrono::high_resolution_clock::now();

/// Starts timer
void tic()
{
  tic_timer = std::chrono::high_resolution_clock::now();
}

/// Ends timer and return elapsed time
double toc()
{
  const auto toc_timer = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> time_span = toc_timer - tic_timer;
  return time_span.count();
}

// Context for each benchmark
struct BenchDomain
{
  static constexpr std::size_t dim = 3;
  static constexpr signed long long int size = 200;
  static constexpr std::size_t N = 100;

  using Space = DGtal::SpaceND<dim>;
  using Point = Space::Point;
  using Domain = DGtal::HyperRectDomain<Space>;

  BenchDomain()
    : a(Point::diagonal(0))
    , b(Point::diagonal(size))
    , domain(Domain(a, b))
    , dimensions(Point::dimension)
    {
      std::iota(dimensions.begin(), dimensions.end(), Dimension(0));
    }

  Point a, b;
  Domain domain;
  std::vector<Point::Dimension> dimensions;
};


TEST_CASE_METHOD( BenchDomain, "Benchmarking HyperRectDomain iterators using custom implementation" )
{
  SECTION("Domain forward traversal")
    {
      Point check;
      double duration;

      for (std::size_t i = 0; i < N; ++i)
        {
          tic();
          for (auto const& pt : domain)
            check += pt;
          duration = toc();
        }

      trace.info() << "Domain traversal: " << duration << " s ; " << (domain.size()/duration*1e-9) << " Gpts/s ; check = " << check << std::endl;
    }

  SECTION("Domain reverse traversal")
    {
      Point check;
      double duration;

      for (std::size_t i = 0; i < N; ++i)
        {
          tic();
          for (auto it = domain.rbegin(), it_end = domain.rend(); it != it_end; ++it)
            check += *it;
          duration = toc();
        }

      trace.info() << "Domain reverse traversal: " << duration << " s ; " << (domain.size()/duration*1e-9) << " Gpts/s ; check = " << check << std::endl;
    }

  SECTION("Benchmarking domain traversal using subRange")
    {
      Point check;
      double duration;

      for (std::size_t i = 0; i < N; ++i)
        {
          tic();
          for (auto const& pt : domain.subRange(dimensions))
            check += pt;
          duration = toc();
        }

      trace.info() << "Domain traversal using subRange: " << duration << " s ; " << (domain.size()/duration*1e-9) << " Gpts/s ; check = " << check << std::endl;
    }

  SECTION("Benchmarking domain reverse traversal using subRange")
    {
      Point check;
      const auto range = domain.subRange(dimensions);
      double duration;

      for (std::size_t i = 0; i < N; ++i)
        {
          tic();
          for (auto it = range.rbegin(), it_end = range.rend(); it != it_end; ++it)
            check += *it;
          duration = toc();
        }

      trace.info() << "Domain reverse traversal using subRange: " << duration << " s ; " << (domain.size()/duration*1e-9) << " Gpts/s ; check = " << check << std::endl;
    }
}

/** @ingroup Tests **/
