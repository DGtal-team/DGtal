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

#include <benchmark/benchmark.h>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

using namespace DGtal;
using namespace std;

// Context for each benchmark
struct BenchDomain
  : public benchmark::Fixture
{
  static constexpr std::size_t dim = 3;
  static constexpr signed long long int size = 200;

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


BENCHMARK_DEFINE_F(BenchDomain, DomainTraversal)(benchmark::State& state)
{
  for (auto _ : state)
    {
      Point check;
      for (auto const& pt : domain)
            check += pt;
      benchmark::DoNotOptimize(check);
    }

  state.SetItemsProcessed(domain.size() * state.iterations());
}

BENCHMARK_DEFINE_F(BenchDomain, DomainReverseTraversal)(benchmark::State& state)
{
  for (auto _ : state)
    {
      Point check;
      for (auto it = domain.rbegin(), it_end = domain.rend(); it != it_end; ++it)
            check += *it;
      benchmark::DoNotOptimize(check);
    }

  state.SetItemsProcessed(domain.size() * state.iterations());
}

BENCHMARK_DEFINE_F(BenchDomain, DomainTraversalSubRange)(benchmark::State& state)
{
  for (auto _ : state)
    {
      Point check;
      for (auto const& pt : domain.subRange(dimensions))
            check += pt;
      benchmark::DoNotOptimize(check);
    }

  state.SetItemsProcessed(domain.size() * state.iterations());
}

BENCHMARK_DEFINE_F(BenchDomain, DomainReverseTraversalSubRange)(benchmark::State& state)
{
  for (auto _ : state)
    {
      Point check;
      const auto range = domain.subRange(dimensions);
      for (auto it = range.rbegin(), it_end = range.rend(); it != it_end; ++it)
            check += *it;
      benchmark::DoNotOptimize(check);
    }

  state.SetItemsProcessed(domain.size() * state.iterations());
}

BENCHMARK_REGISTER_F(BenchDomain, DomainTraversal)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BenchDomain, DomainReverseTraversal)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BenchDomain, DomainTraversalSubRange)->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(BenchDomain, DomainReverseTraversalSubRange)->Unit(benchmark::kMillisecond);

int main(int argc, char* argv[])
{
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}

/** @ingroup Tests **/
