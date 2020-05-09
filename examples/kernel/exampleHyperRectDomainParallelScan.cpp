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
 * @file
 * @ingroup Examples
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2019/10/16
 *
 * @brief Example of parallelization of an HyperRectDomain scan and ImageContainerBySTL initialization.
 *
 * This file is part of the DGtal library.
 */


//////////////////////////////////////////////////////////////////////////////
// Inclusions

#include <numeric>
#include <iterator>
#include <chrono>
#include <string>
#include <iostream>
#include <iomanip>
#include <omp.h>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/base/SimpleConstRange.h"

///////////////////////////////////////////////////////////////////////////////
using namespace DGtal;

// Timer used in tic and toc
auto tic_timer = std::chrono::high_resolution_clock::now();

// Starts timer
void tic()
{
  tic_timer = std::chrono::high_resolution_clock::now();
}

// Ends timer and return elapsed time
double toc()
{
  const auto toc_timer = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> time_span = toc_timer - tic_timer;
  return time_span.count();
}

//! [split_range]
// Splits range in given parts count and returns the part of given idx
template <typename TIterator>
SimpleConstRange<TIterator>
split_range(TIterator it_begin, TIterator it_end, std::size_t idx, std::size_t count)
{
  auto range_size  = std::distance(it_begin, it_end);
  auto begin_shift = (range_size*idx) / count;
  auto end_shift   = (range_size*(idx+1)) / count;
  return { it_begin + begin_shift, it_begin + end_shift };
}

// Splits range in given parts count and returns the part of given idx
template <typename TIterable>
auto
split_range(TIterable & iterable, std::size_t idx, std::size_t count)
    -> decltype(split_range(iterable.begin(), iterable.end(), idx, count))
{
  return split_range(iterable.begin(), iterable.end(), idx, count);
}
//! [split_range]

// Returns a kind of checksum of a given image (to avoid aggressive optimization)
template <typename Image>
typename Image::Value calc_image_checksum(Image const& image)
{
  typename Image::Value sum = 0;

  #pragma omp parallel reduction(+:sum)
    {
      std::size_t thread_idx = omp_get_thread_num();
      std::size_t thread_cnt = omp_get_max_threads();
      auto const range = split_range(image, thread_idx, thread_cnt);
      sum = std::accumulate(range.begin(), range.end(), sum);
    }

  return sum;
}

// Sum a function over a given domain
template <typename Domain, typename Function>
auto sum_fn_on_domain(Domain const& domain, Function const& fn)
    -> decltype(fn(domain.lowerBound()))
{
  //! [domain_scan]
  auto sum = 0 * fn(domain.lowerBound()); // To initialize the sum depending on the function return type

  #pragma omp parallel reduction(+:sum)
    {
      // OpenMP context
      std::size_t thread_idx = omp_get_thread_num();
      std::size_t thread_cnt = omp_get_num_threads();

      for (auto const& pt : split_range(domain, thread_idx, thread_cnt))
        sum += fn(pt);
    }
  //! [domain_scan]

  return sum;
}

// Initialize an image with a given function, using getter and setter
template <typename Image, typename Function>
void init_image_getset(Image & image, Function const& fn)
{
  //! [image_init_getset]
  #pragma omp parallel
    {
      std::size_t thread_idx = omp_get_thread_num();
      std::size_t thread_cnt = omp_get_num_threads();

      for (auto const& pt : split_range(image.domain(), thread_idx, thread_cnt))
        image.setValue(pt, fn(pt, image(pt)));
    }
  //! [image_init_getset]
}

// Initialize an image with a given function, using iterators
template <typename Image, typename Function>
void init_image_iter(Image & image, Function const& fn)
{
  //! [image_init_iter]
  #pragma omp parallel
    {
      std::size_t thread_idx = omp_get_thread_num();
      std::size_t thread_cnt = omp_get_num_threads();

      auto domain_it = split_range(image.domain(), thread_idx, thread_cnt).begin();
      for (auto & v : split_range(image, thread_idx, thread_cnt))
        {
          v = fn(*domain_it, v);
          ++domain_it;
        }
    }
  //! [image_init_iter]
}

int main(int argc, char* argv[])
{
  using Space   = SpaceND<3>;
  using Point   = Space::Point;
  using Domain  = HyperRectDomain<Space>;
  using Value   = double;
  using Image   = ImageContainerBySTLVector<Domain, Value>;

  if (argc < 2)
    {
      std::cerr << "Usage: " << argv[0] << " <domain_size>" << std::endl;
      return 1;
    }

  trace.info() << "Initialization..." << std::endl;
  std::size_t domain_size = std::stoll(argv[1]);
  Domain domain(Point::diagonal(0), Point::diagonal(domain_size-1));
  Image image(domain);

  double ref_duration = 0;
  std::size_t max_threads = omp_get_max_threads();
  trace.info() << std::fixed << std::setprecision(6);

  /////////////////////////////////////////////////////////////////////////////
  // Choose here the function you want to use

  //auto const fn = [&domain] (Point const& pt) { return 25 * ( std::cos( (pt - domain.upperBound()).norm() ) + 1 ); }; // CPU intensive
  auto const fn = [&domain] (Point const& pt) { return (pt - domain.upperBound()).norm(); }; // Mixed
  //auto const fn = [] (Point const& pt) { return Value(pt[0]); }; // Memory bound


  /////////////////////////////////////////////////////////////////////////////
  // Scanning a domain in parallel

  trace.info() << std::endl;
  trace.info() << "Scanning a domain in parallel..." << std::endl;
  for (std::size_t thread_cnt = 1; thread_cnt <= max_threads; ++thread_cnt)
    {
      omp_set_num_threads(thread_cnt);
      Value sum = sum_fn_on_domain(domain, fn);
      tic();
      sum += sum_fn_on_domain(domain, fn);
      const double duration = toc();

      if (thread_cnt == 1)
        ref_duration = duration;

      trace.info() << "\tthreads: " << thread_cnt
                   << "\tduration: " << duration << "s"
                   << "\tspeed: " << 1e-6 * domain.size() / duration << "Mpt/s"
                   << "\tspeedup: " << ref_duration/duration
                   << "\tchecksum: " << sum
                   << std::endl;
    }


  /////////////////////////////////////////////////////////////////////////////
  // Initializing an image in parallel using getter and setter

  trace.info() << std::endl;
  trace.info() << "Initializing an image in parallel using getter and setter..." << std::endl;
  for (std::size_t thread_cnt = 1; thread_cnt <= max_threads; ++thread_cnt)
    {
      omp_set_num_threads(thread_cnt);
      init_image_getset(image, [&fn] (Point const& pt, Value) { return fn(pt); });
      tic();
      init_image_getset(image, [&fn] (Point const& pt, Value v) { return v + fn(pt); });
      const double duration = toc();

      if (thread_cnt == 1)
        ref_duration = duration;

      trace.info() << "\tthreads: " << thread_cnt
                   << "\tduration: " << duration << "s"
                   << "\tspeed: " << 1e-6 * domain.size() / duration << "Mpt/s"
                   << "\tspeedup: " << ref_duration/duration
                   << "\tchecksum: " << calc_image_checksum(image)
                   << std::endl;
    }


  /////////////////////////////////////////////////////////////////////////////
  // Initializing an image in parallel using iterators

  trace.info() << std::endl;
  trace.info() << "Initializing an image in parallel using iterators..." << std::endl;
  for (std::size_t thread_cnt = 1; thread_cnt <= max_threads; ++thread_cnt)
    {
      omp_set_num_threads(thread_cnt);
      init_image_iter(image, [&fn] (Point const& pt, Value) { return fn(pt); });
      tic();
      init_image_iter(image, [&fn] (Point const& pt, Value v) { return v + fn(pt); });
      const double duration = toc();

      if (thread_cnt == 1)
        ref_duration = duration;

      trace.info() << "\tthreads: " << thread_cnt
                   << "\tduration: " << duration << "s"
                   << "\tspeed: " << 1e-6 * domain.size() / duration << "Mpt/s"
                   << "\tspeedup: " << ref_duration/duration
                   << "\tchecksum: " << calc_image_checksum(image)
                   << std::endl;
    }

  return 0;
}
