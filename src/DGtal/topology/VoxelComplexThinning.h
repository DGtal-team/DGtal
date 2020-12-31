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
 * @file VoxelComplexThinning.h
 * @author Pablo Hernandez-Cerdan (\c pablo.hernandez.cerdan@outlook.com)
 *
 * @date 2021/01/01
 *
 * Defines easy to use functions for thinning with VoxelComplex.
 *
 * This file is part of the DGtal library.
 */

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/VoxelComplex.h"
#include "DGtal/topology/VoxelComplexFunctions.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
//////////////////////////////////////////////////////////////////////////////
namespace DGtal {
namespace functions {
/*
Get a skeletonized or thinned image from a binary image set.

Parameters:
----------
complex: TComplex
    input complex to thin

skel_type: str
    Voxels to keep in the skeletonization process.

    [end, ulti, isthmus]
    - end: keep end voxels.
    - ulti: don't keep extra voxels, ultimate skeleton.
    - isthmus: keep voxels that are isthmuses.

select_type: str
    [first, random, dmax]
    - first: the first voxel in the set (no criteria)
    - random: random voxel in the set.
    - dmax: choose voxel with greatest distance map value.
    Strategy to choose voxels in the asymmetric process.

table_folder: str
    Location of the DGtal look-up-tables for simplicity and isthmusicity,
    for example simplicity_table26_6.zlib.
    These tables are distributed with the sgext package. Use the variable
    'sgext.tables_folder'.

persistence: int
    if >0, performs a persistence algorithm that prunes
    branches that are not persistant (less important).

distance_transform: str
    file holding a distance map. Required for select_type dmax option.
    This option provides a centered skeleton.

profile: bool
    time the algorithm

verbose: bool
    extra information displayed during the algorithm.

Return
------
A new thinned voxel complex.
*/
template<typename TComplex,
  typename TDistanceTransform =
    DistanceTransformation<Z3i::Space, Z3i::DigitalSet, ExactPredicateLpSeparableMetric<Z3i::Space, 3>> >
TComplex thinningVoxelComplex(
  TComplex & vc,
  const std::string & skel_type_str,
  const std::string & skel_select_type_str,
  const std::string & tables_folder,
  const int & persistence = 0,
  const TDistanceTransform * distance_transform = nullptr,
  const bool profile = false,
  const bool verbose = false)
{
  if(verbose) {
    using DGtal::trace;
    trace.beginBlock("thin_function parameters:");
    trace.info() << "skel_type_str: " << skel_type_str << std::endl;
    trace.info() << "skel_select_type_str: " << skel_select_type_str << std::endl;
    if(distance_transform) {
      trace.info() << " -- provided distance_transform." << std::endl;
    }
    trace.info() << "persistence: " << persistence << std::endl;
    trace.info() << "profile: " << profile << std::endl;
    trace.info() << "verbose: " << verbose << std::endl;
    trace.info() << "----------" << std::endl;
    trace.endBlock();
  }

  // Validate input skel method and skel_select
  const bool skel_type_str_is_valid =
    skel_type_str == "ultimate" ||
    skel_type_str == "end" ||
    skel_type_str == "isthmus" ||
    skel_type_str == "1isthmus" ||
    skel_type_str == "isthmus1";
  if(!skel_type_str_is_valid) {
    throw std::runtime_error("skel_type_str is not valid: \"" + skel_type_str + "\"");
  }

  const bool skel_select_type_str_is_valid =
    skel_select_type_str == "first" ||
    skel_select_type_str == "random" ||
    skel_select_type_str == "dmax";
  if(!skel_select_type_str_is_valid) {
    throw std::runtime_error("skel_select_type_str is not valid: \"" + skel_select_type_str + "\"");
  }
  // // No filesystem to check the tables folder exist. If incorrect, it will fail loading the table.
  // const fs::path tables_folder_path{tables_folder};
  // if(!fs::exists(tables_folder_path)) {
  //   throw std::runtime_error("tables_folder " + tables_folder_path.string() +
  //       " doesn't exist in the filesystem.\n"
  //       "tables_folder should point to the folder "
  //       "where DGtal tables are: i.e simplicity_table26_6.zlib");
  // }

  // Create a VoxelComplex from the set
  using KSpace = DGtal::Z3i::KSpace;
  using Complex = TComplex;
  using ComplexCell = typename Complex::Cell;
  using ComplexClique = typename Complex::Clique;
  using Point = DGtal::Z3i::Point;

  if(verbose) { DGtal::trace.beginBlock("load isthmus table"); }
  boost::dynamic_bitset<> isthmus_table;
  auto &sk = skel_type_str;
  if(sk == "isthmus") {
    const std::string tableIsthmus = tables_folder + "/isthmusicity_table26_6.zlib";
    isthmus_table = *DGtal::functions::loadTable(tableIsthmus);
  } else if(sk == "isthmus1" || sk == "1ishtmus") {
    const std::string tableOneIsthmus = tables_folder + "/isthmusicityOne_table26_6.zlib";
    isthmus_table = *DGtal::functions::loadTable(tableOneIsthmus);
  }
  if(verbose) { DGtal::trace.endBlock(); }


  // SKEL FUNCTION:
  // Load a look-up-table for the neighborgood of a point
  auto pointMap =
      *DGtal::functions::mapZeroPointNeighborhoodToConfigurationMask<Point>();
  std::function<bool(const Complex&, const ComplexCell&)> Skel;
  if(sk == "ultimate") {
    Skel = DGtal::functions::skelUltimate<Complex>;
  } else if(sk == "end") {
    Skel = DGtal::functions::skelEnd<Complex>;
  // else if (sk == "1is") Skel = oneIsthmus<Complex>;
  // else if (sk == "is") Skel = skelIsthmus<Complex>;
  } else if(sk == "isthmus1" || sk == "1ishtmus") {
    Skel = [&isthmus_table, &pointMap](const Complex& fc,
                                       const ComplexCell& c) {
      return DGtal::functions::skelWithTable(isthmus_table, pointMap, fc, c);
    };
  } else if(sk == "isthmus") {
    Skel = [&isthmus_table, &pointMap](const Complex& fc,
                                       const ComplexCell& c) {
      return DGtal::functions::skelWithTable(isthmus_table, pointMap, fc, c);
    };
  } else {
    throw std::runtime_error("Invalid skel string");
  }

  // SELECT FUNCTION
  std::function<std::pair<typename Complex::Cell, typename Complex::Data>(
      const ComplexClique&)>
      Select;

  // profile
  auto start = std::chrono::system_clock::now();

  // If dmax is chosen, but no distance_transform is provided, create one.
  using DT = TDistanceTransform;
  using DTDigitalSet = typename DT::PointPredicate;
  using DTDigitalSetDomain = typename DTDigitalSet::Domain;
  using Metric = typename DT::SeparableMetric;
  Metric l3;
  auto &sel = skel_select_type_str;
  const bool compute_distance_transform = !distance_transform && sel == "dmax";
  DTDigitalSetDomain vc_domain = compute_distance_transform ?
    DTDigitalSetDomain(vc.space().lowerBound(), vc.space().upperBound()) :
    // dummy domain
    DTDigitalSetDomain(Z3i::Point::zero, Z3i::Point::diagonal(1));

  DTDigitalSet image_set = DTDigitalSet(vc_domain);
  if(compute_distance_transform) {
    vc.dumpVoxels(image_set);
  }
  // Create the distance map here (computationally expensive if not dummy).
  DT dist_map(vc_domain, image_set, l3);
  if(compute_distance_transform) {
    distance_transform = &dist_map;
  }

  if(sel == "random") {
    Select = DGtal::functions::selectRandom<Complex>;
  } else if(sel == "first") {
    Select = DGtal::functions::selectFirst<Complex>;
  } else if(sel == "dmax") {
    Select = [&distance_transform](const ComplexClique& clique) {
      return selectMaxValue<TDistanceTransform, Complex>(*distance_transform, clique);
      };
  } else {
    throw std::runtime_error("Invalid skel select type");
  }

  // Perform the thin/skeletonization
  Complex vc_new(vc.space());
  if(persistence == 0) {
    vc_new = DGtal::functions::asymetricThinningScheme<Complex>(vc, Select, Skel, verbose);
  } else {
    vc_new = DGtal::functions::persistenceAsymetricThinningScheme<Complex>(vc, Select, Skel,
                                                         persistence, verbose);
  }

  // profile
  auto end = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
  if(profile) {
    std::cout << "Time elapsed: " << elapsed.count() << std::endl;
  }

  return vc_new;
}

} // namespace functions
} // namespace DGtal
