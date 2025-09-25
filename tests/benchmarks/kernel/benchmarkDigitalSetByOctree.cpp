#include <iostream>
#include <numeric>
#include <chrono>

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/kernel/sets/DigitalSetByOctree.h"

#include "DGtalCatch.h"

using namespace DGtal;
using namespace std;

using SH3 = Shortcuts<Z3i::KSpace>;
using Octree = DigitalSetByOctree<Z3i::Space>;
using Image = ImageContainerBySTLVector<Z3i::Domain, double>;

const auto CountingFunction = [](Z3i::Point, const std::vector<Z3i::Point>& neighborhood) {
    return neighborhood.size();
};

// Context for each benchmark
struct BenchInfo
{
  inline static std::vector<std::vector<Z3i::Point>> shapes;
  inline static std::vector<Z3i::Domain> domains;
  inline static std::vector<std::string> names;

  BenchInfo() {
    if (shapes.size() != 0) return;

    const auto polys = SH3::getPolynomialList();
    const double steps[]  = { 1.00, 0.50, 0.25 };
    const double noises[] = { 0.00, 0.10  };
    
    for (const auto& poly : polys) {
      for (const auto& step : steps) {
        for (const auto& noise : noises) {

          auto params = SH3::defaultParameters();
          params( "polynomial", poly.second ) 
                ( "gridstep", step) 
                ( "noise", noise);

          auto ishape = SH3::makeImplicitShape3D(params);
          auto dshape = SH3::makeDigitizedImplicitShape3D(ishape, params);
          auto nshape = SH3::makeBinaryImage(dshape, params);

          const std::string name = poly.first + "_" + std::to_string(step) + "_" + std::to_string(noise);
          const auto domain = nshape->domain();

          names.push_back(name);
          domains.push_back(nshape->domain());
          shapes.push_back({});

          for (auto it = domain.begin(); it != domain.end(); ++it) {
            if (nshape->operator()(*it) != 0) {
              shapes.back().push_back(*it);
            }
          }
        }
      }
    }

    std::cout << "Shapes: " << std::endl;
    for (const auto& n : names) {
      std::cout << "\t -" << n << std::endl;
    }
  }
};

TEST_CASE_METHOD(BenchInfo, "Benchmarking DigitalSetByOctree using Catch2", "[catch]")
{
  auto i = GENERATE(range(0, (int)names.size()));

  const auto& name   = names[i];
  const auto& domain = domains[i];
  const auto& pts    = shapes[i];
  
  Octree tree(domain);
  for (const auto& pt : pts) {
    tree.insert(pt);
  }


  BENCHMARK("Octree insert: " + name)
  {
    Octree subtree(domain);
    for (const auto& pt : pts) {
      subtree.insert(pt);
    }
  };

  BENCHMARK_ADVANCED("Octree To DAG: " + name)(Catch::Benchmark::Chronometer meter)
  {
    Octree subtree = tree;
    // Only measure conversion, not copy
    meter.measure([&] {
        subtree.convertToDAG();
    });
  };

  BENCHMARK("Iterating octree: " + name)
  {
    size_t count = 0;
    for (auto it = tree.begin(); it != tree.end(); ++it) {
      count++;
    }
    return count;
  };

  size_t oldMemory = tree.memoryFootprint();
  tree.convertToDAG();
  size_t count = 0; for (auto it = tree.begin(); it != tree.end(); ++it) count++;

  std::cout << "\n\n Stats:\n" << std::endl;
  std::cout << "\t Domain: " << tree.domain() << std::endl;
  std::cout << "\t Memory footprint (octree): " << oldMemory << " bytes." << std::endl;
  std::cout << "\t Memory footprint (dag)   : " << tree.memoryFootprint() << " bytes." << std::endl;
  std::cout << "\t Voxel count (source): " << pts.size() << std::endl;
  std::cout << "\t Voxel count (dag)   : " << count << std::endl;
  std::cout << std::endl;

  BENCHMARK("Counting number of voxel: " + name) 
  {
    tree.computeFunction(tree.begin(), tree.end(), 1, CountingFunction);
  };
};
