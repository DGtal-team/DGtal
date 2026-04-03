#pragma once

#include <vector>

namespace DGtal
{
  /**
   * @brief Data structure returned by Domain splitters
   *
   * This class offers the ability for the splitter to give information
   * about the surfaces or object within the splits.
   * For example, the hintVoxelCount field may allow to pre-allocate buffers.
   */
  template<typename Domain>
  struct SplitInfo
  {

    Domain domain; //< The actual split domain
    uint32_t hintVoxelCount = 0; //< An expected guess for the number of voxel
  };

  /**
   * @brief Splits a domain evenly along all dimensions
   *
   * @tparam The model of domain to split
   */
  template<typename Domain>
  struct RegularDomainSplitter
  {

    //Output spllitted domain type
    typedef std::vector<SplitInfo<Domain>> SplitDomainsInfo;

    /**
     * @brief Splits a domain
     *
     * This functions may split the domain in fewer part than wanted to ensure
     * even subdomains.
     *
     * @param d The domain to split
     * @param splitHint The targeted number of splits
     */
    SplitDomainsInfo operator()(const Domain& d, uint32_t splitHint) const
    {
      // Find best match possible for even splitting
      const uint32_t splitCount = std::floor(std::log(splitHint) / std::log(Domain::dimension));
      const uint32_t totalSplits = std::pow(splitCount, Domain::dimension);

      if (splitCount == 0)
        return { SplitInfo{d, 0} };

      auto splitSize = (d.upperBound() - d.lowerBound()) / (int32_t)splitCount;
      SplitDomainsInfo result;
      result.reserve(totalSplits);

      for (uint32_t i = 0; i < totalSplits; ++i)
      {
        auto start = d.lowerBound();
        auto idx = i;
        for (uint32_t j = 0; j < Domain::dimension; ++j)
        {
          auto k = idx % splitCount;
          start[j] += k * splitSize[j] + k; // +k ensure no overlap between domains
          idx /= splitCount;
        }

        // Make correction to ensure it remains within the domain
        auto end = start + splitSize;
        for (uint32_t j = 0; j < Domain::dimension; ++j)
          end[j] = std::clamp(end[j], d.lowerBound()[j], d.upperBound()[j]);

        result.emplace_back(Domain(start, end), 0);
      }

      return result;
    };
  };


  /**
   * @brief Splits a domain along one of the domain grid axis
   *
   * @tparam The model of domain to split
   */
  template<typename Domain>
  struct AxisDomainSplitter
  {

    //Output spllitted domain type
    typedef std::vector<SplitInfo<Domain>> SplitDomainsInfo;

    /**
     * @brief Regularly splits a domain along on axis
     *
    *
     * @param d The domain to split
     * @param splitHint The targeted number of splits
     * @param dim the split axis
     */
    SplitDomainsInfo operator()(const Domain& d, uint32_t splitHint, typename Domain::Dimension dim) const
    {
      SplitDomainsInfo result;
      result.reserve(splitHint);
      typename Domain::Point extent = d.upperBound() - d.lowerBound();

      auto width = extent[dim] / splitHint;
      typename Domain::Point shift = Domain::Point::zero();
      shift[dim] = width;

      typename Domain::Point start = d.lowerBound();
      typename Domain::Point end = start;
      end[dim] += width;

      for(auto i = 0; i <  splitHint; ++i)
      {
        result.emplace_back(Domain(start, end), 0);
        start += shift;
        end += shift;
      }

      return result;
    };
  };

}
