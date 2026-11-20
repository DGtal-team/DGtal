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
 * @file SegmentComputerUtils.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2011/07/25
 *
 * @deprecated This header is kept for backward compatibility only:
 * include SegmentComputerHelpers.h instead.
 *
 * This file is part of the DGtal library.
 */

#pragma message("SegmentComputerUtils.h is deprecated, include DGtal/geometry/curves/SegmentComputerHelpers.h instead")

#include "DGtal/geometry/curves/SegmentComputerHelpers.h"

namespace DGtal
{
  /**
   * Computes the middle iterator of a given range,
   * i.e. itb + (ite-itb)/2.
   * @deprecated use subRangeMiddle (IteratorFunctions.h) instead.
   * @param itb begin iterator of a range
   * @param ite end iterator of a range
   * @return the middle iterator of the range [itb,ite)
   * @tparam IC iterator or circulator
   */
  template<typename IC>
  [[deprecated("use DGtal::subRangeMiddle instead")]]
  inline
  IC getMiddleIterator(const IC& itb, const IC& ite)
  {
    return subRangeMiddle(itb, ite);
  }
} // namespace DGtal
