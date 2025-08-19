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
 * @file BasicTypes.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/15
 *
 * Header file for module BasicTypes.cpp.
 *
 * This file contains the definition of basic types.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicTypes_RECURSES)
#error Recursive header files inclusion detected in BasicTypes.h
#else // defined(BasicTypes_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicTypes_RECURSES

#if !defined BasicTypes_h
/** Prevents repeated inclusion of headers. */
#define BasicTypes_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstdlib>
#include <cstdint>
#include <iostream>

#include <boost/multiprecision/cpp_int.hpp>
//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{
  ///unsigned 8-bit integer.
  typedef std::uint8_t uint8_t;
  ///unsigned 16-bit integer.
  typedef std::uint16_t uint16_t;
  typedef std::uint16_t uint16_t;
  ///unsigned 32-bit integer.
  typedef std::uint32_t uint32_t;
  ///unsigned 64-bit integer.
  typedef std::uint64_t uint64_t;
  
  ///signed 8-bit integer.  
  typedef std::int8_t int8_t;
  ///signed 16-bit integer.
  typedef std::int16_t int16_t;
  ///signed 32-bit integer.
  typedef std::int32_t int32_t;
  ///signed 94-bit integer.
  typedef std::int64_t int64_t;
  
  typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<>, boost::multiprecision::et_off> BigInteger;
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicTypes_h

#undef BasicTypes_RECURSES
#endif // else defined(BasicTypes_RECURSES)
