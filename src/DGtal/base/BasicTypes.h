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
#include <iostream>
#include <boost/cstdint.hpp>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{
  typedef unsigned int uint;

  typedef boost::uint16_t uint16_t;
  typedef boost::uint32_t uint32_t;
  typedef boost::uint64_t uint64_t;
  
  typedef boost::int16_t int16_t;
  typedef boost::int32_t int32_t;
  typedef boost::int64_t int64_t;

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicTypes_h

#undef BasicTypes_RECURSES
#endif // else defined(BasicTypes_RECURSES)
