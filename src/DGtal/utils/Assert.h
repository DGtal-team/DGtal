#pragma once

/** 
 * @file Assert.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/15
 * 
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(Assert_RECURSES)
#error Recursive header files inclusion detected in Assert.h
#else // defined(Assert_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Assert_RECURSES

#if !defined Assert_h
/** Prevents repeated inclusion of headers. */
#define Assert_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>

#include <boost/assert.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /**
  * DGtal Assert function.
  * At this point, it is just a redirect to the boost/assert.hpp macro.
  *
  **/
#define ASSERT(expr) BOOST_ASSERT(expr)
#if defined(CHECK_ALL_PRE)
#define ASSERT_ALL_PRE(expr) BOOST_ASSERT(expr)
#else // defined(CHECK_ALL_PRE)
#define ASSERT_ALL_PRE(expr)
#endif
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Assert_h

#undef Assert_RECURSES
#endif // else defined(Assert_RECURSES)
