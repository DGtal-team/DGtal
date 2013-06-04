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

#if defined(BOOST_DISABLE_ASSERT) || defined(NDEBUG)
 #define ASSERT_MSG(expr,msg) ((void)0)
#else
#define ASSERT_MSG(expr,msg) if (!expr) {trace.error()<<msg<<std::endl;} BOOST_ASSERT(expr)
#endif
 
#define VERIFY(expr) BOOST_VERIFY(expr)
#define VERIFY_MSG(expr,msg) if (!expr) {trace.error()<<msg<<std::endl;} BOOST_VERIFY(expr)


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
