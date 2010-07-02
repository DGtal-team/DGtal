#pragma once

/**
 * @file BasicFunctors.h
 * @author Guillaume Damiand (\c guillaume.damiand@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for module BasicFunctors.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicFunctors_RECURSES)
#error Recursive header files inclusion detected in BasicFunctors.h
#else // defined(BasicFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicFunctors_RECURSES

#if !defined BasicFunctors_h
/** Prevents repeated inclusion of headers. */
#define BasicFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <algorithm>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  template<typename T>
  struct MinFunctor
  {
    T operator() (const T&a, const T&b) const
    { return std::min(a,b); }
  };
  
  template<typename T>
  struct MaxFunctor
  {
    T operator() (const T&a, const T&b) const
    { return std::max(a,b); }
  };
}
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicFunctors_h

#undef BasicFunctors_RECURSES
#endif // else defined(BasicFunctors_RECURSES)
