#pragma once

/**
 * @file Exceptions.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/06
 *
 * Header file for module Exceptions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Exceptions_RECURSES)
#error Recursive header files inclusion detected in Exceptions.h
#else // defined(Exceptions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Exceptions_RECURSES

#if !defined Exceptions_h
/** Prevents repeated inclusion of headers. */
#define Exceptions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   * IOException derived class.
   */ 
  class IOException: public exception
  {
    virtual const char* what() const throw()
    {
      return "DGtal IO error";
    }
  };

  /**
   * MemoryException derived class.
   */ 
  class MemoryException: public exception
  {
    virtual const char* what() const throw()
    {
      return "DGtal memory error";
    }
  };

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Exceptions_h

#undef Exceptions_RECURSES
#endif // else defined(Exceptions_RECURSES)
