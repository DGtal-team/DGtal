#pragma once

/**
 * @file GenericReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/02
 *
 * Header file for module GenericReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericReader_RECURSES)
#error Recursive header files inclusion detected in GenericReader.h
#else // defined(GenericReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericReader_RECURSES

#if !defined GenericReader_h
/** Prevents repeated inclusion of headers. */
#define GenericReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericReader
  /**
   * Description of template class 'GenericReader' <p>
   * \brief Aim: implements a generic DGtal object reader with the help of boost 
   * serialisation concepts.
   *  
   * The Object type given by the template parameter @tparam TObject,
   * must implement a boost::serialize method. 
   * 
   */
  template <typename TObject>
  struct GenericReader
  {
    // ----------------------- Standard services ------------------------------
  
    typedef TObject Object;
 
    /** 
     * Import an Object with the Boost serialisation process
     * with portable XML format.
     * 
     * @param filename name of the output file
     * @return an instance of  Object.
     */
    static Object & importXML(const std::string & filename);
    
    /** 
     * Import an Object with the Boost serialisation process
     * with portable text format.
     * 
     * @param filename name of the output file
     * @return an instance of  Object.  
     */
    static Object & importTXT(const std::string & filename);
    

    /** 
     * Import an Object with the Boost serialisation process
     * with non-portable binary format.
     * 
     * @param filename name of the output file
     * 
     * @return an instance of  Object.
     */
    static Object & importBIN(const std::string & filename);
    

  };
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/GenericReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericReader_h

#undef GenericReader_RECURSES
#endif // else defined(GenericReader_RECURSES)
