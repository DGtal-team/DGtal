#pragma once

/**
 * @file GenericWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/02
 *
 * Header file for module GenericWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericWriter_RECURSES)
#error Recursive header files inclusion detected in GenericWriter.h
#else // defined(GenericWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericWriter_RECURSES

#if !defined GenericWriter_h
/** Prevents repeated inclusion of headers. */
#define GenericWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <fstream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericWriter
  /**
   * Description of template class 'GenericWriter' <p>
   * \brief Aim: implements a generic DGtal object writer with the help of boost 
   * serialisation concepts.
   *  
   * The Object type given by the template parameter @tparam TObject,
   * must implement a boost::serialize method. 
   * 
   */
  template <typename TObject>
  struct GenericWriter
  {
    // ----------------------- Standard services ------------------------------
  
    typedef TObject Object;
 
    /** 
     * Export an Object with the Boost serialisation process
     * with portable XML format.
     * 
     * @param filename name of the output file
     * @param anObject the object to export
     * 
     * @return true if no errors occur.
     */
    static bool exportXML(const std::string & filename, const Object &anObject);
    
    /** 
     * Export an Object with the Boost serialisation process
     * with portable text format.
     * 
     * @param filename name of the output file
     * @param anObject the object to export
     * 
     * @return true if no errors occur.
     */
    static bool exportTXT(const std::string & filename, const Object &anObject);
    

    /** 
     * Export an Object with the Boost serialisation process
     * with non-portable binary format.
     * 
     * @param filename name of the output file
     * @param anObject the object to export
     * 
     * @return true if no errors occur.
     */
    static bool exportBIN(const std::string & filename, const Object &anObject);
    

  };
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/GenericWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericWriter_h

#undef GenericWriter_RECURSES
#endif // else defined(GenericWriter_RECURSES)
