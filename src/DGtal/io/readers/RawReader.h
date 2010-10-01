#pragma once

/**
 * @file RawReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Header file for module RawReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RawReader_RECURSES)
#error Recursive header files inclusion detected in RawReader.h
#else // defined(RawReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RawReader_RECURSES

#if !defined RawReader_h
/** Prevents repeated inclusion of headers. */
#define RawReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <cstdio>
#include "DGtal/base/Common.h"
#include <boost/static_assert.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class RawReader
  /**
   * Description of template class 'RawReader' <p>
   * \brief Aim: implements methods to read a "Vol" file format.
   *
   * The main import method "importRaw8" returns an instance of the template 
   * parameter TImageContainer.
   *
   * Example usage:
   * @code
   * ...
   * typedef SpaceND<int,3> Space3;
   * typedef HyperRectDomain<Space3> TDomain;
   * typedef TDomain::Point Point;
   *
   * //Default image container = STLVector
   * typedef ImageSelector<TDomain, int>::Type Image;
   * 
   * RawReader<Image> reader;
   * Image image = reader.importRaw8("data.raw");
   *
   * trace.info() << image <<endl;
   * ...
   * @endcode
   *
   * @tparam TImageContainer the image container to use. 
   *
   * @see testRawReader.cpp
   */
  template <typename TImageContainer>
  struct RawReader
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain::Vector Vector;

    BOOST_STATIC_ASSERT( (ImageContainer::Domain::staticDimension == 2) || 
			 (ImageContainer::Domain::staticDimension == 3));


    /** 
     * Main method to import a Raw (8bits) into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param filename the file name to import.
     * @param extent the size of the raw data set.
     * @return an instance of the ImageContainer.
     */
    static ImageContainer importRaw8(const std::string & filename,
				     const Vector & extent) throw(DGtal::DGtalIOException);
    
  }; // end of class RawReader


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/RawReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RawReader_h

#undef RawReader_RECURSES
#endif // else defined(RawReader_RECURSES)
