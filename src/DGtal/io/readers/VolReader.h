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
 * @file VolReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Header file for module VolReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VolReader_RECURSES)
#error Recursive header files inclusion detected in VolReader.h
#else // defined(VolReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VolReader_RECURSES

#if !defined VolReader_h
/** Prevents repeated inclusion of headers. */
#define VolReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdio>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  struct VolHeader {
    inline static constexpr std::string requiredFields[] {
      "X", "Y", "Z", "Voxel-Size", "Int-Endian", "Voxel-Endian", "Alpha-Color"
    };
    inline static constexpr unsigned int requiredFieldsCount = sizeof(requiredFields) / sizeof(requiredFields[0]);

    VolHeader() { }

    bool parse(std::ifstream& in);

    bool validate() const;
    
    template<typename T>
    T getAs(const std::string& name) const;

    bool exists(const std::string& field) const;

    std::map<std::string, std::string> fields;
  };

  /////////////////////////////////////////////////////////////////////////////
  // template class VolReader
  /**
   * Description of template class 'VolReader' <p>
   * \brief Aim: implements methods to read a "Vol" file format.
   *
   * The main import method "importVol" returns an instance of the template 
   * parameter TImageContainer.
   *
   * The private methods have been backported from the SimpleVol project 
   * (see http://liris.cnrs.fr/david.coeurjolly).
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
   * Image image = VolReader<Image>::importVol("data.vol");
   *
   * trace.info() << image <<endl;
   * ...
   * @endcode
   *
   * @tparam TImageContainer the image container to use. 
   *
   * @tparam TFunctor the type of functor used in the import (by default set to functors::Cast< TImageContainer::Value>) .
   * @see testVolReader.cpp
   */
  template <typename TImageContainer,  
	    typename TFunctor = functors::Cast< typename TImageContainer::Value > >
  struct VolReader
  {
    // ----------------------- Standard services ------------------------------

    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Value Value;
    typedef TFunctor Functor;
    
    BOOST_CONCEPT_ASSERT((  concepts::CUnaryFunctor<TFunctor, unsigned char, Value > )) ;    

    BOOST_STATIC_ASSERT(ImageContainer::Domain::dimension == 3);


    /** 
     * Main method to import a Vol into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param filename the file name to import.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (by
     * default set to functors::Cast < TImageContainer::Value > .
     *
 
     * @return an instance of the ImageContainer.
     */
    static ImageContainer importVol(const std::string & filename, 
                                    const Functor & aFunctor =  Functor());
    
  }; // end of class VolReader


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/VolReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VolReader_h

#undef VolReader_RECURSES
#endif // else defined(VolReader_RECURSES)
