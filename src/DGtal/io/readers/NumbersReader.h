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
 * @file NumbersReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/11/30
 *
 * Header file for module NumbersReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NumbersReader_RECURSES)
#error Recursive header files inclusion detected in NumbersReader.h
#else // defined(NumbersReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NumbersReader_RECURSES

#if !defined NumbersReader_h
/** Prevents repeated inclusion of headers. */
#define NumbersReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/FreemanChain.h" 
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class NumbersReader
  /**
   * Description of class 'NumbersReader' <p> 
   *  \brief Aim: Implements method to read a set of numbers represented in each line of a file.
   *
   *
   *  The main method to read a set of numbers where each number is
   * given in a single line. Blank line or line beginning with "#" are
   * skipped.
   *
   *  
   * Simple example:
   * 
   *  @code
   *  #include "DGtal/helpers/StdDefs.h"
   *  #include "DGtal/io/readers/NumbersReader.h"
   *  .... 
   *  string filename= "testFile.dat"; 
   *  vector<Z2i::Point> vectPoints = NumbersReader<unsigned int>::getPointsFromFile(filename);
   * @endcode 
   * and you can specifying the point position:
   *  @code
   *  vector<unsigned int> vectPoints = NumbersReader<unsigned int>::getPointsFromFile(filename, 2);
   *  @endcode
   *   
   * @see testNumbersReader.cpp
   * @tparam TNumber the type fo the integer to be read.
   **/

  template <typename TNumber>
  struct  NumbersReader
  {
    // ----------------------- Standard services ------------------------------
  public:
  
  
    /** 
     * Method to import a vector containing a list of integers given
     * in an input stream. One integer is extracted on each line of the input
     * stream.  Blank line or line beginning with "#" are skipped.
     *
     * @param in the input stream.
     * @param aPosition the position of indices where the integer has to extracted.
     * @return a vector containing the set of integer.
     **/
    static std::vector< TNumber >  
    getNumbersFromFile (const std::string & aFilename, 
                        unsigned int aPosition);  
    
    /** 
     * Method to import a vector containing a list of integers given
     * in a file. One integer is extracted on each line of the input
     * stream.  Blank line or line beginning with "#" are skipped.
     *
     * @param in the input file.
     * @param aPosition the position of indices where the integer has to extracted.
     * @return a vector containing the set of integer.
     **/
    
    static std::vector< TNumber >  
    getNumbersFromInputStream (std::istream &in, 
                               unsigned int aPosition);  
  

  }; // end of class NumbersReader



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/NumbersReader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NumbersReader_h

#undef NumbersReader_RECURSES
#endif // else defined(NumbersReader_RECURSES)
