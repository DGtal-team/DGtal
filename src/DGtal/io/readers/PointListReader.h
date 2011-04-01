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
 * @file PointListReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/03/31
 *
 * Header file for module PointListReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PointListReader_RECURSES)
#error Recursive header files inclusion detected in PointListReader.h
#else // defined(PointListReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointListReader_RECURSES

#if !defined PointListReader_h
/** Prevents repeated inclusion of headers. */
#define PointListReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class PointListReader
/**
 * Description of class 'PointListReader' <p> 
 *  \brief Aim: Implements method to read a set of points represented in each line of a file.
 *
 *
 *  The main method to read a set of points as a simple format where
 * each elements is represented in a single line. Blank line or line beginning with
 * "#" are skipped.
 *
 *  
   * Simple example:
   * 
   *  @code
   *  #include "DGtal/helpers/StdDefs.h"
   *  .... 
   *  ifstream infile;
   *  string filename= "testFile.dat"; 
   *  infile.open (filename.c_str(), ifstream::in);
   *  vector<Z2i::Point> vectPoints = PointListReader<Z2i::Point>::getPointsFromInputStream(infile);
   * @endcode 
   * and you can specifying the point position:
   *  @code
   vector<uint> vIndice;
   vIndice.push_back(1); // select for X coordinate the second position number of the line.
   vIndice.push_back(2); // select for Y coordinate the third position number of the line.
   vector<Z2i::Point> vectPoints = PointListReader<Z2i::Point>::getPointsFromInputStream(infile,vectPos);
   *  @endcode
   *   
   */
 *
 */

template <typename TPoint>
 struct  PointListReader
{
 
  typedef TPoint Point;
  
   // ----------------------- Standard services ------------------------------
public:

  
  /** 
   * Main method to import a vector containing a list of points
   * defined in a file where each line defines a point. 
   * 
   * 
   * @param in the input stream to import.
   * @param aVectPosition used to specify the position of indices of points value (default set to 0,..dimension) 
   * @return a DigitalSet containing the in elements.
   **/

  //  static void  getSetFromInputStream(std::istream &in, std::vector<uint> position);
  
  static std::vector<Point>  getPointsFromInputStream (std::istream &in, 
						       std::vector<uint> aVectPosition=std::vector<uint>());
  

  

}; // end of class PointListReader



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/io/readers/PointListReader.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PointListReader_h

#undef PointListReader_RECURSES
#endif // else defined(PointListReader_RECURSES)
