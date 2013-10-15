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

/**
 * @file testDicomReader.cpp
 * @ingroup Tests
 * @author Adrien Krähenbühl (\c adrien.krahenbuhl@loria.fr )
 * LORIA (CNRS, UMR 7503), Université de Lorraine, France
 *
 * @date 2013/10/14
 *
 * Functions for testing class DicomReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/DicomReader.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DicomReader.
///////////////////////////////////////////////////////////////////////////////

template<typename T>
struct HounsfieldToGrayscaleFunctor
{
	int minHounsfieldValue;
	int maxHounsfieldValue;
	HounsfieldToGrayscaleFunctor() : minHounsfieldValue(-3000), maxHounsfieldValue(3000) {}
	HounsfieldToGrayscaleFunctor( const int &min, const int &max ) : minHounsfieldValue(min), maxHounsfieldValue(max) {}
	inline
	T operator() (const T& a) const
	{ return a<=minHounsfieldValue ? 0 : a >= maxHounsfieldValue ? 255 : (a-minHounsfieldValue)*(255./(maxHounsfieldValue-minHounsfieldValue)); }
};


/**
 * Example of a test. To be completed.
 *
 */
bool testDicomReader()
{
  //Default image selector = STLVector
  typedef ImageContainerBySTLVector<DGtal::Z3i::Domain,  int > Image3D;

  //std::string filename = testPath + "samples/cat10.Dicom";
  std::string filename = "../../../Images/DICOM/Premier_billon/partiel/1.2.840.113619.2.55.3.1670609623.220.1201508363.125.1.dcm";
  Image3D image = DicomReader< Image3D, HounsfieldToGrayscaleFunctor<int> >::importDicom( filename, HounsfieldToGrayscaleFunctor<int>(-900,530) );

  trace.info() << image <<endl;

  unsigned int nbval=0;
  for ( Image3D::ConstIterator it=image.begin(), itend=image.end(); it != itend;   ++it)
	if ( (*it) > -100 ) nbval++;

  trace.info() << "Number of points with (val!=0)  = " << nbval << endl;

  return true;
}


bool testIOException()
{
  //Default image selector = STLVector
  typedef ImageContainerBySTLVector<Z3i::Domain,  int > Image3D;

  std::string filename = testPath + "samples/null.Dicom";

  try {
	Image3D image = DicomReader< Image3D, HounsfieldToGrayscaleFunctor<int> >::importDicom( filename, HounsfieldToGrayscaleFunctor<int>(-900,530) );
  }
  catch(exception& e) {
	trace.info() << "Exception catched. Message : " << e.what()<<endl;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DicomReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
	trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDicomReader() && testIOException();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return !res;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
