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
 * @file imageBasicSubsampling.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2014/05/21
 *
 * An example file named imageBasicSubsampling.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/io/writers/GenericWriter.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  //! [imageBasicSubsamplingType2D]
  typedef ImageContainerBySTLVector < Z3i::Domain, unsigned char> Image3D;
  typedef ImageContainerBySTLVector < Z2i::Domain, unsigned char> Image2D;
  typedef ConstImageAdapter<Image2D,  Image2D::Domain, 
                            BasicDomainSubSampler<Image2D::Domain>,  
                            Image2D::Value, 
                            DGtal::DefaultFunctor > ConstImageAdapterForZoom;
  typedef ConstImageAdapter<Image3D,  Image3D::Domain, 
                            BasicDomainSubSampler<Image3D::Domain>,  
                            Image3D::Value, 
                            DGtal::DefaultFunctor > ConstImageAdapterForZoom3D;

  //! [imageBasicSubsamplingType]

  trace.beginBlock ( "Example imageBasicSubsampling" );

  std::string imageFilename3D = examplesPath + "samples/Al.100.vol";
  std::string imageFilename2D = examplesPath + "samples/church.pgm";
  Image3D image3D = GenericReader<Image3D>::import( imageFilename3D );
  Image2D image2D = GenericReader<Image2D>::import( imageFilename2D );

  for (unsigned int i=1; i<=20; i*=2){
    std::vector<Z2i::Domain::Size> aGridSize2D;
    aGridSize2D.push_back(i);
    aGridSize2D.push_back(i);
    
    BasicDomainSubSampler<Image2D::Domain> subSampler2D(image2D.domain(), aGridSize2D, Z2i::Point(0 ,0));
    DGtal::DefaultFunctor df;
    Image2D::Domain subSampledDomain2D  = subSampler2D.getSubSampledDomain();
    ConstImageAdapterForZoom imageZoom2D (image2D, subSampledDomain2D, subSampler2D, df);
    stringstream outputname; 
    outputname << "subSampledImage"<< i<< "x"<< i << ".pgm" ;
    GenericWriter<ConstImageAdapterForZoom>::exportFile(outputname.str(), imageZoom2D );
    trace.info() << "Exporting 2D subsampled image by grid size :" << i << "x"<< i<< " in "<< outputname.str() << std::endl;
  } 

  for (unsigned int i=1; i<=20; i*=2){
    std::vector<Z3i::Domain::Size> aGridSize3D;
    aGridSize3D.push_back(i);
    aGridSize3D.push_back(i);
    aGridSize3D.push_back(i);
    
    BasicDomainSubSampler<Image3D::Domain> subSampler3D(image3D.domain(), aGridSize3D, Z3i::Point(0 ,0, 0));
    DGtal::DefaultFunctor df;
    Image3D::Domain subSampledDomain3D  = subSampler3D.getSubSampledDomain();
    ConstImageAdapterForZoom3D imageZoom3D (image3D, subSampledDomain3D, subSampler3D, df);
    stringstream outputname3D; 
    outputname3D << "subSampledImage3D"<< i<< "x"<< i << ".vol" ;
    GenericWriter<ConstImageAdapterForZoom3D>::exportFile(outputname3D.str(), imageZoom3D );
    trace.info() << "Exporting 3D subsampled image by grid size :" << i << "x"<< i<< "x"<< i<< " in "<< outputname3D.str() << std::endl;
  }

  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
