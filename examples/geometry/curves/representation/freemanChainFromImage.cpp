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
 * @file freemanChainDisplay.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/12/01
 * 
 * An example of FreemanChain display with background source image.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////


#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"

#include "DGtal/base/BasicTypes.h"
#include "DGtal/geometry/curves/representation/FreemanChain.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"

#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "ConfigExamples.h"


using namespace std;
using namespace DGtal;
using namespace Z2i;



///////////////////////////////////////////////////////////////////////////////

int main()
{
  
  typedef ImageSelector < Z2i::Domain, int>::Type Image;
  Image image = PNMReader<Image>::importPGM( examplesPath + "samples/circleR10modif.pgm" ); 
  Z2i::KSpace ks;
  if(! ks.init( image.domain().lowerBound(), 
    image.domain().upperBound(), true )){
    cerr << "Problem in initialisation of KSpace" << endl;
    exit(1);
  }
  
  Z2i::DigitalSet set2d (image.domain());
  SetPredicate<Z2i::DigitalSet> set2dPredicate( set2d );
  SetFromImage<Z2i::DigitalSet>::append<Image>(set2d, image, 0, 255);
  SurfelAdjacency<2> sAdj( true );
  std::vector< std::vector< Z2i::Point >  >  vectContoursBdryPointels;
  Surfaces<Z2i::KSpace>::extractAllPointContours4C( vectContoursBdryPointels,
                ks, set2dPredicate, sAdj );  
  Board2D aBoard;
  aBoard << set2d;
  aBoard << image.domain();  

  GradientColorMap<int> cmap_grad( 0, vectContoursBdryPointels.size() );
  cmap_grad.addColor( Color( 50, 50, 255 ) );
  cmap_grad.addColor( Color( 255, 0, 0 ) );
  cmap_grad.addColor( Color( 255, 255, 10 ) );
  cmap_grad.addColor( Color( 25, 255, 255 ) );
  cmap_grad.addColor( Color( 255, 25, 255 ) );
  cmap_grad.addColor( Color( 25, 25, 25 ) );
  
  
  for(unsigned int i=0; i<vectContoursBdryPointels.size(); i++){
    //  Constructing and displaying FreemanChains from contours. 
    FreemanChain<Z2i::Integer> fc (vectContoursBdryPointels.at(i));    
    aBoard << SetMode( fc.className(), "InterGrid" );
    aBoard<< CustomStyle( fc.className(), 
            new CustomColors(  cmap_grad(i),  Color::None ) );    
    
    
    
    aBoard << fc;
  }   
  
  aBoard.saveEPS("freemanChainFromImage.eps");
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
