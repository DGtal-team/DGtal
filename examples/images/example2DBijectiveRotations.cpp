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
* @file CBDR.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly ; stephane.breuils@univ-smb.fr
 * @ingroup Examples
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */
#include <iostream>
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/readers/PPMReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/images/RigidTransformation2D.h"


//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/images/bijectiverotations/QSH.h"
#include "DGtal/images/bijectiverotations/CDLR.h"
#include "DGtal/images/bijectiverotations/RBC.h"
#include "DGtal/images/bijectiverotations/OTC.h"
#include "DGtal/images/bijectiverotations/CBDR.h"
#include "DGtal/images/bijectiverotations/Rotationtables.h"


using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z2i;

std::vector<std::string> supportedBijectiveRotation = {
    "OTC", "CBDR", "CDLR", "QSH" , "RBC"
};

// Generic function to handle rotation based on type
template <typename TImage,typename TBijectiveRotation>
TImage
performRotation( const TImage& img, TBijectiveRotation& obj )
{
  const auto& sbr = supportedBijectiveRotation;
  std::string structName = obj.tostring();
  TImage output( img.domain() );
  if ( std::find( sbr.begin(), sbr.end(), structName ) != sbr.end() )
    {
      trace.beginBlock ( obj.tostring() );
      output = obj.rotateImage( img );
      trace.endBlock ();
    }
  else
    throw std::runtime_error("Unsupported bijective rotation : " + structName);
  return output;
}

void usage( const std::string& cmd )
{
  std::cout << "Usage: " << cmd << " <image> <angle> [<method>] [black|*white*] [detect]" << "\n"
	    << "\t Compute the rotated <image> by an angle <angle> (in degrees) \n"
	    << "\t - <method> in { OTC, CBDR, CDLR, QSH, *RBC* }\n"
	    << std::endl;
}


int main( int argc, char* argv[] )
{
  if ( argc < 3 ) { usage( argv[ 0 ] ); return 1; }
  std::string fname  = argv[ 1 ];
  double      degree = std::atof( argv[ 2 ] );
  double      angle  = degree * M_PI / 180.0;
  std::string method = ( argc > 3 ) ? argv[ 3 ] : "RBC";

  typedef ImageContainerBySTLVector< Domain, Color > Image;
  typedef ForwardRigidTransformation2D < Space > ForwardTrans;
  typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
  typedef MyDomainTransformer::Bounds Bounds;

  Image image  = DGtal::PPMReader<Image>::importPPM ( fname );
  Image output = image;
  int W=image.domain().myUpperBound[0]+1;
  int H=image.domain().myUpperBound[1]+1;
  trace.info() << "Image has size " << W << "x" << H << std::endl;
  Point c(W/2, H/2);

  if ( method == "QSH" )
    {
      DGtal::QSH<Space> rotQSH(angle,c);
      output = performRotation<Image,QSH<Space>> (image,rotQSH);
    }
  else if ( method == "CDLR" )
    {
      auto linf = std::make_shared<DGtal::LinfPolicy<Space,Domain,DGtal::CDLR_naiverotation<Space>>>();
      DGtal::CDLR<DGtal::SpaceND<2, DGtal::int32_t> > rotDSL(angle, c, linf);
      output = performRotation<Image,CDLR<Space>> (image,rotDSL);
    }
  else if ( method == "RBC" )
    {
      DGtal::RBC_vec<Space,RealPoint> rot_rbcvec(2*max(W,H));
      rot_rbcvec.setAngle() = angle;
      rot_rbcvec.center() = c;
      DGtal::RBC<Space,RealPoint> rot_RBC(rot_rbcvec,angle,c);
      output = performRotation<Image,DGtal::RBC<Space,RealPoint>> (image,rot_RBC);
    }
  else if ( method == "CBDR" )
    {
      auto linfCBDR = std::make_shared<DGtal::LinfPolicy<Space,Domain,DGtal::CBDR_naiverotation<Space>>>();
      const int n = 4;
      const int kmax=30;
      DGtal::CBDR<Space,RealPoint> rot_CBDR(angle,c,n,kmax,linfCBDR,true);
      output = performRotation<Image,DGtal::CBDR<Space,RealPoint>> (image,rot_CBDR);
    }

  else if ( method == "OTC" )
    {
      int rwidth = 2;
      std::vector< std::vector< int > > tableOTC = DGtal::functions::loadOTCTable<Space>("../tables/",rwidth);
      DGtal::OTC<Space,RealPoint> rot_OTC( tableOTC, rwidth, c, W, H );
      int alpha = int( round( angle * 180.0 / M_PI ) );
      rot_OTC.set_angle( alpha ); // in degree
      output = performRotation<Image,DGtal::OTC<Space,RealPoint>> (image,rot_OTC);
    }


  std::string out_fname = "rotated-image-" + method + "-" + std::to_string( int(degree) );
  out_fname += ".ppm";
  output >> out_fname;
  return 0;
}