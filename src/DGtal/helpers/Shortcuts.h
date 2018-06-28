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
 * @file
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2018/06/25
 *
 * Header file for module Shortcuts.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Shortcuts_RECURSES)
#error Recursive header files inclusion detected in Shortcuts.h
#else // defined(Shortcuts_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Shortcuts_RECURSES

#if !defined Shortcuts_h
/** Prevents repeated inclusion of headers. */
#define Shortcuts_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstdlib>
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/IntervalForegroundPredicate.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/volumes/KanungoNoise.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/helpers/Parameters.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  
  /////////////////////////////////////////////////////////////////////////////
  // template class Shortcuts
  /**
   * Description of template class 'Shortcuts' <p> \brief Aim: This
   * class is used to simplify shape and surface creation. With it,
   * you can create new shapes and surface in a few lines. The
   * drawback is that you use specific types or objects, which could
   * lead to faster code or more compact data structures.
   *
   * @tparam TKSpace any cellular grid space, a model of
   * concepts::CCellularGridSpaceND like KhalimskySpaceND.
   */
  template  < typename TKSpace >
  class Shortcuts
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

    // ----------------------- Usual space types --------------------------------------
  public:
    /// Digital cellular space
    typedef TKSpace                                  KSpace;
    /// Digital space
    typedef typename KSpace::Space                   Space;
    /// Integer numbers
    typedef typename Space::Integer                  Integer;
    /// Point with integer coordinates.
    typedef typename Space::Point                    Point;
    /// Vector with integer coordinates.
    typedef typename Space::Vector                   Vector;
    /// Vector with floating-point coordinates.
    typedef typename Space::RealVector               RealVector;
    /// Point with floating-point coordinates.
    typedef typename Space::RealPoint                RealPoint;
    /// Floating-point numbers.
    typedef typename RealVector::Component           Scalar;
    /// An (hyper-)rectangular domain.
    typedef HyperRectDomain<Space>                   Domain;
    /// The type for 8-bits gray-scale elements.
    typedef unsigned char                            GrayScale;

    // ----------------------- Shortcut types --------------------------------------
  public:
    /// defines a multi-variate polynomial : RealPoint -> Scalar
    typedef MPolynomial< Space::dimension, Scalar >      ScalarPolynomial;
    /// defines an implicit shape of the space, which is the
    /// zero-level set of a ScalarPolynomial.
    typedef ImplicitPolynomial3Shape<Space>              ImplicitShape3D;
    /// defines the digitization of an implicit shape.
    typedef GaussDigitizer< Space, ImplicitShape3D >     DigitizedImplicitShape3D;
    /// defines a black and white image with (hyper-)rectangular domain.
    typedef ImageContainerBySTLVector<Domain, bool>      BinaryImage;
    /// defines a grey-level image with (hyper-)rectangular domain.
    typedef ImageContainerBySTLVector<Domain, GrayScale> GrayScaleImage;

    /// defines a set of surfels
    typedef typename KSpace::SurfelSet                   SurfelSet;
    
    /// defines a light container that represents a connected digital
    /// surface over a binary image.
    typedef LightImplicitDigitalSurface< KSpace, BinaryImage >  SimpleSurfaceContainer;
    typedef DigitalSurface< SimpleSurfaceContainer >            SimpleDigitalSurface;
    typedef typename SimpleDigitalSurface::Surfel               Surfel;
    typedef typename SimpleDigitalSurface::Cell                 Cell;
    typedef typename SimpleDigitalSurface::SCell                SCell;
    typedef typename SimpleDigitalSurface::Vertex               Vertex;
    typedef typename SimpleDigitalSurface::Arc                  Arc;
    typedef typename SimpleDigitalSurface::ArcRange             ArcRange;

    // ----------------------- Static services --------------------------------------
  public:

    // ----------------------- General static services ------------------------------
  public:
    
    /// @return the parameters and their default values used in shortcuts.
    /// - "polynomial" : "sphere1"
    static Parameters defaultParameters()
    {
      return parametersImplicitShape3D()
	| parametersKSpace()
	| parametersDigitizedImplicitShape3D()
	| parametersBinaryImage()
	| parametersDigitalSurface();
    }

    // ----------------------- ImplicitShape3D static services ------------------------
  public:

    /// Returns a map associating a name and a polynomial,
    /// e.g. "sphere1", "x^2+y^2+z^2-1".
    ///
    /// { "sphere1", "x^2+y^2+z^2-1" },
    /// { "sphere9", "x^2+y^2+z^2-81" },
    /// { "ellipsoid", "3*x^2+2*y^2+z^2-90" },
    /// { "cylinder", "x^2+2*z^2-90" },
    ///	{ "torus",   "(x^2+y^2+z^2+6*6-2*2)^2-4*6*6*(x^2+y^2)" },
    /// { "rcube",   "x^4+y^4+z^4-6561" },
    /// { "goursat", "-1*(8-0.03*x^4-0.03*y^4-0.03*z^4+2*x^2+2*y^2+2*z^2)" },
    /// { "distel",  "10000-(x^2+y^2+z^2+1000*(x^2+y^2)*(x^2+z^2)*(y^2+z^2))"},
    /// { "leopold", "(x^2*y^2*z^2+4*x^2+4*y^2+3*z^2)-100" },
    /// { "diabolo", "x^2-(y^2+z^2)^2" },
    /// { "heart",   "-1*(x^2+2.25*y^2+z^2-1)^3+x^2*z^3+0.1125*y^2*z^3" },
    /// { "crixxi",  "-0.9*(y^2+z^2-1)^2-(x^2+y^2-1)^3" }
    ///
    /// @return the map associating a polynomial to a name.
    static std::map< std::string, std::string >
    getPolynomialList()
    {
      std::vector< std::pair< std::string, std::string > >
	Ps = { { "sphere1", "x^2+y^2+z^2-1" },
	       { "sphere9", "x^2+y^2+z^2-81" },
	       { "ellipsoid", "3*x^2+2*y^2+z^2-90" },
	       { "cylinder", "x^2+2*z^2-90" },
	       { "torus",   "(x^2+y^2+z^2+6*6-2*2)^2-4*6*6*(x^2+y^2)" },
	       { "rcube",   "x^4+y^4+z^4-6561" },
	       { "goursat", "-1*(8-0.03*x^4-0.03*y^4-0.03*z^4+2*x^2+2*y^2+2*z^2)" },
	       { "distel",  "10000-(x^2+y^2+z^2+1000*(x^2+y^2)*(x^2+z^2)*(y^2+z^2))"},
	       { "leopold", "(x^2*y^2*z^2+4*x^2+4*y^2+3*z^2)-100" },
	       { "diabolo", "x^2-(y^2+z^2)^2" },
	       { "heart",   "-1*(x^2+2.25*y^2+z^2-1)^3+x^2*z^3+0.1125*y^2*z^3" },
	       { "crixxi",  "-0.9*(y^2+z^2-1)^2-(x^2+y^2-1)^3" } };
      std::map< std::string, std::string > L;
      for ( auto p : Ps )
	L[ p.first ] = p.second;
      return L;
    }

    /// @return the parameters and their default values which are used
    /// to define an implicit shape.
    ///   - polynomial["sphere1"]: the implicit polynomial whose zero-level set
    ///                            defines the shape of interest.
    static Parameters parametersImplicitShape3D()
    {
      return Parameters( "polynomial", "sphere1" );
    }

    /// Builds a 3D implicit shape from parameters
    ///
    /// @param[in] params the parameters:
    ///   - polynomial["sphere1"]: the implicit polynomial whose zero-level set
    ///                            defines the shape of interest.
    ///
    /// @return a smart pointer on the created implicit shape.
    static CountedPtr<ImplicitShape3D>
    makeImplicitShape3D( const Parameters& params = parametersImplicitShape3D() )
    {
      typedef MPolynomialReader< Space::dimension, Scalar> Polynomial3Reader;
      std::string poly_str = params[ "polynomial" ].as<std::string>();
      // Recognizes specific strings as polynomials.
      auto PL = getPolynomialList();
      if ( PL[ poly_str ] != "" ) poly_str = PL[ poly_str ];
      ScalarPolynomial poly;
      Polynomial3Reader reader;
      std::string::const_iterator iter
	= reader.read( poly, poly_str.begin(), poly_str.end() );
      if ( iter != poly_str.end() )
	{
	  trace.error() << "[Shortcuts::makeImplicitShape3D]"
			<< " ERROR reading polynomial: I read only <"
			<< poly_str.substr( 0, iter - poly_str.begin() )
			<< ">, and I built P=" << poly << std::endl;
	}
      return CountedPtr<ImplicitShape3D>( new ImplicitShape3D( poly ) );
    }

    // ----------------------- KSpace static services ------------------------------
  public:
    
    /// @return the parameters and their default values which are used for digitization.
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    static Parameters parametersKSpace()
    {
      return Parameters
	( "closed",  1 );
    }

    /// Builds a Khalimsky space that encompasses the lower and upper
    /// digital points.  Note that digital points are cells of the
    /// Khalimsky space with maximal dimensions.  A closed Khalimsky
    /// space adds lower dimensional cells all around its boundary to
    /// define a closed complex.
    ///
    /// @param[in] params the parameters:
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    static KSpace getKSpace( const Point& low, const Point& up,
			      Parameters params = parametersKSpace() )
    {
      int closed  = params[ "closed"  ].as<int>();
      KSpace K;
      if ( ! K.init( low, up, closed ) )
	trace.error() << "[Shortcuts::getKSpace]"
		      << " Error building Khalimsky space K=" << K << std::endl;
      return K;
    }

    /// Builds a Khalimsky space that encompasses the domain of the given image.
    /// Note that digital points are cells of the Khalimsky space with
    /// maximal dimensions.  A closed Khalimsky space adds lower
    /// dimensional cells all around its boundary to define a closed
    /// complex.
    ///
    /// @param[in] params the parameters:
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    static KSpace getKSpace( CountedPtr<BinaryImage> bimage,
			     Parameters params = parametersKSpace() )
    {
      int closed  = params[ "closed"  ].as<int>();
      KSpace K;
      if ( ! K.init( bimage->domain().lowerBound(),
		     bimage->domain().upperBound(),
		     closed ) )
	trace.error() << "[Shortcuts::getKSpace]"
		      << " Error building Khalimsky space K=" << K << std::endl;
      return K;
    }

    // ----------------------- DigitizedImplicitShape3D static services --------------
  public:
    
    /// @return the parameters and their default values which are used for digitization.
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    static Parameters parametersDigitizedImplicitShape3D()
    {
      return Parameters
	( "minAABB",  -10.0 )
	( "maxAABB",   10.0 )
	( "gridstep",   1.0 )
	( "offset",     5.0 );
    }

    
    /// Builds a Khalimsky space that encompasses the bounding box
    /// specified by \a params. It is for instance useful when
    /// digitizing an implicit shape.
    ///
    /// @param[in] params the parameters:
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    ///   - closed   [1]    : specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    /// @see makeDigitizedImplicitShape3D
    static KSpace getKSpaceDigitizedImplicitShape3D( Parameters params =
					  parametersKSpace() | parametersDigitizedImplicitShape3D() )
    {
      Scalar min_x  = params[ "minAABB"  ].as<Scalar>();
      Scalar max_x  = params[ "maxAABB"  ].as<Scalar>();
      Scalar h      = params[ "gridstep" ].as<Scalar>();
      Scalar offset = params[ "offset"   ].as<Scalar>();
      bool   closed = params[ "closed"   ].as<int>();
      RealPoint p1( min_x - offset * h, min_x - offset * h, min_x - offset * h );
      RealPoint p2( max_x + offset * h, max_x + offset * h, max_x + offset * h );
      CountedPtr<DigitizedImplicitShape3D> dshape( new DigitizedImplicitShape3D() );
      dshape->init( p1, p2, h );
      Domain domain = dshape->getDomain();
      KSpace K;
      if ( ! K.init( domain.lowerBound(), domain.upperBound(), closed ) )
	trace.error() << "[Shortcuts::getKSpace]"
		      << " Error building Khalimsky space K=" << K << std::endl;
      return K;
    }

    /// Makes the Gauss digitization of the given implicit shape
    /// according to parameters. Use getKSpace to build the associated digital space.
    ///
    /// @param[in] shape a smart pointer on the implicit shape.
    /// @param[in] params the parameters:
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    ///
    /// @return a smart pointer on the created implicit digital shape.
    /// @see getKSpaceDigitizedImplicitShape3D 
    static CountedPtr<DigitizedImplicitShape3D>
    makeDigitizedImplicitShape3D
    ( CountedPtr<ImplicitShape3D> shape,
      Parameters params = parametersDigitizedImplicitShape3D() )
    {
      Scalar min_x  = params[ "minAABB"  ].as<Scalar>();
      Scalar max_x  = params[ "maxAABB"  ].as<Scalar>();
      Scalar h      = params[ "gridstep" ].as<Scalar>();
      Scalar offset = params[ "offset"   ].as<Scalar>();
      RealPoint p1( min_x - offset * h, min_x - offset * h, min_x - offset * h );
      RealPoint p2( max_x + offset * h, max_x + offset * h, max_x + offset * h );
      CountedPtr<DigitizedImplicitShape3D> dshape( new DigitizedImplicitShape3D() );
      dshape->attach( shape );
      dshape->init( p1, p2, h );
      return dshape;
    }

    // ----------------------- BinaryImage static services --------------------------
  public:
    
    /// @return the parameters and their default values which are
    /// related to binary images and synthetic noise.
    ///   - noise        [0.0]: specifies the Kanungo noise level for binary pictures.
    ///   - thresholdMin [  0]: specifies the threshold min (excluded) to define binary shape
    ///   - thresholdMax [255]: specifies the threshold max (included) to define binary shape
    static Parameters parametersBinaryImage()
    {
      return Parameters
	( "noise", 0.0 )
	( "thresholdMin", 0 )
	( "thresholdMax", 255 );
    }
    
    /// Makes an empty binary image within a given domain.
    ///
    /// @param[in] domain any domain.
    ///
    /// @return a smart pointer on a binary image that fits the given domain.
    static CountedPtr<BinaryImage>
    makeBinaryImage( Domain shapeDomain )
    {
      return CountedPtr<BinaryImage>( new BinaryImage( shapeDomain ) );
    }
    
    /// Vectorizes an implicitly defined digital shape into a binary
    /// image, and possibly add Kanungo noise to the result depending
    /// on parameters given in \a params.
    ///
    /// @param[in] shape_digitization a smart pointer on an implicit digital shape.
    /// @param[in] params the parameters:
    ///   - noise   [0.0]: specifies the Kanungo noise level for binary pictures.
    ///
    /// @return a smart pointer on a binary image that samples the digital shape.
    static CountedPtr<BinaryImage>
    makeBinaryImage( CountedPtr<DigitizedImplicitShape3D> shape_digitization,
		     Parameters params = parametersBinaryImage() )
    {
      return makeBinaryImage( shape_digitization,
			      shape_digitization->getDomain(),
			      params );
    }
    
    /// Vectorizes an implicitly defined digital shape into a binary
    /// image, in the specified (hyper-)rectangular domain, and
    /// possibly add Kanungo noise to the result depending on
    /// parameters given in \a params.
    ///
    /// @param[in] shape_digitization a smart pointer on an implicit digital shape.
    /// @param[in] domain any domain.
    /// @param[in] params the parameters:
    ///   - noise   [0.0]: specifies the Kanungo noise level for binary pictures.
    ///
    /// @return a smart pointer on a binary image that samples the digital shape.
    static CountedPtr<BinaryImage>
    makeBinaryImage( CountedPtr<DigitizedImplicitShape3D> shape_digitization,
		     Domain shapeDomain,
		     Parameters params = parametersBinaryImage() )
    {
      const Scalar noise        = params[ "noise"  ].as<Scalar>();
      CountedPtr<BinaryImage> img ( new BinaryImage( shapeDomain ) );
      if ( noise <= 0.0 )
	{
	  std::transform( shapeDomain.begin(), shapeDomain.end(),
			  img->begin(),
			  [&shape_digitization]
			  ( const Point& p ) { return (*shape_digitization)(p); } );
	}
      else
	{
	  typedef KanungoNoise< DigitizedImplicitShape3D, Domain > KanungoPredicate;
	  KanungoPredicate noisy_dshape( *shape_digitization, shapeDomain, noise );
	  std::transform( shapeDomain.begin(), shapeDomain.end(),
			  img->begin(),
			  [&noisy_dshape] ( const Point& p ) { return noisy_dshape(p); } );
	}
      return img;
    }

    /// Adds Kanungo noise to a binary image and returns the resulting new image. 
    ///
    /// @param[in] bimage a smart pointer on a binary image.
    /// @param[in] params the parameters:
    ///   - noise   [0.0]: specifies the Kanungo noise level for binary pictures.
    ///
    /// @return a smart pointer on the noisified binary image.
    static CountedPtr<BinaryImage>
    makeBinaryImage( CountedPtr<BinaryImage> bimage,
		     Parameters params = parametersBinaryImage() )
    {
      const Scalar noise = params[ "noise"  ].as<Scalar>();
      if ( noise <= 0.0 ) return bimage;
      typedef KanungoNoise< BinaryImage, Domain > KanungoPredicate;
      const Domain shapeDomain    = bimage->domain();
      CountedPtr<BinaryImage> img ( new BinaryImage( shapeDomain ) );
      KanungoPredicate noisy_dshape( *bimage, shapeDomain, noise );
      std::transform( shapeDomain.begin(), shapeDomain.end(),
		      img->begin(),
		      [&noisy_dshape] ( const Point& p ) { return noisy_dshape(p); } );
      return img;
    }

    /// Loads an arbitrary image file (e.g. vol file in 3D) and returns
    /// the binary image corresponding to the threshold/noise parameters.
    ///
    /// @param[in] input the input filename.
    /// @param[in] params the parameters:
    ///   - noise        [0.0]: specifies the Kanungo noise level for binary pictures.
    ///   - thresholdMin [  0]: specifies the threshold min (excluded) to define binary shape
    ///   - thresholdMax [255]: specifies the threshold max (included) to define binary shape
    ///
    /// @return a smart pointer on a binary image that represents the
    /// (thresholded/noisified) image file.
    static CountedPtr<BinaryImage>
    makeBinaryImage
    ( std::string input,
      Parameters params = parametersBinaryImage() )
    {
      int     thresholdMin = params["thresholdMin"].as<int>();
      int     thresholdMax = params["thresholdMax"].as<int>();
      GrayScaleImage image = GenericReader<GrayScaleImage>::import( input );
      Domain        domain = image.domain();
      typedef functors::IntervalForegroundPredicate<GrayScaleImage> ThresholdedImage;
      ThresholdedImage tImage( image, thresholdMin, thresholdMax );
      CountedPtr<BinaryImage> img ( new BinaryImage( domain ) );
      std::transform( domain.begin(), domain.end(),
		      img->begin(),
		      [tImage] ( const Point& p ) { return tImage(p); } );
      return makeBinaryImage( img, params );
    }

    /// Binarizes an arbitrary gray scale image file and returns
    /// the binary image corresponding to the threshold/noise parameters.
    ///
    /// @param[in] gray_scale_image the input gray scale image.
    /// @param[in] params the parameters:
    ///   - noise        [0.0]: specifies the Kanungo noise level for binary pictures.
    ///   - thresholdMin [  0]: specifies the threshold min (excluded) to define binary shape
    ///   - thresholdMax [255]: specifies the threshold max (included) to define binary shape
    ///
    /// @return a smart pointer on a binary image that represents the
    /// (thresholded/noisified) gray scale image.
    static CountedPtr<BinaryImage>
    makeBinaryImage
    ( CountedPtr<GrayScaleImage> gray_scale_image,
      Parameters params = parametersBinaryImage() )
    {
      int     thresholdMin = params["thresholdMin"].as<int>();
      int     thresholdMax = params["thresholdMax"].as<int>();
      Domain        domain = gray_scale_image->domain();
      typedef functors::IntervalForegroundPredicate<GrayScaleImage> ThresholdedImage;
      ThresholdedImage tImage( *gray_scale_image, thresholdMin, thresholdMax );
      CountedPtr<BinaryImage> img ( new BinaryImage( domain ) );
      std::transform( domain.begin(), domain.end(),
		      img->begin(),
		      [tImage] ( const Point& p ) { return tImage(p); } );
      return makeBinaryImage( img, params );
    }

    
    /// Saves an arbitrary image file (e.g. vol file in 3D).
    ///
    /// @param[in] output the output filename .
    /// @return 'true' if everything went well, 'false' if there was an error during save.
    static bool
    saveBinaryImage
    ( CountedPtr<BinaryImage> bimage, std::string output )
    {
      auto gray_scale_image = makeGrayScaleImage( bimage );
      return saveGrayScaleImage( gray_scale_image, output );
    }


    // ----------------------- GrayScaleImage static services -------------------------
  public:
    
    /// Makes an empty gray scale image within a given domain (values are unsigned char).
    ///
    /// @param[in] domain any domain.
    ///
    /// @return a smart pointer on a gray scale image that fits the given domain.
    static CountedPtr<GrayScaleImage>
    makeGrayScaleImage( Domain shapeDomain )
    {
      return CountedPtr<GrayScaleImage>( new BinaryImage( shapeDomain ) );
    }

    /// Loads an arbitrary binary image file (e.g. vol file in 3D) and returns
    /// the corresponding gray-scale image.
    ///
    /// @param[in] input the input filename.
    ///
    /// @return a smart pointer on the loaded gray-scale image.
    static CountedPtr<GrayScaleImage>
    makeGrayScaleImage
    ( std::string input )
    {
      GrayScaleImage image = GenericReader<GrayScaleImage>::import( input );
      return CountedPtr<GrayScaleImage>( new GrayScaleImage( image ) );
    }

    /// Makes a gray-scale image from a binary image using the given transformation
    ///
    /// @param[in] binary_image the input binary image.
    ///
    /// @return a smart pointer on the resulting gray-scale image.
    static CountedPtr<GrayScaleImage>
    makeGrayScaleImage
    ( CountedPtr<BinaryImage> binary_image,
      std::function< GrayScale( bool ) > const & bool2grayscale
      = [] ( bool v ) { return v ? (GrayScale) 255 : (GrayScale) 0; } )
    {
      const Domain domain = binary_image->domain(); 
      CountedPtr<GrayScaleImage> gray_scale_image( new GrayScaleImage( domain ) );
      std::transform( binary_image->begin(), binary_image->end(),
		      gray_scale_image->begin(),
		      bool2grayscale );
      return gray_scale_image;
    }

    /// Saves an arbitrary gray-scale image file (e.g. vol file in 3D).
    ///
    /// @param[in] output the output filename .
    /// @return 'true' if everything went well, 'false' if there was an error during save.
    static bool
    saveGrayScaleImage
    ( CountedPtr<GrayScaleImage> gray_scale_image, std::string output )
    {
      return GenericWriter< GrayScaleImage > 
	::exportFile( output, *gray_scale_image );
    }

    /// @return the parameters and their default values which are
    /// related to digital surfaces.
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel [100000]: number of tries in method Surfaces::findABel
    static Parameters parametersDigitalSurface()
    {
      return Parameters
	( "surfelAdjacency", 1 )
	( "nbTriesToFindABel", 100000 );
    }

    /// Builds a simple digital surface from a space \a K and a binary image \a bimage.
    ///
    /// @param[in] bimage a binary image representing the characteristic function of a digital shape.
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    ///
    /// @return a smart pointer on a (light) digital surface that
    /// represents the boundary of the digital shape (at least a big
    /// component).
    static CountedPtr<SimpleDigitalSurface>
    makeSimpleDigitalSurface
    ( CountedPtr<BinaryImage> bimage,
      const KSpace& K,
      Parameters params = parametersDigitalSurface() )
    {
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      int nb_tries_to_find_a_bel = params[ "nbTriesToFindABel" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );

      // We have to search for a surfel that belong to a big connected component.
      CountedPtr<SimpleDigitalSurface> ptrSurface;
      Surfel       bel;
      Scalar       minsize    = bimage->extent().norm();
      unsigned int nb_surfels = 0;
      unsigned int tries      = 0;
      do {
        try { // Search initial bel
          bel = Surfaces<KSpace>::findABel( K, *bimage, nb_tries_to_find_a_bel );
        } catch (DGtal::InputException e) {
          trace.error() << "[Shortcuts::makeSimpleDigitalSurface]"
			<< " ERROR Unable to find bel." << std::endl;
          return ptrSurface;
        }
	// this pointer will be acquired by the surface.
        SimpleSurfaceContainer* surfContainer
	  = new SimpleSurfaceContainer( K, *bimage, surfAdj, bel );
        ptrSurface = CountedPtr<SimpleDigitalSurface>
	  ( new SimpleDigitalSurface( surfContainer ) ); // acquired
        nb_surfels = ptrSurface->size();
      } while ( ( nb_surfels < 2 * minsize ) && ( tries++ < 150 ) );
      if( tries >= 150 ) {
	trace.warning() << "[Shortcuts::makeSimpleDigitalSurface]"
			<< "ERROR cannot find a proper bel in a big enough component."
			<< std::endl;
      }
      return ptrSurface;
    }

    /// Returns a vector containing all the simple digital surface in
    /// the binary image \a bimage.
    ///
    /// @param[in] bimage a binary image representing the
    /// characteristic function of a digital shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the
    /// digital shape.
    ///
    /// @return a vector of smart pointers to the connected (light)
    /// digital surfaces present in the binary image.
    static std::vector< CountedPtr<SimpleDigitalSurface> >
    getSimpleDigitalSurfaces
    ( CountedPtr<BinaryImage> bimage,
      const KSpace& K,
      Parameters params = parametersDigitalSurface() )
    {
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      int nb_tries_to_find_a_bel = params[ "nbTriesToFindABel" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );
      // Extracts all boundary surfels
      SurfelSet all_surfels;
      Surfaces<KSpace>::sMakeBoundary( all_surfels, K, *bimage,
				       K.lowerBound(), K.upperBound() );
      // Builds all connected components of surfels.
      SurfelSet marked_surfels;
      std::vector< CountedPtr<SimpleDigitalSurface> > result;
      CountedPtr<SimpleDigitalSurface> ptrSurface;
      for ( auto bel : all_surfels )
	{
	  if ( marked_surfels.count( bel ) != 0 ) continue;
	  SimpleSurfaceContainer* surfContainer
	    = new SimpleSurfaceContainer( K, *bimage, surfAdj, bel );
	  ptrSurface = CountedPtr<SimpleDigitalSurface>
	    ( new SimpleDigitalSurface( surfContainer ) ); // acquired
	  // mark all surfels of the surface component.
	  marked_surfels.insert( ptrSurface->begin(), ptrSurface->end() );
	  // add surface component to result.
	  result.push_back( ptrSurface );
	}
      return result;
    }

    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     */
    Shortcuts() = delete;

    /**
     * Destructor.
     */
    ~Shortcuts() = delete;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Shortcuts ( const Shortcuts & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    Shortcuts ( Shortcuts && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Shortcuts & operator= ( const Shortcuts & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    Shortcuts & operator= ( Shortcuts && other ) = delete;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Shortcuts


  /**
   * Overloads 'operator<<' for displaying objects of class 'Shortcuts'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Shortcuts' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const Shortcuts<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/helpers/Shortcuts.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Shortcuts_h

#undef Shortcuts_RECURSES
#endif // else defined(Shortcuts_RECURSES)
