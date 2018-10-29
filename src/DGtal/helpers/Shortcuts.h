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
#include <sstream>
#include <iterator>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/RegularPointEmbedder.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/math/Statistic.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/IntervalForegroundPredicate.h"
#include <DGtal/images/ImageLinearCellEmbedder.h>
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/ShapeGeometricFunctors.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/IndexedDigitalSurface.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/CCellEmbedder.h"
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/volumes/KanungoNoise.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/surfaces/estimation/TrueDigitalSurfaceLocalEstimator.h"
#include "DGtal/geometry/surfaces/estimation/VoronoiCovarianceMeasureOnDigitalSurface.h"
#include "DGtal/geometry/surfaces/estimation/VCMDigitalSurfaceLocalEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IIGeometricFunctors.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantVolumeEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantCovarianceEstimator.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/graph/BreadthFirstVisitor.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"
#include "DGtal/helpers/Parameters.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace sgf = DGtal::functors::ShapeGeometricFunctors;
  
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
    typedef LightImplicitDigitalSurface< KSpace, BinaryImage >  LightSurfaceContainer;
    /// defines a connected digital surface over a binary image.
    typedef DigitalSurface< LightSurfaceContainer >             LightDigitalSurface;
    /// defines a heavy container that represents any digital surface.
    typedef SetOfSurfels< KSpace, SurfelSet >                   ExplicitSurfaceContainer;
    /// defines an arbitrary digital surface over a binary image.
    typedef DigitalSurface< ExplicitSurfaceContainer >          DigitalSurface;
    /// defines a connected or not indexed digital surface.
    typedef IndexedDigitalSurface< ExplicitSurfaceContainer >   IdxDigitalSurface;
    typedef typename LightDigitalSurface::Surfel                Surfel;
    typedef typename LightDigitalSurface::Cell                  Cell;
    typedef typename LightDigitalSurface::SCell                 SCell;
    typedef typename LightDigitalSurface::Vertex                Vertex;
    typedef typename LightDigitalSurface::Arc                   Arc;
    typedef typename LightDigitalSurface::Face                  Face;
    typedef typename LightDigitalSurface::ArcRange              ArcRange;
    typedef typename IdxDigitalSurface::Vertex                  IdxSurfel;
    typedef typename IdxDigitalSurface::Vertex                  IdxVertex;
    typedef typename IdxDigitalSurface::Arc                     IdxArc;
    typedef typename IdxDigitalSurface::ArcRange                IdxArcRange;
    typedef std::set< IdxSurfel >                               IdxSurfelSet;
    typedef std::vector< Surfel >                               SurfelRange;
    typedef std::vector< Cell >                                 CellRange;
    typedef std::vector< IdxSurfel >                            IdxSurfelRange;
    typedef std::vector< Scalar >                               Scalars;
    typedef std::vector< RealVector >                           RealVectors;

    typedef DGtal::Statistic<Scalar>                            ScalarStatistic;
    
    typedef sgf::ShapePositionFunctor<ImplicitShape3D>          PositionFunctor;
    typedef sgf::ShapeNormalVectorFunctor<ImplicitShape3D>      NormalFunctor;
    typedef sgf::ShapeMeanCurvatureFunctor<ImplicitShape3D>     MeanCurvatureFunctor;
    typedef sgf::ShapeGaussianCurvatureFunctor<ImplicitShape3D> GaussianCurvatureFunctor;
    typedef TrueDigitalSurfaceLocalEstimator
    < KSpace, ImplicitShape3D, PositionFunctor >                TruePositionEstimator;
    typedef TrueDigitalSurfaceLocalEstimator
    < KSpace, ImplicitShape3D, NormalFunctor >                  TrueNormalEstimator;
    typedef TrueDigitalSurfaceLocalEstimator
    < KSpace, ImplicitShape3D, MeanCurvatureFunctor >           TrueMeanCurvatureEstimator;
    typedef TrueDigitalSurfaceLocalEstimator
    < KSpace, ImplicitShape3D, GaussianCurvatureFunctor >       TrueGaussianCurvatureEstimator;

    typedef RegularPointEmbedder<Space>                         PointEmbedder;
    typedef ImageLinearCellEmbedder
    < KSpace, GrayScaleImage, PointEmbedder >                   ImageCellEmbedder;
    typedef Mesh<RealPoint>                                     Mesh;
    typedef TriangulatedSurface<RealPoint>                      TriangulatedSurface;
    typedef PolygonalSurface<RealPoint>                         PolygonalSurface;
    typedef std::map<Surfel, IdxSurfel>                         Surfel2Index;
    
    // ----------------------- Static services --------------------------------------
  public:

    // ----------------------- General static services ------------------------------
  public:
    
    /// @return the parameters and their default values used in shortcuts.
    static Parameters defaultParameters()
    {
      return parametersImplicitShape3D()
	| parametersKSpace()
	| parametersDigitizedImplicitShape3D()
	| parametersBinaryImage()
	| parametersDigitalSurface()
	| parametersGeometryEstimation();
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
    ///   - polynomial     ["sphere1"]: the implicit polynomial whose zero-level set
    ///                                 defines the shape of interest.
    ///   - projectionMaxIter [    20]: the maximum number of iteration for the projection.
    ///   - projectionAccuracy[0.0001]: the zero-proximity stop criterion during projection.
    ///   - projectionGamma   [   0.5]: the damping coefficient of the projection.
    static Parameters parametersImplicitShape3D()
    {
      return Parameters
	( "polynomial", "sphere1" )
	( "projectionMaxIter", 20 )
	( "projectionAccuracy", 0.0001 )
	( "projectionGamma",    0.5 );
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

    /// Given a space \a K, an implicit \a shape, a sequence of \a
    /// surfels, and a gridstep \a h, returns the closest positions on
    /// the surface at the specified surfels, in the same order.
    ///
    /// @note The surfel centroids are iteratively projected onto the
    /// implicit surface through a damped Newton process.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    /// @param[in] shape the implicit shape.
    /// @param[in] h the grid step to embed surfels.
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    ///
    /// @param[in] params the parameters:
    ///   - gridstep          [   1.0]: the digitization gridstep (often denoted by h).
    ///   - projectionMaxIter [    20]: the maximum number of iteration for the projection.
    ///   - projectionAccuracy[0.0001]: the zero-proximity stop criterion during projection.
    ///   - projectionGamma   [   0.5]: the damping coefficient of the projection.
    ///
    /// @return the vector containing the true normals, in the same
    /// order as \a surfels.
    static RealVectors
    getPositions
    ( CountedPtr<ImplicitShape3D> shape,
      const KSpace&               K,
      const SurfelRange&          surfels,
      const Parameters&           params = parametersImplicitShape3D() )
    {
      RealVectors         n_true_estimations;
      TruePositionEstimator true_estimator;
      int     maxIter = params[ "projectionMaxIter"  ].as<int>();
      double accuracy = params[ "projectionAccuracy" ].as<double>();
      double    gamma = params[ "projectionGamma"    ].as<double>();
      double gridstep = params[ "gridstep"           ].as<double>();
      true_estimator.attach( *shape );
      true_estimator.setParams( K, PositionFunctor(), maxIter, accuracy, gamma );
      true_estimator.init( gridstep, surfels.begin(), surfels.end() );
      true_estimator.eval( surfels.begin(), surfels.end(),
			   std::back_inserter( n_true_estimations ) );
      return n_true_estimations;
    }
    
    /// Given a space \a K, an implicit \a shape, a sequence of \a
    /// surfels, and a gridstep \a h, returns the normal vectors at the
    /// specified surfels, in the same order.
    ///
    /// @note that the normal vector is approximated by projecting the
    /// surfel centroid onto the implicit 3D shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    /// @param[in] shape the implicit shape.
    /// @param[in] h the grid step to embed surfels.
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    ///
    /// @param[in] params the parameters:
    ///   - gridstep          [   1.0]: the digitization gridstep (often denoted by h).
    ///   - projectionMaxIter [    20]: the maximum number of iteration for the projection.
    ///   - projectionAccuracy[0.0001]: the zero-proximity stop criterion during projection.
    ///   - projectionGamma   [   0.5]: the damping coefficient of the projection.
    ///
    /// @return the vector containing the true normals, in the same
    /// order as \a surfels.
    static RealVectors
    getNormalVectors
    ( CountedPtr<ImplicitShape3D> shape,
      const KSpace&               K,
      const SurfelRange&          surfels,
      const Parameters&           params = parametersImplicitShape3D() )
    {
      RealVectors         n_true_estimations;
      TrueNormalEstimator true_estimator;
      int     maxIter = params[ "projectionMaxIter"  ].as<int>();
      double accuracy = params[ "projectionAccuracy" ].as<double>();
      double    gamma = params[ "projectionGamma"    ].as<double>();
      double gridstep = params[ "gridstep"           ].as<double>();
      true_estimator.attach( *shape );
      true_estimator.setParams( K, NormalFunctor(), maxIter, accuracy, gamma );
      true_estimator.init( gridstep, surfels.begin(), surfels.end() );
      true_estimator.eval( surfels.begin(), surfels.end(),
			   std::back_inserter( n_true_estimations ) );
      return n_true_estimations;
    }
    
    /// Given a space \a K, an implicit \a shape, a sequence of \a
    /// surfels, and a gridstep \a h, returns the mean curvatures at the
    /// specified surfels, in the same order.
    ///
    /// @note that the mean curvature is approximated by projecting the
    /// surfel centroid onto the implicit 3D shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    /// @param[in] shape the implicit shape.
    /// @param[in] h the grid step to embed surfels.
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    ///
    /// @param[in] params the parameters:
    ///   - gridstep          [   1.0]: the digitization gridstep (often denoted by h).
    ///   - projectionMaxIter [    20]: the maximum number of iteration for the projection.
    ///   - projectionAccuracy[0.0001]: the zero-proximity stop criterion during projection.
    ///   - projectionGamma   [   0.5]: the damping coefficient of the projection.
    ///
    /// @return the vector containing the mean curvatures, in the same
    /// order as \a surfels.
    static Scalars
    getMeanCurvatures
    ( CountedPtr<ImplicitShape3D> shape,
      const KSpace&               K,
      const SurfelRange&          surfels,
      const Parameters&           params = parametersImplicitShape3D() )
    {
      Scalars                n_true_estimations;
      TrueMeanCurvatureEstimator true_estimator;
      int     maxIter = params[ "projectionMaxIter"  ].as<int>();
      double accuracy = params[ "projectionAccuracy" ].as<double>();
      double    gamma = params[ "projectionGamma"    ].as<double>();
      double gridstep = params[ "gridstep"           ].as<double>();
      true_estimator.attach( *shape );
      true_estimator.setParams( K, MeanCurvatureFunctor(), maxIter, accuracy, gamma );
      true_estimator.init( gridstep, surfels.begin(), surfels.end() );
      true_estimator.eval( surfels.begin(), surfels.end(),
			   std::back_inserter( n_true_estimations ) );
      return n_true_estimations;
    }
    
    /// Given a space \a K, an implicit \a shape, a sequence of \a
    /// surfels, and a gridstep \a h, returns the gaussian curvatures at the
    /// specified surfels, in the same order.
    ///
    /// @note that the gaussian curvature is approximated by projecting the
    /// surfel centroid onto the implicit 3D shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    /// @param[in] shape the implicit shape.
    /// @param[in] h the grid step to embed surfels.
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    ///
    /// @param[in] params the parameters:
    ///   - gridstep          [   1.0]: the digitization gridstep (often denoted by h).
    ///   - projectionMaxIter [    20]: the maximum number of iteration for the projection.
    ///   - projectionAccuracy[0.0001]: the zero-proximity stop criterion during projection.
    ///   - projectionGamma   [   0.5]: the damping coefficient of the projection.
    ///
    /// @return the vector containing the gaussian curvatures, in the same
    /// order as \a surfels.
    static Scalars
    getGaussianCurvatures
    ( CountedPtr<ImplicitShape3D> shape,
      const KSpace&               K,
      const SurfelRange&          surfels,
      const Parameters&           params = parametersImplicitShape3D() )
    {
      Scalars                n_true_estimations;
      TrueGaussianCurvatureEstimator true_estimator;
      int     maxIter = params[ "projectionMaxIter"  ].as<int>();
      double accuracy = params[ "projectionAccuracy" ].as<double>();
      double    gamma = params[ "projectionGamma"    ].as<double>();
      double gridstep = params[ "gridstep"           ].as<double>();
      true_estimator.attach( *shape );
      true_estimator.setParams( K, GaussianCurvatureFunctor(), maxIter, accuracy, gamma );
      true_estimator.init( gridstep, surfels.begin(), surfels.end() );
      true_estimator.eval( surfels.begin(), surfels.end(),
			   std::back_inserter( n_true_estimations ) );
      return n_true_estimations;
    }
    
    
    // ----------------------- KSpace static services ------------------------------
  public:
    
    /// @return the parameters and their default values which are used for digitization.
    ///   - closed   [1  ]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///   - gridsizex[1.0]: specifies the space between points along x.
    ///   - gridsizey[1.0]: specifies the space between points along y.
    ///   - gridsizez[1.0]: specifies the space between points along z.
    static Parameters parametersKSpace()
    {
      return Parameters
	( "closed",    1   )
	( "gridsizex", 1.0 )
	( "gridsizey", 1.0 )
	( "gridsizez", 1.0 );
    }

    /// Builds a Khalimsky space that encompasses the lower and upper
    /// digital points.  Note that digital points are cells of the
    /// Khalimsky space with maximal dimensions.  A closed Khalimsky
    /// space adds lower dimensional cells all around its boundary to
    /// define a closed complex.
    ///
    /// @param[in] low the lowest point in the space
    /// @param[in] up the highest point in the space
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
    /// @param[in] bimage any binary image
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

    /// Builds a Khalimsky space that encompasses the domain of the given image.
    /// Note that digital points are cells of the Khalimsky space with
    /// maximal dimensions.  A closed Khalimsky space adds lower
    /// dimensional cells all around its boundary to define a closed
    /// complex.
    ///
    /// @param[in] gimage any gray-scale image
    /// @param[in] params the parameters:
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    static KSpace getKSpace( CountedPtr<GrayScaleImage> gimage,
			     Parameters params = parametersKSpace() )
    {
      int closed  = params[ "closed"  ].as<int>();
      KSpace K;
      if ( ! K.init( gimage->domain().lowerBound(),
		     gimage->domain().upperBound(),
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
    /// specified by a digitization in \a params. It is useful when
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
    static KSpace
    getKSpace( Parameters params =
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

    /// @param[in] params the parameters:
    ///   - gridsizex[1.0]: specifies the space between points along x.
    ///   - gridsizey[1.0]: specifies the space between points along y.
    ///   - gridsizez[1.0]: specifies the space between points along z.
    static ImageCellEmbedder
    getImageLinearEmbedder( CountedPtr<GrayScaleImage> gray_scale_image,
			    const Parameters&          params = parametersKSpace() )
    {
      auto K = getKSpace();
    }

    
    // ----------------------- DigitalSurface static services ------------------------
  public:
        
    /// @return the parameters and their default values which are
    /// related to digital surfaces.
    ///   - surfelAdjacency     [        1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel   [   100000]: number of tries in method Surfaces::findABel
    ///   - surfaceComponents   [ "AnyBig"]: "AnyBig"|"All", "AnyBig": any big-enough componen
    ///   - surfaceTraversal    ["Default"]: "Default"|"DepthFirst"|"BreadthFirst": "Default" default surface traversal, "DepthFirst": depth-first surface traversal, "BreadthFirst": breadth-first surface traversal.
    ///   - dualFaceSubdivision [     "No"]: "No"|"Naive"|"Centroid" specifies how dual faces are subdivided when exported.
    static Parameters parametersDigitalSurface()
    {
      return Parameters
	( "surfelAdjacency",   1 )
	( "nbTriesToFindABel", 100000 )
	( "surfaceComponents", "AnyBig" )
	( "surfaceTraversal",  "Default" )
	( "dualFaceSubdivision",   "No" ); 
    }

    /// Builds a light digital surface from a space \a K and a binary image \a bimage.
    ///
    /// @param[in] bimage a binary image representing the characteristic function of a digital shape.
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel [100000]: number of tries in method Surfaces::findABel
    ///
    /// @return a smart pointer on a (light) digital surface that
    /// represents the boundary of any big component of the digital shape.
    static CountedPtr<LightDigitalSurface>
    makeLightDigitalSurface
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      int nb_tries_to_find_a_bel = params[ "nbTriesToFindABel" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );

      // We have to search for a surfel that belong to a big connected component.
      CountedPtr<LightDigitalSurface> ptrSurface;
      Surfel       bel;
      Scalar       minsize    = bimage->extent().norm();
      unsigned int nb_surfels = 0;
      unsigned int tries      = 0;
      do {
        try { // Search initial bel
          bel = Surfaces<KSpace>::findABel( K, *bimage, nb_tries_to_find_a_bel );
        } catch (DGtal::InputException e) {
          trace.error() << "[Shortcuts::makeLightDigitalSurface]"
			<< " ERROR Unable to find bel." << std::endl;
          return ptrSurface;
        }
	// this pointer will be acquired by the surface.
        LightSurfaceContainer* surfContainer
	  = new LightSurfaceContainer( K, *bimage, surfAdj, bel );
        ptrSurface = CountedPtr<LightDigitalSurface>
	  ( new LightDigitalSurface( surfContainer ) ); // acquired
        nb_surfels = ptrSurface->size();
      } while ( ( nb_surfels < 2 * minsize ) && ( tries++ < 150 ) );
      if( tries >= 150 ) {
	trace.warning() << "[Shortcuts::makeLightDigitalSurface]"
			<< "ERROR cannot find a proper bel in a big enough component."
			<< std::endl;
      }
      return ptrSurface;
    }

    /// Returns a vector containing either all the light digital
    /// surfaces in the binary image \a bimage, or any one of its big
    /// components according to parameters.
    ///
    /// @param[in] bimage a binary image representing the
    /// characteristic function of a digital shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the
    /// digital shape.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [       1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel [  100000]: number of tries in method Surfaces::findABel
    ///   - surfaceComponents ["AnyBig"]: "AnyBig"|"All", "AnyBig": any big-enough component (> twice space width), "All": all components
    ///
    /// @return a vector of smart pointers to the connected (light)
    /// digital surfaces present in the binary image.
    static std::vector< CountedPtr<LightDigitalSurface> >
    makeLightDigitalSurfaces
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      SurfelRange surfel_reps;
      return makeLightDigitalSurfaces( surfel_reps, bimage, K, params );
    }

    /// Returns a vector containing either all the light digital
    /// surfaces in the binary image \a bimage, or any one of its big
    /// components according to parameters.
    ///
    /// @param[out] surfel_reps a vector of surfels, one surfel per
    /// digital surface component.
    ///
    /// @param[in] bimage a binary image representing the
    /// characteristic function of a digital shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the
    /// digital shape.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [       1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel [  100000]: number of tries in method Surfaces::findABel
    ///   - surfaceComponents ["AnyBig"]: "AnyBig"|"All", "AnyBig": any big-enough component (> twice space width), "All": all components
    ///
    /// @return a vector of smart pointers to the connected (light)
    /// digital surfaces present in the binary image.
    static std::vector< CountedPtr<LightDigitalSurface> >
    makeLightDigitalSurfaces
    ( SurfelRange&            surfel_reps,
      CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      std::vector< CountedPtr<LightDigitalSurface> > result;
      std::string component      = params[ "surfaceComponents" ].as<std::string>();
      if ( component == "AnyBig" ) {
	result.push_back( makeLightDigitalSurface( bimage, K, params ) );
	surfel_reps.push_back( *( result[ 0 ]->begin() ) );
	return result;
      }	
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      int nb_tries_to_find_a_bel = params[ "nbTriesToFindABel" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );
      // Extracts all boundary surfels
      SurfelSet all_surfels;
      Surfaces<KSpace>::sMakeBoundary( all_surfels, K, *bimage,
				       K.lowerBound(), K.upperBound() );
      // Builds all connected components of surfels.
      SurfelSet marked_surfels;
      CountedPtr<LightDigitalSurface> ptrSurface;
      for ( auto bel : all_surfels )
	{
	  if ( marked_surfels.count( bel ) != 0 ) continue;
	  surfel_reps.push_back( bel );
	  LightSurfaceContainer* surfContainer
	    = new LightSurfaceContainer( K, *bimage, surfAdj, bel );
	  ptrSurface = CountedPtr<LightDigitalSurface>
	    ( new LightDigitalSurface( surfContainer ) ); // acquired
	  // mark all surfels of the surface component.
	  marked_surfels.insert( ptrSurface->begin(), ptrSurface->end() );
	  // add surface component to result.
	  result.push_back( ptrSurface );
	}
      return result;
    }

    /// Creates a explicit digital surface representing the boundaries in
    /// the binary image \a bimage, or any one of its big components
    /// according to parameters.
    ///
    /// @param[in] bimage a binary image representing the
    /// characteristic function of a digital shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the
    /// digital shape.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [       1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on the explicit digital surface
    /// representing the boundaries in the binary image.
    static CountedPtr< DigitalSurface >
    makeDigitalSurface
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      SurfelSet all_surfels;
      bool      surfel_adjacency = params[ "surfelAdjacency" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );
      // Extracts all boundary surfels
      Surfaces<KSpace>::sMakeBoundary( all_surfels, K, *bimage,
				       K.lowerBound(), K.upperBound() );
      ExplicitSurfaceContainer* surfContainer
	= new ExplicitSurfaceContainer( K, surfAdj, all_surfels );
      return CountedPtr< DigitalSurface >
	    ( new DigitalSurface( surfContainer ) ); // acquired
    }


    /// Builds a explicit digital surface from an indexed digital surface.
    ///
    /// @note if the given surfel adjacency is not the same as the one
    /// chosen for the input indexed digital surface, the number of
    /// connected components may change in the process.
    ///
    /// @param[in] idx_surface any indexed digital surface.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [       1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on a explicit digital surface.
    static CountedPtr< DigitalSurface >
    makeDigitalSurface
    ( CountedPtr<IdxDigitalSurface> idx_surface,
      const Parameters&             params = parametersDigitalSurface() )
    {
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      const KSpace& K = idx_surface->container().space();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );
      auto all_idx_surfels
	= getIdxSurfelRange( idx_surface, Parameters( "surfaceTraversal", "Default" ) );
      auto idx2surfel = idx_surface->surfels();
      SurfelSet all_surfels;
      for ( auto idx : all_idx_surfels ) all_surfels.insert( idx2surfel[ idx ] );
      ExplicitSurfaceContainer* surfContainer
	= new ExplicitSurfaceContainer( K, surfAdj, all_surfels );
      return CountedPtr<DigitalSurface>
	( new DigitalSurface( surfContainer ) ); // acquired
    }
    
    /// Builds an indexed digital surface from a space \a K and a
    /// binary image \a bimage. Note that it may connected or not
    /// depending on parameters.
    ///
    /// @param[in] bimage a binary image representing the
    /// characteristic function of a digital shape.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the
    /// digital shape.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///   - nbTriesToFindABel [100000]: number of tries in method Surfaces::findABel
    ///   - surfaceComponents ["AnyBig"]: "AnyBig"|"All", "AnyBig": any big-enough component (> twice space width), "All": all components
    ///
    /// @return a smart pointer on the required indexed digital surface.
    static CountedPtr<IdxDigitalSurface>
    makeIdxDigitalSurface
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      std::string component      = params[ "surfaceComponents" ].as<std::string>();
      SurfelSet surfels;
      if ( component == "AnyBig" )
	{
	  auto light_surface = makeLightDigitalSurface( bimage, K, params );
	  surfels.insert( light_surface->begin(), light_surface->end() );
	}
      else if ( component == "All" )
	{
	  Surfaces<KSpace>::sMakeBoundary( surfels, K, *bimage,
					   K.lowerBound(), K.upperBound() );
	}
      return makeIdxDigitalSurface( surfels, K, params );
    }    

    /// Builds an indexed digital surface from a space \a K and an
    /// arbitrary range of surfels.
    ///
    /// @param[in] surfels an arbitrary range of surfels.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the given surfels.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on the indexed digital surface built over the surfels.
    template <typename TSurfelRange>
    static CountedPtr<IdxDigitalSurface>
    makeIdxDigitalSurface
    ( const TSurfelRange&  surfels,
      ConstAlias< KSpace > K,
      const Parameters&    params = parametersDigitalSurface() )
    {
      bool surfel_adjacency      = params[ "surfelAdjacency" ].as<int>();
      SurfelAdjacency< KSpace::dimension > surfAdj( surfel_adjacency );
      // Build indexed digital surface.
      CountedPtr<ExplicitSurfaceContainer> ptrSurfContainer
        ( new ExplicitSurfaceContainer( K, surfAdj, surfels ) );
      CountedPtr<IdxDigitalSurface> ptrSurface
	( new IdxDigitalSurface() );
      bool ok = ptrSurface->build( ptrSurfContainer );
      if ( !ok )
	trace.warning() << "[Shortcuts::makeIdxDigitalSurface] Error building indexed digital surface." << std::endl;
      return ptrSurface;
    }

    /// Builds an indexed digital surface from a light digital
    /// surface. Note that the surfel adjacency may be changed and a
    /// connected light digital surface could be disconnected in the process.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[in] surface a smart pointer on a light digital surface.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on the required indexed digital surface.
    template <typename TAnyDigitalSurface>
    static CountedPtr<IdxDigitalSurface>
    makeIdxDigitalSurface
    ( CountedPtr< TAnyDigitalSurface> surface,
      const Parameters&               params = parametersDigitalSurface() )
    {
      const KSpace& K = surface->container().space();
      SurfelSet     surfels;
      surfels.insert( surface->begin(), surface->end() );
      return makeIdxDigitalSurface( surfels, K, params );
    }    

    /// Builds an indexed digital surface from a vector of light digital
    /// surfaces. Note that the surfel adjacency may be changed and a
    /// connected light digital surface could be disconnected in the process.
    ///
    /// @note the surfaces must live in the same digital spaces. 
    ///
    /// @param[in] surfaces a vector of smart pointers on light digital surfaces.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on the required indexed digital surface.
    static CountedPtr<IdxDigitalSurface>
    makeIdxDigitalSurface
    ( const std::vector< CountedPtr<LightDigitalSurface> >& surfaces,
      const Parameters&       params = parametersDigitalSurface()  )
    {
      if ( surfaces.empty() ) return CountedPtr<IdxDigitalSurface>( 0 );
      const KSpace& K = surfaces[ 0 ]->container().space();
      SurfelSet     surfels;
      for ( auto i = 0; i < surfaces.size(); ++i ) {
	const KSpace& Ki = surfaces[ i ]->container().space();
	if ( ( Ki.lowerBound() != K.lowerBound() )
	     || ( Ki.upperBound() != K.upperBound() ) )
	  trace.warning() << "[Shortcuts::makeIdxDigitalSurface] Incompatible digital spaces for surface " << i << std::endl;
	surfels.insert( surfaces[ i ]->begin(), surfaces[ i ]->end() );
      }
      return makeIdxDigitalSurface( surfels, K, params );
    }    

    
    /// Given any digital surface, returns a vector of surfels in
    /// some specified order.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @param[in] params the parameters:
    ///   - surfaceTraversal  ["Default"]: "Default"|"DepthFirst"|"BreadthFirst": "Default" default surface traversal, "DepthFirst": depth-first surface traversal, "BreadthFirst": breadth-first surface traversal.
    ///
    /// @return a range of surfels as a vector.
    template <typename TAnyDigitalSurface>
    static SurfelRange
    getSurfelRange
    ( CountedPtr< TAnyDigitalSurface> surface,
      const Parameters&   params = parametersDigitalSurface() )
    {
      return getSurfelRange( surface, *( surface->begin() ), params );
    }

    /// Given a light digital surface, returns a vector of surfels in
    /// some specified order.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @param[in] start_surfel the surfel where the traversal starts
    /// in case of depth-first/breadth-first traversal.
    ///
    /// @param[in] params the parameters:
    ///   - surfaceTraversal  ["Default"]: "Default"|"DepthFirst"|"BreadthFirst": "Default" default surface traversal, "DepthFirst": depth-first surface traversal, "BreadthFirst": breadth-first surface traversal.
    ///
    /// @return a range of surfels as a vector.
    template <typename TAnyDigitalSurface>
    static SurfelRange
    getSurfelRange
    ( CountedPtr< TAnyDigitalSurface> surface,
      const Surfel&       start_surfel,
      const Parameters&   params = parametersDigitalSurface() )
    {
      SurfelRange result;
      std::string traversal = params[ "surfaceTraversal" ].as<std::string>();
      if ( traversal == "DepthFirst" )
	{
	  typedef DepthFirstVisitor< TAnyDigitalSurface > Visitor;
	  typedef GraphVisitorRange< Visitor > VisitorRange;
	  VisitorRange range( new Visitor( *surface, start_surfel ) );
	  std::for_each( range.begin(), range.end(),
			 [&result] ( Surfel s ) { result.push_back( s ); } );
	}
      else if ( traversal == "BreadthFirst" )
	{
	  typedef BreadthFirstVisitor< TAnyDigitalSurface > Visitor;
	  typedef GraphVisitorRange< Visitor > VisitorRange;
	  VisitorRange range( new Visitor( *surface, start_surfel ) );
	  std::for_each( range.begin(), range.end(),
			 [&result] ( Surfel s ) { result.push_back( s ); } );
	}
      else
	{
	  std::for_each( surface->begin(), surface->end(),
			 [&result] ( Surfel s ) { result.push_back( s ); } );
	}
      return result;
    }

    /// Given an indexed digital surface, returns a vector of surfels in
    /// some specified order.
    ///
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @param[in] params the parameters:
    ///   - surfaceTraversal  ["Default"]: "Default"|"DepthFirst"|"BreadthFirst": "Default" default surface traversal, "DepthFirst": depth-first surface traversal, "BreadthFirst": breadth-first surface traversal.
    ///
    /// @return a range of indexed surfels as a vector.
    static IdxSurfelRange
    getIdxSurfelRange
    ( CountedPtr<IdxDigitalSurface> surface,
      const Parameters&   params = parametersDigitalSurface() )
    {
      return getIdxSurfelRange( surface, (IdxSurfel) 0, params );
    }
    
    /// Given an indexed digital surface, returns a vector of surfels in
    /// some specified order.
    ///
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @param[in] start_surfel the surfel where the traversal starts
    /// in case of depth-first/breadth-first traversal.
    ///
    /// @param[in] params the parameters:
    ///   - surfaceTraversal  ["Default"]: "Default"|"DepthFirst"|"BreadthFirst": "Default" default surface traversal, "DepthFirst": depth-first surface traversal, "BreadthFirst": breadth-first surface traversal.
    ///
    /// @return a range of indexed surfels as a vector.
    static IdxSurfelRange
    getIdxSurfelRange
    ( CountedPtr<IdxDigitalSurface> surface,
      const IdxSurfel&    start_surfel,
      const Parameters&   params = parametersDigitalSurface() )
    {
      IdxSurfelRange result;
      std::string traversal = params[ "surfaceTraversal" ].as<std::string>();
      if ( traversal == "DepthFirst" )
	{
	  typedef DepthFirstVisitor< IdxDigitalSurface > Visitor;
	  typedef GraphVisitorRange< Visitor > VisitorRange;
	  VisitorRange range( new Visitor( *surface, start_surfel ) );
	  std::for_each( range.begin(), range.end(),
			 [&result] ( IdxSurfel s ) { result.push_back( s ); } );
	}
      else if ( traversal == "BreadthFirst" )
	{
	  typedef BreadthFirstVisitor< IdxDigitalSurface > Visitor;
	  typedef GraphVisitorRange< Visitor > VisitorRange;
	  VisitorRange range( new Visitor( *surface, start_surfel ) );
	  std::for_each( range.begin(), range.end(),
			 [&result] ( IdxSurfel s ) { result.push_back( s ); } );
	}
      else return surface->allVertices();
      return result;
    }

    // --------------------------- geometry estimation ------------------------------
  public:

    /// @return the parameters and their default values which are used
    /// to estimate the geometry of a digital surface.
    ///   - verbose         [     1]: verbose trace mode 0: silent, 1: verbose.
    ///   - t-ring          [   3.0]: the radius used when computing convolved trivial normals (it is a graph distance, not related to the grid step).
    ///   - R-radius        [  10.0]: the constant for distance parameter R in R(h)=R h^alpha (VCM).
    ///   - r-radius        [   3.0]: the constant for kernel radius parameter r in r(h)=r h^alpha (VCM,II,Trivial).
    ///   - kernel          [ "hat"]: the kernel integration function chi_r, either "hat" or "ball". )
    ///   - alpha           [  0.33]: the parameter alpha in r(h)=r h^alpha (VCM, II)."
    ///   - surfelEmbedding [     0]: the surfel -> point embedding for VCM estimator: 0: Pointels, 1: InnerSpel, 2: OuterSpel.
    static Parameters parametersGeometryEstimation()
    {
      return Parameters
	( "verbose",           1 )
	( "t-ring",          3.0 )
	( "kernel",        "hat" )
	( "gridstep",        1.0 )
	( "R-radius",       10.0 )
	( "r-radius",        3.0 )
	( "alpha",          0.33 )
	( "surfelEmbedding",   0 );
    }
    
    /// Given a digital space \a K and a vector of \a surfels,
    /// returns the trivial normals at the specified surfels, in the
    /// same order.
    ///
    /// @param[in] K the Khalimsky space whose domain encompasses the digital shape.
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    ///
    /// @return the vector containing the trivial normal vectors, in the
    /// same order as \a surfels.
    static RealVectors
    getTrivialNormalVectors( const KSpace&      K,
			     const SurfelRange& surfels )
    {
      std::vector< RealVector > result;
      for ( auto s : surfels )
	{
	  Dimension  k = K.sOrthDir( s );
	  bool  direct = K.sDirect( s, k );
	  RealVector t = RealVector::zero;
	  t[ k ]       = direct ? -1.0 : 1.0;
	  result.push_back( t );
	}
      return result;
    }

    /// Given a digital surface \a surface, a sequence of \a surfels,
    /// and some parameters \a params, returns the convolved trivial
    /// normal vector estimations at the specified surfels, in the
    /// same order.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[in] surface the digital surface
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    /// @param[in] params the parameters:
    ///   - verbose         [     1]: verbose trace mode 0: silent, 1: verbose.
    ///   - t-ring          [   3.0]: the radius used when computing convolved trivial normals (it is a graph distance, not related to the grid step).
    ///
    /// @return the vector containing the estimated normals, in the
    /// same order as \a surfels.
    template <typename TAnyDigitalSurface>
    static RealVectors
    getCTrivialNormalVectors
    ( CountedPtr<TAnyDigitalSurface> surface,
      const SurfelRange&             surfels,
      const Parameters&              params = parametersGeometryEstimation() )
    {
      int    verbose = params[ "verbose"  ].as<int>();
      Scalar       t = params[ "t-ring"   ].as<double>();
      typedef typename TAnyDigitalSurface::DigitalSurfaceContainer  SurfaceContainer;
      typedef ExactPredicateLpSeparableMetric<Space,2>              Metric;
      typedef functors::HatFunction<Scalar>                         Functor;
      typedef functors::ElementaryConvolutionNormalVectorEstimator
	< Surfel, CanonicSCellEmbedder<KSpace> >                    SurfelFunctor;
      typedef LocalEstimatorFromSurfelFunctorAdapter
	< SurfaceContainer, Metric, SurfelFunctor, Functor>         NormalEstimator;
      if ( verbose > 0 )
	trace.info() << " CTrivial normal t=" << t << " (discrete)" << std::endl;
      const Functor fct( 1.0, t );
      const KSpace &  K = surface->container().space();
      Metric    aMetric;
      CanonicSCellEmbedder<KSpace> canonic_embedder( K );
      std::vector< RealVector >    n_estimations;
      SurfelFunctor                surfelFct( canonic_embedder, 1.0 );
      NormalEstimator              estimator;
      estimator.attach( *surface);
      estimator.setParams( aMetric, surfelFct, fct, t );
      estimator.init( 1.0, surfels.begin(), surfels.end());
      estimator.eval( surfels.begin(), surfels.end(),
		      std::back_inserter( n_estimations ) );
      std::transform( n_estimations.cbegin(), n_estimations.cend(), n_estimations.begin(),
		      [] ( RealVector v ) { return -v; } );
      return n_estimations;
    }

    /// Given a digital surface \a surface, a sequence of \a surfels,
    /// and some parameters \a params, returns the normal Voronoi
    /// Covariance Measure (VCM) estimation at the specified surfels,
    /// in the same order.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[in] surface the digital surface
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    /// @param[in] params the parameters:
    ///   - verbose         [     1]: verbose trace mode 0: silent, 1: verbose.
    ///   - t-ring          [   3.0]: the radius used when computing convolved trivial normals (it is a graph distance, not related to the grid step).
    ///   - R-radius        [  10.0]: the constant for distance parameter R in R(h)=R h^alpha (VCM).
    ///   - r-radius        [   3.0]: the constant for kernel radius parameter r in r(h)=r h^alpha (VCM,II,Trivial).
    ///   - kernel          [ "hat"]: the kernel integration function chi_r, either "hat" or "ball". )
    ///   - alpha           [  0.33]: the parameter alpha in r(h)=r h^alpha (VCM, II)."
    ///   - surfelEmbedding [     0]: the surfel -> point embedding for VCM estimator: 0: Pointels, 1: InnerSpel, 2: OuterSpel.
    ///
    /// @return the vector containing the estimated normals, in the
    /// same order as \a surfels.
    template <typename TAnyDigitalSurface>
    static RealVectors
    getVCMNormalVectors
    ( CountedPtr<TAnyDigitalSurface> surface,
      const SurfelRange&             surfels,
      const Parameters&              params = parametersGeometryEstimation() )
    {
      typedef ExactPredicateLpSeparableMetric<Space,2> Metric;
      typedef typename TAnyDigitalSurface::DigitalSurfaceContainer SurfaceContainer;
      RealVectors n_estimations;
      int        verbose = params[ "verbose"   ].as<int>();
      std::string kernel = params[ "kernel"    ].as<std::string>();
      Scalar      h      = params[ "gridstep"  ].as<Scalar>();
      Scalar      R      = params[ "R-radius"  ].as<Scalar>();
      Scalar      r      = params[ "r-radius"  ].as<Scalar>();
      Scalar      t      = params[ "t-ring"    ].as<Scalar>();
      Scalar      alpha  = params[ "alpha"     ].as<Scalar>();
      int      embedding = params[ "embedding" ].as<int>();
      // Adjust parameters according to gridstep if specified.
      if ( alpha != 1.0 ) R *= pow( h, alpha-1.0 );
      if ( alpha != 1.0 ) r *= pow( h, alpha-1.0 );
      Surfel2PointEmbedding embType = embedding == 0 ? Pointels :
                                      embedding == 1 ? InnerSpel : OuterSpel;
      if ( verbose > 0 ) {
	trace.info() << "- VCM normal kernel=" << kernel << " emb=" << embedding
		     << " alpha=" << alpha << std::endl;
	trace.info() << "- VCM normal r=" << (r*h)  << " (continuous) "
		     << r << " (discrete)" << std::endl;
	trace.info() << "- VCM normal R=" << (R*h)  << " (continuous) "
		     << R << " (discrete)" << std::endl;
	trace.info() << "- VCM normal t=" << t << " (discrete)" << std::endl;
      }
      if ( kernel == "hat" ) {
	typedef functors::HatPointFunction<Point,Scalar>             KernelFunction;
	typedef VoronoiCovarianceMeasureOnDigitalSurface
	  < SurfaceContainer, Metric, KernelFunction >               VCMOnSurface;
	typedef functors::VCMNormalVectorFunctor<VCMOnSurface>       NormalFunctor;
	typedef VCMDigitalSurfaceLocalEstimator
	  < SurfaceContainer, Metric, KernelFunction, NormalFunctor> VCMNormalEstimator;
	KernelFunction chi_r( 1.0, r );
	VCMNormalEstimator estimator;
	estimator.attach( *surface );
	estimator.setParams( embType, R, r, chi_r, t, Metric(), verbose > 0 );
	estimator.init( h, surfels.begin(), surfels.end() );
	estimator.eval( surfels.begin(), surfels.end(),
			std::back_inserter( n_estimations ) );
      } else if ( kernel == "ball" ) {
	typedef functors::BallConstantPointFunction<Point,Scalar>    KernelFunction;
	typedef VoronoiCovarianceMeasureOnDigitalSurface
	  < SurfaceContainer, Metric, KernelFunction >               VCMOnSurface;
	typedef functors::VCMNormalVectorFunctor<VCMOnSurface>       NormalFunctor;
	typedef VCMDigitalSurfaceLocalEstimator
	  < SurfaceContainer, Metric, KernelFunction, NormalFunctor> VCMNormalEstimator;
	KernelFunction chi_r( 1.0, r );
	VCMNormalEstimator estimator;
	estimator.attach( *surface );
	estimator.setParams( embType, R, r, chi_r, t, Metric(), verbose > 0 );
	estimator.init( h, surfels.begin(), surfels.end() );
	estimator.eval( surfels.begin(), surfels.end(),
			std::back_inserter( n_estimations ) );
      }
      return n_estimations;
    }

    /// Given a digital shape \a bimage, a sequence of \a surfels,
    /// and some parameters \a vm, returns the normal Integral
    /// Invariant (VCM) estimation at the specified surfels, in the
    /// same order.
    ///
    /// @param[in] bimage the characteristic function of the shape as a binary image (inside is true, outside is false).
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    /// @param[in] params the parameters:
    ///   - verbose         [     1]: verbose trace mode 0: silent, 1: verbose.
    ///   - r-radius        [   3.0]: the constant for kernel radius parameter r in r(h)=r h^alpha (VCM,II,Trivial).
    ///   - alpha           [  0.33]: the parameter alpha in r(h)=r h^alpha (VCM, II)."
    ///   - gridstep        [   1.0]: the digitization gridstep (often denoted by h).
    ///
    /// @return the vector containing the estimated normals, in the
    /// same order as \a surfels.
    ///
    /// @note It is better to have surfels in a specific order, as
    /// given for instance by a depth-first traversal (@see getSurfelRange)
    static RealVectors
    getIINormalVectors( CountedPtr<BinaryImage> bimage,
			const SurfelRange&      surfels,
			const Parameters&       params
			= parametersGeometryEstimation() | parametersKSpace() )
    {
      typedef functors::IINormalDirectionFunctor<Space> IINormalFunctor;
      typedef IntegralInvariantCovarianceEstimator
	<KSpace, BinaryImage, IINormalFunctor>          IINormalEstimator;
      auto K =  getKSpace( bimage, params );

      RealVectors n_estimations;
      int        verbose = params[ "verbose"   ].as<int>();
      Scalar      h      = params[ "gridstep"  ].as<Scalar>();
      Scalar      r      = params[ "r-radius"  ].as<Scalar>();
      Scalar      alpha  = params[ "alpha"     ].as<Scalar>();
      if ( alpha != 1.0 ) r *= pow( h, alpha-1.0 );
      if ( verbose > 0 ) {
	trace.info() << "- II normal alpha=" << alpha << std::endl;
	trace.info() << "- II normal r=" << (r*h)  << " (continuous) "
		     << r << " (discrete)" << std::endl;
      }
      IINormalFunctor     functor;
      functor.init( h, r*h );
      IINormalEstimator   ii_estimator( functor );
      ii_estimator.attach( K, *bimage );
      ii_estimator.setParams( r );
      ii_estimator.init( h, surfels.begin(), surfels.end() );
      ii_estimator.eval( surfels.begin(), surfels.end(),
			 std::back_inserter( n_estimations ) );
      const RealVectors n_trivial = getTrivialNormalVectors( K, surfels );
      orientVectors( n_estimations, n_trivial );
      return n_estimations;
    }

    // ------------------------- Error measures services -------------------------

    /// Orient \a v so that it points in the same direction as \a
    /// ref_v (scalar product is then non-negative afterwards).
    ///
    /// @param[inout] v the vectors to reorient.
    /// @param[in]    ref_v the vectors having the reference orientation.
    static void
    orientVectors( RealVectors&       v,
		   const RealVectors& ref_v )
    {
      std::transform( ref_v.cbegin(), ref_v.cend(), v.cbegin(), v.begin(), 
		      [] ( RealVector rw, RealVector w )
		      { return rw.dot( w ) >= 0.0 ? w : -w; } );
    }
    
    /// Computes the statistic that measures the angle differences
    /// between the two arrays of unit vectors.
    ///
    /// @param[in] v1 the first array of unit vectors (normals)
    /// @param[in] v2 the second array of unit vectors (normals)
    /// @return their angle difference as a statistic.
    static ScalarStatistic
    getVectorsAngleDeviation( const RealVectors& v1,
			      const RealVectors& v2 )
    {
      ScalarStatistic stat;
      for ( auto it1 = v1.cbegin(), it2 = v2.cbegin(), itE1 = v1.end();
	    it1 != itE1; ++it1, ++it2 )
	{
          Scalar angle_error = acos( (*it1).dot( *it2 ) );
          stat.addValue( angle_error );
	}
      stat.terminate();
      return stat;
    }
    
    /// Computes the absolute difference between each element of the two vectors.
    /// @param[in] v1 any vector of values.
    /// @param[in] v2 any vector of values.
    /// @return the vector composed of elemenst |v1[i]-v2[i]|.
    static Scalars
    getScalarsAbsoluteDifference( const Scalars & v1,
				  const Scalars & v2 )
    {
      Scalars result( v1.size() );
      std::transform( v2.cbegin(), v2.cend(), v1.cbegin(), result.begin(), 
		      [] ( Scalar val1, Scalar val2 )
		      { return fabs( val1 - val2 ); } );
      return result;
    }

    /// Computes the l2-norm of v1-v2, ie the square root of the
    /// mean-squared error of the two vectors.
    ///
    /// @param[in] v1 any vector of values.
    /// @param[in] v2 any vector of values.
    /// @return the normL2 of v1-v2, ie. sqrt( 1/n sum_i (v1[i]-v2[i])^2 ).
    static Scalar
    getScalarsNormL2( const Scalars & v1,
		      const Scalars & v2 )
    {
      Scalar sum = 0;
      for ( unsigned int i = 0; i < v1.size(); i++ )
	sum += ( v1[ i ] - v2[ i ] ) * ( v1[ i ] - v2[ i ] );
      return sqrt( sum / v1.size() );
    }

    /// Computes the l1-norm of v1-v2, ie the average of the absolute
    /// differences of the two vectors.
    ///
    /// @param[in] v1 any vector of values.
    /// @param[in] v2 any vector of values.
    /// @return the normL1 of v1-v2, ie. 1/n sum_i |v1[i]-v2[i]|.
    static Scalar
    getScalarsNormL1( const Scalars & v1,
		      const Scalars & v2 )
    {
      Scalar sum = 0;
      for ( unsigned int i = 0; i < v1.size(); i++ )
	sum += fabs( v1[ i ] - v2[ i ] );
      return sum / v1.size();
    }

    /// Computes the loo-norm of v1-v2, ie the maximum of the absolute
    /// differences of the two vectors.
    ///
    /// @param[in] v1 any vector of values.
    /// @param[in] v2 any vector of values.
    /// @return the normLoo of v1-v2, ie. max_i |v1[i]-v2[i]|.
    static Scalar
    getScalarsNormLoo( const Scalars & v1,
		       const Scalars & v2 )
    {
      Scalar loo = 0;
      for ( unsigned int i = 0; i < v1.size(); i++ )
	loo = std::max( loo, fabs( v1[ i ] - v2[ i ] ) );
      return loo;
    }

    // ----------------------- Mesh services ------------------------------
  public:
      
    /// Builds a triangulated surface (class TriangulatedSurface) from
    /// a mesh (class Mesh). Note that a triangulated surface contains
    /// only triangles, so polygonal faces (0,1,2,3,4,...) of the
    /// input mesh are (naively) triangulated (triangles (0,1,2),
    /// (0,2,3), (0,3,4), etc). Furthermore, the output triangulated
    /// surface rebuilds a topology between faces.
    ///
    /// @param[in] aMesh any mesh (which should be a valid combinatorial surface).
    ///
    /// @return a smart pointer on the built triangulated surface or 0 if
    /// the mesh was invalid.
    static CountedPtr< TriangulatedSurface >
    makeTriangulatedSurface( CountedPtr< Mesh > aMesh )
    {
      auto pTriSurf = CountedPtr<TriangulatedSurface>
	    ( new TriangulatedSurface ); // acquired
      bool ok = MeshHelpers::mesh2TriangulatedSurface( *aMesh, *pTriSurf );
      return ok ? pTriSurf : 0;
    }

    /// Builds the dual triangulated surface associated to the given digital surface.
    ///
    /// @param[in] aSurface any digital surface
    /// @param[out] s2i the map Surfel -> Vertex index in the triangulated surface.
    /// @return a smart pointer on the built triangulated surface.
    static CountedPtr< TriangulatedSurface >
    makeTriangulatedSurface( CountedPtr< DigitalSurface > aSurface,
			     Surfel2Index& s2i )
    {
      CanonicCellEmbedder< KSpace > cembedder;
      auto pTriSurf = CountedPtr<TriangulatedSurface>
	    ( new TriangulatedSurface ); // acquired
      MeshHelpers::digitalSurface2DualTriangulatedSurface
	( *aSurface, cembedder, *pTriSurf, s2i );
      return pTriSurf;
    }

    /// Builds the dual triangulated surface associated to the given digital surface.
    ///
    /// @param[in] aSurface any (light) digital surface
    /// @param[out] s2i the map Surfel -> Vertex index in the triangulated surface.
    /// @return a smart pointer on the built triangulated surface.
    static CountedPtr< TriangulatedSurface >
    makeTriangulatedSurface( CountedPtr< LightDigitalSurface > aSurface,
			     Surfel2Index& s2i )
    {
      CanonicCellEmbedder< KSpace > cembedder;
      auto pTriSurf = CountedPtr<TriangulatedSurface>
	    ( new TriangulatedSurface ); // acquired
      MeshHelpers::digitalSurface2DualTriangulatedSurface
	( *aSurface, cembedder, *pTriSurf, s2i );
      return pTriSurf;
    }

    /// Builds a polygon mesh (class PolygonalSurface) from
    /// a mesh (class Mesh). The output polygonal
    /// surface rebuilds a topology between faces.
    ///
    /// @param[in] aMesh any mesh (which should be a valid combinatorial surface).
    ///
    /// @return a smart pointer on the built polygonal surface or 0 if
    /// the mesh was invalid.
    static CountedPtr< PolygonalSurface >
    makePolygonalSurface( CountedPtr< Mesh > aMesh )
    {
      auto pPolySurf = CountedPtr<PolygonalSurface>
	    ( new PolygonalSurface ); // acquired
      bool ok = MeshHelpers::mesh2PolygonalSurface( *aMesh, *pPolySurf );
      return ok ? pPolySurf : 0;
    }

			     
    // ------------------------------ utilities ------------------------------
  public:
    
    /// Outputs a range of surfels as an OBJ file, embedding each
    /// vertex using the given cell embedder (3D only).
    ///
    /// @tparam TCellEmbedder any model of CCellEmbedder
    /// @param[out] output the output stream.
    /// @param[in] surfels the range of surfels (oriented cells of dimension 2).
    /// @param[in] embedder the embedder for mapping surfel vertices (cells of dimension 0) to points in space.
    /// @return 'true' if the output stream is good.
    template <typename TCellEmbedder = CanonicCellEmbedder< KSpace > >
    static bool
    outputSurfelsAsObj
    ( std::ostream&              output,
      const SurfelRange&         surfels,
      const TCellEmbedder&       embedder )
    {
      typedef unsigned long Size;
      BOOST_STATIC_ASSERT (( KSpace::dimension == 3 ));
      BOOST_CONCEPT_ASSERT(( concepts::CCellEmbedder< TCellEmbedder > ));
      const KSpace& K = embedder.space();
      // Number and output vertices.
      std::map< Cell, Size > vtx_numbering;
      Size n = 1;  // OBJ vertex numbering start at 1 
      for ( auto&& s : surfels ) {
	CellRange primal_vtcs = getPrimalVertices( K, s );
	for ( auto&& primal_vtx : primal_vtcs ) {
	  if ( ! vtx_numbering.count( primal_vtx ) ) {
	    vtx_numbering[ primal_vtx ] = n++;
	    // Output vertex positions
	    RealPoint p = embedder( primal_vtx );
	    output << "v " << p[ 0 ] << " " << p[ 1 ] << " " << p[ 2 ] << std::endl;
	  }
	}
      }
      // Outputs all faces
      for ( auto&& s : surfels ) {
	output << "f";
	CellRange primal_vtcs = getPrimalVertices( K, s, true );
	for ( auto&& primal_vtx : primal_vtcs ) {
	  output << " " << vtx_numbering[ primal_vtx ];
	}
	output << std::endl;
      }      
      return output.good();
    }
    
    /// Outputs a digital surface, seen from the primal point of view
    /// (surfels=face), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside).
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on a digital surface.
    ///
    /// @return 'true' if the output stream is good.
    template <typename TAnyDigitalSurface>
    static bool
    outputPrimalDigitalSurfaceAsObj
    ( std::ostream&              output,
      CountedPtr<TAnyDigitalSurface> surface )
    {
      CanonicCellEmbedder< KSpace > embedder( surface->container().space() );
      return outputPrimalDigitalSurfaceAsObj( output, surface, embedder );
    }
    
    /// Outputs a digital surface, seen from the primal point of view
    /// (surfels=face), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside). Each vertex is
    /// mapped to a 3D point using the given cell embedder.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @tparam TCellEmbedder any model of CCellEmbedder
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on a digital surface.
    /// @param[in] embedder the embedder for mapping surfel vertices (cells of dimension 0) to points in space.
    /// @return 'true' if the output stream is good.
    template < typename TAnyDigitalSurface,
	       typename TCellEmbedder = CanonicCellEmbedder< KSpace > >
    static bool
    outputPrimalDigitalSurfaceAsObj
    ( std::ostream&              output,
      CountedPtr<TAnyDigitalSurface> surface,
      const TCellEmbedder&       embedder )
    {
      auto surfels = getSurfelRange( surface, Parameters( "Traversal", "Default" ) );
      return outputSurfelsAsObj( output, surfels, embedder );
    }

    /// Outputs an indexed digital surface, seen from the primal point
    /// of view (surfels=face), as an OBJ file (3D only). Note that
    /// faces are oriented consistently (normals toward outside).
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on an indexed digital surface.
    ///
    /// @return 'true' if the output stream is good.
    static bool
    outputPrimalIdxDigitalSurfaceAsObj
    ( std::ostream&              output,
      CountedPtr<IdxDigitalSurface> surface )
    {
      CanonicCellEmbedder< KSpace > embedder( surface->container().space() );
      return outputPrimalIdxDigitalSurfaceAsObj( output, surface, embedder );
    }
    
    /// Outputs an indexed digital surface, seen from the primal point of view
    /// (surfels=face), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside). Each vertex is
    /// mapped to a 3D point using the given cell embedder.
    ///
    /// @tparam TCellEmbedder any model of CCellEmbedder
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on an indexed digital surface.
    /// @param[in] embedder the embedder for mapping surfel vertices (cells of dimension 0) to points in space.
    /// @return 'true' if the output stream is good.
    template <typename TCellEmbedder = CanonicCellEmbedder< KSpace > >
    static bool
    outputPrimalIdxDigitalSurfaceAsObj
    ( std::ostream&                 output,
      CountedPtr<IdxDigitalSurface> surface,
      const TCellEmbedder&          embedder )
    {
      auto idxsurfels = getIdxSurfelRange( surface, Parameters( "Traversal", "Default" ) );
      auto  surfelmap = surface->surfels();
      SurfelRange surfels;
      for ( auto&& idx : idxsurfels )
	surfels.push_back( surfelmap[ idx ] );
      return outputSurfelsAsObj( output, surfels, embedder );
    }

    /// Outputs a digital surface, seen from the dual point of view
    /// (surfels=vertices), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside).
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on a digital surface.
    /// @param[in] params the parameters:
    ///   - dualFaceSubdivision [     "No"]: "No"|"Naive"|"Centroid" specifies how dual faces are subdivided when exported.
    ///
    /// @return 'true' if the output stream is good.
    template <typename TAnyDigitalSurface>
    static bool
    outputDualDigitalSurfaceAsObj
    ( std::ostream&              output,
      CountedPtr<TAnyDigitalSurface> surface,
      const Parameters&   params = parametersDigitalSurface() )
    {
      CanonicCellEmbedder< KSpace > embedder( surface->container().space() );
      return outputDualDigitalSurfaceAsObj( output, surface, embedder, params );
    }
    
    /// Outputs a digital surface, seen from the dual point of view
    /// (surfels=vertices), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside). Each vertex is
    /// mapped to a 3D point using the given cell embedder.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::LightDigitalSurface or Shortcuts::DigitalSurface.
    ///
    /// @tparam TCellEmbedder any model of CCellEmbedder
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on a digital surface.
    /// @param[in] embedder the embedder for mapping (unsigned) surfels (cells of dimension 2) to points in space.
    /// @param[in] params the parameters:
    ///   - dualFaceSubdivision [     "No"]: "No"|"Naive"|"Centroid" specifies how dual faces are subdivided when exported.
    ///
    /// @return 'true' if the output stream is good.
    template < typename TAnyDigitalSurface,
	       typename TCellEmbedder = CanonicCellEmbedder< KSpace > >
    static bool
    outputDualDigitalSurfaceAsObj
    ( std::ostream&              output,
      CountedPtr<TAnyDigitalSurface> surface,
      const TCellEmbedder&       embedder,
      const Parameters&   params = parametersDigitalSurface() )
    {
      typedef unsigned long Size;
      typedef typename TAnyDigitalSurface::Face Face;
      BOOST_STATIC_ASSERT (( KSpace::dimension == 3 ));
      BOOST_CONCEPT_ASSERT(( concepts::CCellEmbedder< TCellEmbedder > ));
      std::string dualFaceSubdivision = params[ "dualFaceSubdivision" ].as<std::string>();
      const int   subdivide
	= dualFaceSubdivision == "Naive"    ? 1
	: dualFaceSubdivision == "Centroid" ? 2
	: 0;
      const KSpace& K = embedder.space();
      // Number and ouput vertices.
      std::map< Surfel, Size > vtx_numbering;
      std::map< Face,   Size > sub_numbering;
      Size n = 1;  // OBJ vertex numbering start at 1 
      for ( auto&& s : *surface ) {
	if ( ! vtx_numbering.count( s ) ) {
	  vtx_numbering[ s ] = n++;
	  // Output vertex positions
	  RealPoint p = embedder( K.unsigns( s ) );
	  output << "v " << p[ 0 ] << " " << p[ 1 ] << " " << p[ 2 ] << std::endl;
	}
      }
      auto faces = surface->allClosedFaces();
      Size c = 0;
      // Prepare centroids if necessary
      if ( subdivide == 2 ) {
	for ( auto&& f : faces ) {
	  auto vtcs = surface->verticesAroundFace( f );
	  Size   nv = vtcs.size();
	  if ( nv > 3 ) {
	    sub_numbering[ f ] = n++;
	    RealPoint p = RealPoint::zero;
	    for ( auto&& s : vtcs ) p += embedder( K.unsigns( s ) );
	    p /= nv;
	    output << "v " << p[ 0 ] << " " << p[ 1 ] << " " << p[ 2 ] << std::endl;
	  }
	}
      }
      // Outputs closed faces.
      if ( subdivide == 0 ) { // No subdivision
	for ( auto&& f : faces ) {
	  output << "f";
	  auto vtcs = surface->verticesAroundFace( f );
	  std::reverse( vtcs.begin(), vtcs.end() );
	  for ( auto&& s : vtcs )
	    output << " " << vtx_numbering[ s ];
	  output << std::endl;
	}
      } else if ( subdivide == 1 ) { // naive subdivision
	for ( auto&& f : faces ) {
	  auto vtcs = surface->verticesAroundFace( f );
	  Size   nv = vtcs.size();
	  for ( Size i = 1; i < nv - 1; ++i )
	    output << "f " << vtx_numbering[ vtcs[ 0 ] ]
		   << " "  << vtx_numbering[ vtcs[ i+1 ] ]
		   << " "  << vtx_numbering[ vtcs[ i ] ] << std::endl;
	}
      } else if ( subdivide == 2 ) { // centroid subdivision
	for ( auto&& f : faces ) {
	  auto vtcs = surface->verticesAroundFace( f );
	  Size   nv = vtcs.size();
	  if ( nv == 3 )
	    output << "f " << vtx_numbering[ vtcs[ 0 ] ]
		   << " "  << vtx_numbering[ vtcs[ 2 ] ]
		   << " "  << vtx_numbering[ vtcs[ 1 ] ] << std::endl;
	  else {
	    Size c = sub_numbering[ f ];
	    for ( Size i = 0; i < nv; ++i ) {
	      output << "f " << c
		     << " "  << vtx_numbering[ vtcs[ (i+1)%nv ] ]
		     << " "  << vtx_numbering[ vtcs[ i ] ] << std::endl;
	    }
	  }
	}
      }
      return output.good();
    }

    /// Outputs an indexed digital surface, seen from the dual point of view
    /// (surfels=vertices), as an OBJ file (3D only). Note that faces are
    /// oriented consistently (normals toward outside).
    ///
    /// @param[out] output the output stream.
    /// @param[in] surface a smart pointer on an indexed digital surface.
    /// @param[in] params the parameters:
    ///   - dualFaceSubdivision [     "No"]: "No"|"Naive"|"Centroid" specifies how dual faces are subdivided when exported.
    ///
    /// @return 'true' if the output stream is good.
    static bool
    outputDualIdxDigitalSurfaceAsObj
    ( std::ostream&                 output,
      CountedPtr<IdxDigitalSurface> surface,
      const Parameters&             params = parametersDigitalSurface() )
    {
      const KSpace& K = surface->container().space();
      auto explicit_surface = makeDigitalSurface( surface, params );
      struct CellEmbedder {
	using KSpace    = KSpace;
	using Cell      = Cell;
	using RealPoint = RealPoint;
	using Argument  = Cell;
	using Value     = RealPoint;
	const KSpace* _K;
	std::map< Cell, RealPoint > _embedding;
	CellEmbedder( ConstAlias< KSpace > K ) : _K( &K ) {}
	const KSpace & space() const { return *_K; }
	RealPoint embed( const Cell & cell ) const
	{
	  auto it = _embedding.find( cell );
	  return it != _embedding.end() ? it->second : RealPoint::zero;
	}
	RealPoint operator()( const Cell & cell ) const
	{ return embed( cell ); }
      };
      
      CellEmbedder embedder( K );
      for ( auto&& idx_surfel : *surface ) {
	Cell      s = K.unsigns( surface->surfel( idx_surfel ) );
	RealPoint p = surface->position( idx_surfel );
	embedder._embedding[ s ] = p;
      }
      return outputDualDigitalSurfaceAsObj( output,
					    explicit_surface, embedder, params );
    }

    // -------------------- map I/O services ------------------------------------------
  public:
    struct CellWriter {
      void operator()( std::ostream& output, const KSpace& K, const Cell & cell )
      {
	for ( Dimension d = 0; d < KSpace::dimension; ++d )
	  output << " " << K.sKCoord( cell, d );
      }
    };
    struct CellReader {
      Cell operator()( std::istream& input, const KSpace& K )
      {
	Point kp;
	for ( Dimension d = 0; d < KSpace::dimension; ++d )
	  input >> kp[ d ];
	return K.uCell( kp );
      }
    };
    struct SCellWriter {
      void operator()( std::ostream& output, const KSpace& K, const SCell & scell )
      {
	CellWriter::operator()( output, K, K.unsigns( scell ) );
	output << " " << K.sSign( scell );
      }
    };
    struct SCellReader {
      SCell operator()( std::istream& input, const KSpace& K )
      {
	Point                 kp;
	typename KSpace::Sign s;
	for ( Dimension d = 0; d < KSpace::dimension; ++d )
	  input >> kp[ d ];
	input >> s;
	return K.sCell( kp, s );
      }
    };

    template <typename Value>
    struct ValueWriter {
      void operator()( std::ostream& output, const Value& v )
      {
	output << " " << v;
      }
      void operator()( std::ostream& output, const std::vector<Value>& vv )
      {
	for ( auto&& v : vv ) output << " " << v;
      }
    };

    template <typename Value>
    struct ValueReader {
      bool operator()( std::istream& input, Value& value )
      {
	std::string str;
	std::getline( input, str );
	// construct a stream from the string
	std::stringstream strstr(str);
	// use stream iterators to copy the stream to the vector as whitespace separated strings
	std::istream_iterator<std::string> it(strstr);
	std::istream_iterator<std::string> end;
	std::vector<std::string> results(it, end);
	std::stringstream sstr( results[ 0 ] );
	sstr >> value;
	return ( results.size() == 1 ) && input.good();
      }
      
      bool operator()( std::istream& input, std::vector<Value>& values )
      {
	std::string str;
	std::getline( input, str );
	// construct a stream from the string
	std::stringstream strstr(str);
	// use stream iterators to copy the stream to the vector as whitespace separated strings
	std::istream_iterator<std::string> it(strstr);
	std::istream_iterator<std::string> end;
	std::vector<std::string> results(it, end);
	values.resize( results.size() );
	for ( unsigned int i = 0; i < results.size(); ++i ) {
	  std::stringstream sstr( results[ i ] );
	  sstr >> values[ i ];
	}
	return input.good();
      }
    };

    // Outputs in \a output a map \a anyMap: SCell -> Value given the
    // appropriate value \a writer.
    //
    // @tparam TSCellMap any model of map SCell -> Value., e.g. std::map<SCell,double>
    // @tparam TValueWriter any model of value writer, e.g. ValueWriter<double>
    //
    // @param[out] output the output stream
    // @param[in]  K the Khalimsky space where cells are defined.
    // @param[in]  anyMap the map associated a value to signed cells.
    // @param[in]  writer the writer that can write values on the ouput
    // stream, e.g. ValueWriter<double> to write double value or
    // vector<double> values.
    template <typename TSCellMap, typename TValueWriter>
    static
    bool outputSCellMapAsCSV
    ( std::ostream&       output,
      const KSpace&       K,
      const TSCellMap&     anyMap,
      const TValueWriter& writer )
    {
      SCellWriter w;
      for ( auto&& v : anyMap ) {
	w( output, K, v.first );
	writer( output, v.second );
	output << std::endl;
      }
      return output.good();
    }
    
    // Outputs in \a output a map \a anyMap: SCell -> value given the
    // appropriate value \a writer.
    //
    // @tparam TCellMap any model of map Cell -> Value., e.g. std::map<Cell,double>
    // @tparam TValueWriter any model of value writer, e.g. ValueWriter<double>
    //
    // @param[out] output the output stream
    // @param[in]  K the Khalimsky space where cells are defined.
    // @param[in]  anyMap the map associated a value to signed cells.
    // @param[in]  writer the writer that can write values on the ouput
    // stream, e.g. ValueWriter<double> to write double value or
    // vector<double> values.
    template <typename TCellMap, typename TValueWriter>
    static
    bool outputCellMapAsCSV
    ( std::ostream&       output,
      const KSpace&       K,
      const TCellMap&      anyMap,
      const TValueWriter& writer )
    {
      CellWriter w;
      for ( auto&& v : anyMap ) {
	w( output, K, v.first );
	writer( output, v.second );
	output << std::endl;
      }
      return output.good();
    }
    
    /// Given a space \a K and an oriented cell \a s, returns its vertices.
    /// @param K any cellular grid space.
    /// @param s any signed cell.
    /// @return the vector of the vertices of s, as unsigned cells of dimension 0.
    static
    CellRange getPrimalVertices( const KSpace& K, const SCell& s )
    {
      auto faces = K.uFaces( K.unsigns( s ) );
      CellRange primal_vtcs;
      for ( auto&& f : faces ) {
	if ( K.uDim( f ) == 0 ) primal_vtcs.push_back( f );
      }
      return primal_vtcs;
    }
    
    /// Given a space \a K and a surfel \a s, returns its vertices in ccw or cw order.
    /// @param K any cellular grid space of dimension 3.
    /// @param s any surfel, a signed cell of dimension 2.
    /// @param ccw when 'true', the order corresponds to a ccw orientation seen from the exterior normal to the surfel, otherwise it is a cw order.
    /// @return the vector of the vertices of s, as unsigned cells of dimension 0.
    /// @note useful when exporting faces to OBJ format. 
    static
    CellRange getPrimalVertices( const KSpace& K, const Surfel& s, bool ccw )
    {
      BOOST_STATIC_ASSERT(( KSpace::dimension == 3 ));
      CellRange vtcs = getPrimalVertices( K, s );
      std::swap( vtcs[ 2 ], vtcs[ 3 ] );
      auto  orth_dir = K.sOrthDir( s );
      auto    direct = K.sDirect( s, orth_dir ) ? ccw : ! ccw;
      Vector    s0s1 = K.uCoords( vtcs[ 1 ] ) - K.uCoords( vtcs[ 0 ] );
      Vector    s0s2 = K.uCoords( vtcs[ 2 ] ) - K.uCoords( vtcs[ 0 ] );
      Vector       t = s0s1.crossProduct( s0s2 );
      if ( ( ( t[ orth_dir ] > 0.0 ) && direct )
	   || ( ( t[ orth_dir ] < 0.0 ) && ! direct ) )
	std::reverse( vtcs.begin(), vtcs.end() );
      return vtcs;
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