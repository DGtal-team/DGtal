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
#include "DGtal/shapes/ShapeGeometricFunctors.h"
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
    typedef LightImplicitDigitalSurface< KSpace, BinaryImage >  SimpleSurfaceContainer;
    /// defines a connected digital surface over a binary image.
    typedef DigitalSurface< SimpleSurfaceContainer >            SimpleDigitalSurface;
    /// defines a heavy container that represents any digital surface.
    typedef SetOfSurfels< KSpace, SurfelSet >                   MultiSurfaceContainer;
    /// defines an arbitrary digital surface over a binary image.
    typedef DigitalSurface< MultiSurfaceContainer >             MultiDigitalSurface;
    /// defines a connected or not indexed digital surface.
    typedef IndexedDigitalSurface< MultiSurfaceContainer >      IdxDigitalSurface;
    typedef typename SimpleDigitalSurface::Surfel               Surfel;
    typedef typename SimpleDigitalSurface::Cell                 Cell;
    typedef typename SimpleDigitalSurface::SCell                SCell;
    typedef typename SimpleDigitalSurface::Vertex               Vertex;
    typedef typename SimpleDigitalSurface::Arc                  Arc;
    typedef typename SimpleDigitalSurface::Face                 Face;
    typedef typename SimpleDigitalSurface::ArcRange             ArcRange;
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

    /// Builds a simple digital surface from a space \a K and a binary image \a bimage.
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
    static CountedPtr<SimpleDigitalSurface>
    makeAnyBigSimpleDigitalSurface
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
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

    /// Returns a vector containing either all the simple digital
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
    static std::vector< CountedPtr<SimpleDigitalSurface> >
    makeSimpleDigitalSurfaces
    ( CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      SurfelRange surfel_reps;
      return makeSimpleDigitalSurfaces( surfel_reps, bimage, K, params );
    }

    /// Returns a vector containing either all the simple digital
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
    static std::vector< CountedPtr<SimpleDigitalSurface> >
    makeSimpleDigitalSurfaces
    ( SurfelRange&            surfel_reps,
      CountedPtr<BinaryImage> bimage,
      const KSpace&           K,
      const Parameters&       params = parametersDigitalSurface() )
    {
      std::vector< CountedPtr<SimpleDigitalSurface> > result;
      std::string component      = params[ "surfaceComponents" ].as<std::string>();
      if ( component == "AnyBig" ) {
	result.push_back( makeAnyBigSimpleDigitalSurface( bimage, K, params ) );
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
      CountedPtr<SimpleDigitalSurface> ptrSurface;
      for ( auto bel : all_surfels )
	{
	  if ( marked_surfels.count( bel ) != 0 ) continue;
	  surfel_reps.push_back( bel );
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

    /// Creates a multi digital surface representing the boundaries in
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
    /// @return a smart pointer on the multi digital surface
    /// representing the boundaries in the binary image.
    static CountedPtr< MultiDigitalSurface >
    makeMultiDigitalSurface
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
      MultiSurfaceContainer* surfContainer
	= new MultiSurfaceContainer( K, surfAdj, all_surfels );
      return CountedPtr< MultiDigitalSurface >
	    ( new MultiDigitalSurface( surfContainer ) ); // acquired
    }


    /// Builds a multi digital surface from an indexed digital surface.
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
    /// @return a smart pointer on a multi digital surface.
    static CountedPtr< MultiDigitalSurface >
    makeMultiDigitalSurface
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
      MultiSurfaceContainer* surfContainer
	= new MultiSurfaceContainer( K, surfAdj, all_surfels );
      return CountedPtr<MultiDigitalSurface>
	( new MultiDigitalSurface( surfContainer ) ); // acquired
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
	  auto simple_surface = makeAnyBigSimpleDigitalSurface( bimage, K, params );
	  surfels.insert( simple_surface->begin(), simple_surface->end() );
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
      CountedPtr<MultiSurfaceContainer> ptrSurfContainer
        ( new MultiSurfaceContainer( K, surfAdj, surfels ) );
      CountedPtr<IdxDigitalSurface> ptrSurface
	( new IdxDigitalSurface() );
      bool ok = ptrSurface->build( ptrSurfContainer );
      if ( !ok )
	trace.warning() << "[Shortcuts::makeIdxDigitalSurface] Error building indexed digital surface." << std::endl;
      return ptrSurface;
    }

    /// Builds an indexed digital surface from a simple digital
    /// surface. Note that the surfel adjacency may be changed and a
    /// connected simple digital surface could be disconnected in the process.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
    ///
    /// @param[in] surface a smart pointer on a simple digital surface.
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

    /// Builds an indexed digital surface from a vector of simple digital
    /// surfaces. Note that the surfel adjacency may be changed and a
    /// connected simple digital surface could be disconnected in the process.
    ///
    /// @note the surfaces must live in the same digital spaces. 
    ///
    /// @param[in] surfaces a vector of smart pointers on simple digital surfaces.
    ///
    /// @param[in] params the parameters:
    ///   - surfelAdjacency   [     1]: specifies the surfel adjacency (1:ext, 0:int)
    ///
    /// @return a smart pointer on the required indexed digital surface.
    static CountedPtr<IdxDigitalSurface>
    makeIdxDigitalSurface
    ( const std::vector< CountedPtr<SimpleDigitalSurface> >& surfaces,
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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

    /// Given a simple digital surface, returns a vector of surfels in
    /// some specified order.
    ///
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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
    ///   - trivialRadius     [   3.0]: the radius used when computing convolved trivial normals.
    static Parameters parametersGeometryEstimation()
    {
      return Parameters
	( "trivialRadius", 3.0 );
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
    ///
    /// @param[in] surface the digital surface
    /// @param[in] surfels the sequence of surfels at which we compute the normals
    /// @param[in] params the parameters:
    ///   - trivialRadius     [   3.0]: the radius used when computing convolved trivial normals.
    ///
    /// @return the vector containing the estimated normals, in the
    /// same order as \a surfels.
    template <typename TAnyDigitalSurface>
    static RealVectors
    getConvolvedTrivialNormalVectors
    ( CountedPtr<TAnyDigitalSurface> surface,
      const SurfelRange&             surfels,
      const Parameters&               params = parametersGeometryEstimation() )
    {
      Scalar t = params[ "trivialRadius" ].as<double>();
      typedef typename TAnyDigitalSurface::DigitalSurfaceContainer  SurfaceContainer;
      typedef ExactPredicateLpSeparableMetric<Space,2>              Metric;
      typedef functors::HatFunction<Scalar>                         Functor;
      typedef functors::ElementaryConvolutionNormalVectorEstimator
	< Surfel, CanonicSCellEmbedder<KSpace> >                    SurfelFunctor;
      typedef LocalEstimatorFromSurfelFunctorAdapter
	< SurfaceContainer, Metric, SurfelFunctor, Functor>         NormalEstimator;
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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
    /// @tparam TAnyDigitalSurface either kind of DigitalSurface, like Shortcuts::SimpleDigitalSurface or Shortcuts::MultiDigitalSurface.
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
      auto multi_surface = makeMultiDigitalSurface( surface, params );
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
					    multi_surface, embedder, params );
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
