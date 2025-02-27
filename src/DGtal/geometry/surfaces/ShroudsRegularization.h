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
 * @file ShroudsRegularization.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/06/09
 *
 * Header file for module ShroudsRegularization.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ShroudsRegularization_RECURSES)
#error Recursive header files inclusion detected in ShroudsRegularization.h
#else // defined(ShroudsRegularization_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ShroudsRegularization_RECURSES

#if !defined ShroudsRegularization_h
/** Prevents repeated inclusion of headers. */
#define ShroudsRegularization_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/topology/IndexedDigitalSurface.h"
#include "DGtal/topology/DigitalSurface2DSlice.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal {
  /// Description of template class 'ShroudsRegularization' <p>
  ///
  /// \brief Aim: Implements the Shrouds Regularization algorithm of
  /// Nielson et al \cite nielson2003shrouds.
  ///
  /// Represents a decomposition of a closed digital surface into
  /// three stacks of slices, one per dimension. Gives methods to
  /// optimize the positions of vertices and smooth the resulting
  /// surface, generally according to curvature-related
  /// regularizers.
  ///
  /// @see \ref moduleShrouds
  ///
  /// @note This method is limited to \b closed digital surfaces.
  ///
  /// @tparam TDigitalSurfaceContainer any digital surface container
  /// (a model concepts::CDigitalSurfaceContainer), for instance a
  /// SetOfSurfels.
  ///
  /// @see testShroudsRegularization.cpp
  template <concepts::CDigitalSurfaceContainer TDigitalSurfaceContainer>
  class ShroudsRegularization
  {
  public:
    typedef TDigitalSurfaceContainer           Container;
    typedef ShroudsRegularization< Container > Self;
    typedef typename Container::KSpace         KSpace;
    typedef typename KSpace::Space             Space;
    typedef typename Space::RealVector         RealVector;
    typedef typename Space::RealPoint          RealPoint;
    typedef typename RealVector::Component     Scalar;
    typedef IndexedDigitalSurface< Container > IdxDigitalSurface;
    typedef typename IdxDigitalSurface::Vertex IdxVertex;
    typedef typename IdxDigitalSurface::Surfel IdxSurfel;
    typedef IdxVertex                          Vertex;
    typedef std::vector< IdxSurfel >           IdxSurfelRange;
    typedef std::vector< Scalar >              Scalars;
    typedef std::vector< RealVector >          RealVectors;
    typedef std::vector< RealPoint >           RealPoints;

    /// The enum class specifying the possible shrouds regularization.
    enum class Regularization { AREA, SNAKE, SQUARED_CURVATURE };
    
    // ----------------------- Standard services ------------------------------
  public:
    /// @name Standard services (construction, initialization)
    /// @{
    
    /// Default constructor. The object is not valid.
    ShroudsRegularization()
      : myPtrIdxSurface( nullptr ), myPtrK( nullptr )
    {}
    
    /// Constructor from (closed) \a surface.
    /// Also calls methods \ref precomputeTopology and \ref init.
    /// @param surface a counted pointer on an indexed digital surface.
    ///
    /// @note Complexity is linear in the number of surfels of \a surface.
    ShroudsRegularization( CountedPtr< IdxDigitalSurface > surface )
      : myPtrIdxSurface( surface ),
	myPtrK( &surface->container().space() ),
	myEpsilon( 0.0001 ), myAlpha( 1.0 ), myBeta( 1.0 )
    {
      precomputeTopology();
      init();
    }

    /// Prepares the shroud for optimization. Must be called before
    /// any call to \ref regularize, \ref oneStepAreaMinimization,
    /// \ref oneStepSnakeMinimization, 
    /// \ref oneStepSquaredCurvatureMinimization.
    ///
    /// @note Complexity is linear in the number of surfels of \a surface.
    void init()
    {
      const auto embedder = CanonicSCellEmbedder<KSpace>( *myPtrK );
      const auto nbV      = myPtrIdxSurface->nbVertices();
      myT.resize( nbV );
      myInsV.resize( nbV );
      myOutV.resize( nbV );
      for ( Dimension i = 0; i < 3; ++i )
	{
	  myPrevD[ i ].resize( nbV );
	  myNextD[ i ].resize( nbV );
	}
      for ( Vertex v = 0; v < myT.size(); ++v )
	{
	  const auto s = myPtrIdxSurface->surfel( v );
	  const auto k = myPtrK->sOrthDir( s );
	  myInsV[ v ]  = embedder( myPtrK->sDirectIncident( s, k ) );
	  myOutV[ v ]  = embedder( myPtrK->sIndirectIncident( s, k ) );
	}
    }

    /// Sets some parameters that affect the output regularized shape.
    ///
    /// @param eps the bounds for varying the positions of vertices in ]0,1[
    /// @param alpha parameter for Snake first order regularization (~ area)
    /// @param beta  parameter for Snake second order regularization (~ curvature)
    void setParams( double eps, double alpha = 1.0, double beta = 1.0 )
    {
      myEpsilon = eps;
      myAlpha   = alpha;
      myBeta    = beta;
    }
    /// Retrieves the parameters that affect the output regularized shape.
    ///
    /// @return a tuple (eps, alpha, beta) where \a eps is the bounds
    /// for varying the positions of vertices in ]0,1[, \a alpha is
    /// the parameter for Snake first order regularization (~ area),
    /// \a beta is the parameter for Snake second order regularization
    /// (~ curvature).
    std::tuple< double, double, double > getParams() const
    {
      return std::make_tuple( myEpsilon, myAlpha, myBeta );
    }
    
    /// @}
    
    // ----------------------- Accessor services ------------------------------
  public:
    /// @name Accessor services
    /// @{
    
    /// @param v any valid vertex.
    /// @param t some adjustement parameter
    /// @return the position of vertex v for this parameter t.
    RealPoint position( const Vertex v, const double t ) const
    {
      return (1-t) * myInsV[ v ] + t * myOutV[ v ];
    }
    
    /// @param v any valid vertex.
    /// @return its position.
    RealPoint position( const Vertex v ) const
    {
      const auto t = myT[ v ];
      return (1-t) * myInsV[ v ] + t * myOutV[ v ];
    }
    
    /// @return the vector of vertex positions
    RealPoints positions() const
    {
      RealPoints result( myT.size() );
      for ( Vertex v = 0; v < myT.size(); ++v )
	result[ v ] = position( v );
      return result;
    }

    /// Useful to find the two tangent directions to a given vertex
    /// (i.e. `(orthDir(v)+1)%3` and `(orthDir(v)+2)%3`).
    ///
    /// @return the orthogonal direction to vertex v.
    Dimension orthDir( const Vertex v ) const
    {
      return myOrthDir[ v ];
    }
    
    /// Useful to navigate tangentially along a slice.
    /// @param v_i a pair (vertex,tangent direction)
    /// @return the next vertex and associated tangent direction along the slice. 
    std::pair<Vertex,Dimension> next( const std::pair<Vertex,Dimension> & v_i ) const
    {
      const Vertex    vn = myNext[ v_i.second ][ v_i.first ];
      const Dimension in = myOrthDir[ vn ] == v_i.second
	? myOrthDir[ v_i.first ] : v_i.second;
      return std::make_pair( vn, in );
    }

    /// Useful to navigate tangentially along a slice.
    /// @param v_i a pair (vertex,tangent direction)
    /// @return the previous vertex and associated tangent direction along the slice. 
    std::pair<Vertex,Dimension> prev( const std::pair<Vertex,Dimension> &v_i ) const
    {
      const Vertex    vp = myPrev[ v_i.second ][ v_i.first ];
      const Dimension ip = myOrthDir[ vp ] == v_i.second
	? myOrthDir[ v_i.first ] : v_i.second;
      return std::make_pair( vp, ip );
    }
    
    /// @}

    // ----------------------- Geometric services ------------------------------
  public:
    /// @name Geometric services
    /// @{

    /// Computes the distances between the vertices along slices.
    void parameterize()
    {
      Scalar d = 0.0;
      // Vertex n = 0;
      for ( Dimension i = 0; i < 3; ++i )
	for ( Vertex v = 0; v < myT.size(); ++v )
	  {
	    if ( myNext[ i ][ v ] == myInvalid )  continue; // not a valid slice
	    myNextD[ i ][ v ] = ( position( myNext[ i ][ v ] ) - position( v ) ).norm();
	    myPrevD[ i ][ v ] = ( position( myPrev[ i ][ v ] ) - position( v ) ).norm();
	    d += myNextD[ i ][ v ];
	    //	    n += 1;
	  }
    }

    /// @param v_i a pair (vertex,tangent direction)
    /// @return the coefficient for centered first-order finite difference.
    /// @note We have y'_i ~= c1 * (y_{i+1} - y_{i-1} )
    Scalar c1( const std::pair<Vertex,Dimension> &v_i ) const
    {
      const Scalar din = myNextD[ v_i.second ][ v_i.first ];
      const Scalar dip = myPrevD[ v_i.second ][ v_i.first ];
      return 1.0 / (din + dip);
    }
    
    /// @param v_i a pair (vertex,tangent direction)
    /// @return the coefficients for centered second-order finite difference.
    /// @note We have y''_i ~= c2<0> * y_{i+1} - c2<1> * y_{i} + c2<2> * y_{i-1}
    std::tuple<Scalar,Scalar,Scalar> c2_all( const std::pair<Vertex,Dimension> &v_i ) const
    {
      const Scalar din = myNextD[ v_i.second ][ v_i.first ];
      const Scalar dip = myPrevD[ v_i.second ][ v_i.first ];
      return std::make_tuple( 2.0 / ( din * ( din + dip ) ),
			      2.0 / ( din * dip ),
			      2.0 / ( dip * ( din + dip ) ) );
    }
    
    /// @}

    // -------------------------- regularization services ----------------------------
  public:
    /// @name Regularization services
    /// @{

    /// Generic method for shrouds regularization.
    ///
    /// @param reg your choice of regularization in AREA (not nice in
    /// 3D), SNAKE (not so bad), SQUARED_CURVATURE (works best)
    ///
    /// @param randomization if greater than 0.0 add some perturbation to
    /// the solution. May be used for quitting a local minima.
    ///
    /// @param max_loo the maximum \f$ l_\infty \f$-norm of one step
    /// of regularization before stopping the regularization.
    ///
    /// @param maxNb the maximum number of optimization steps.
    ///
    /// @see oneStepAreaMinimization
    /// @see oneStepSnakeMinimization
    /// @see oneStepSquaredCurvatureMinimization
    std::pair<double,double>
    regularize( const Regularization   reg = Regularization::SQUARED_CURVATURE,
		const double randomization = 0.0,
		const double       max_loo = 0.0001,
		const int            maxNb = 100 )
    {
      double loo      = 0.0;
      double  l2      = 0.0;
      int     nb      = 0;
      double   r      = 0.5;
      (void)randomization; //parameter not used, avoiding warning
      do {
	std::tie( loo, l2 ) =
	  reg == Regularization::SQUARED_CURVATURE
	  ? oneStepSquaredCurvatureMinimization( r )
	  : reg == Regularization::SNAKE
	  ? oneStepSnakeMinimization( myAlpha, myBeta, r )
	  : oneStepAreaMinimization( r );
	if ( nb % 50 == 0 )
	  trace.info() << "[Shrouds iteration " << nb
		       << " E=" << energy( reg )
		       << "] dx <= " << loo << " l2=" << l2 << std::endl;
	r  *= 0.9;
	nb += 1;
      } while ( loo > max_loo && nb < maxNb );
      return std::make_pair( loo, l2 );
    }

    /// @param reg your choice of regularization in AREA (not nice in
    /// 3D), SNAKE (not so bad), SQUARED_CURVATURE (works best)
    ///
    /// @return the current energy of the shrouds, according to the
    /// chosen regularization process.
    double energy( const Regularization reg = Regularization::SQUARED_CURVATURE )
    {
      if ( reg == Regularization::SQUARED_CURVATURE )
	return energySquaredCurvature();
      else if ( reg == Regularization::SNAKE )
        return energySnake();
      else
	return energyArea();
    }

    /// @return the current energy associated to the squared curvature
    /// regularization process.
    double energySquaredCurvature();

    /// @return the current energy associated to the snake
    /// regularization process.
    double energySnake();

    /// @return the current energy associated to the area
    /// regularization process.
    double energyArea();
    
    /// Smooths the shape according to the minimization of area.
    ///
    /// @param randomization if greater than 0.0 add some perturbation to
    /// the solution. May be used for quitting a local minima.
    ///
    /// @return the pair of \f$ l_\infty \f$ and \f$ l_2 \f$ norms of
    /// vertex displacements.
    ///
    /// @note Not very nice.
    std::pair<double,double> oneStepAreaMinimization( const double randomization = 0.0 );

    /// Smooths the shape according to the minimization of elastic + thin plate
    /// energy (like snakes).
    ///
    /// \f[
    /// E^{snk}(C) = \int_C \alpha (x'(s)^2 + y'(s)^2) + \beta (x''(s)^2 + y''(s)^2) ds, 
    /// \f]
    ///
    /// for \f$ C=(x(s),y(s)) \f$ and boundary constraints.
    ///
    /// @param alpha parameter for first order regularization (~ area)
    /// @param beta  parameter for second order regularization (~ curvature)
    /// @param randomization if greater than 0.0 add some perturbation to
    /// the solution. May be used for quitting a local minima.
    ///
    /// @return the pair of \f$ l_\infty \f$ and \f$ l_2 \f$ norms of
    /// vertex displacements.
    ///
    /// Euler-Lagrange equations leads to necessary conditions:
    /// for all s, \f$ x''''(s) = 0 \f$ and \f$ y''''(s) = 0 \f$.
    ///
    /// A vertex \f$ v(s) \f$ has a position \f$ X \f$. Next and previous vertices are
    /// denoted by \f$ Xn, Xnn, Xp, Xpp \f$. If i the tangent direction and k the
    /// orthogonal direction at \f$ X \f$, then \f$ x(s)=X[i], y(s)=X[k] \f$.
    ///
    /// At a vertex \f$ v(s) \f$, the position is parameterized by \a t. Varying
    /// \a t only modifies the vertical position \f$ X[k] \f$, i.e. \a y.
    /// So \f$ y(t) = (1-t)*I[v][k] + t*O[v][k] \f$
    /// (for \f$ I[v] \f$ and \f$ O[v] \f$ inside and outside voxel positions).
    ///
    /// \f$ x' ~= ( Xn[i] - Xp[i] ) / (d(X,Xn)+d(X,Xp) \f$ does not depend on t.
    /// \f$ y' ~= ( Xn[k] - Xp[k] ) / (d(X,Xn)+d(X,Xp) \f$ does not depend on t.
    /// \f$ x'' ~= ( c_0 * Xn[i] - c_1 * X[i] + c_2 * Xp[i] ) \f$ does not depend on t.
    ///
    /// for \f$ c_0, c_1, c_2 \f$ constants depending on \f$ d(X,Xn)
    /// \f$ and \f$ d(X,Xp) \f$.  \f$ y'' ~= ( c_0 * Xn[k] - c_1 *
    /// X[k] + c_2 * Xp[k] ) \f$ solely \b depend on t.
    std::pair<double,double> oneStepSnakeMinimization
    ( const double alpha = 1.0, const double beta = 1.0, const double randomization = 0.0 );

    /// Smooths the shape according to the minimization of squared curvature.
    ///
    /// \f[
    /// E^{\kappa^2}(C) = \int_C (x'(s) y''(s) + x''(s) y'(s))^2 / (x'(s)^2 + y'(s)^2)^3 ds,
    /// \f]
    ///
    /// for \f$ C=(x(s),y(s)) \f$ and boundary constraints.
    ///
    /// @param randomization if greater than 0.0 add some perturbation to
    /// the solution. May be used for quitting a local minima.
    ///
    /// @return the pair of \f$ l_\infty \f$ and \f$ l_2 \f$ norms of
    /// vertex displacements.
    ///
    /// Euler-Lagrange equations leads to rather complex necessary
    /// conditions.  At each position (we omit parameter \a s for making
    /// things more readable):
    ///
    /// \f[ 24*x'^3*x''^3*y' 
    /// + x'''*( - 13*x'^4*x''*y' - 14*x'^2*x''*y'^3 - x''*y'^5 )
    /// + y'''*( 8*x'^5*x'' + 4*x'^3*x''*y'^2 - 4*x'*x''*y'^4 )
    /// + x''''*( x'^5*y' + 2*x'^3*y'^3 + x'*y'^5 )
    /// + y''*( 12*x'^4*y'*y''' + 12*x'^2*y'^3*y''' - 24*x'^4*x''^2 + 5*x'^5*x''' + 51*x'^2*x''^2*y'^2 - 2*x'^3*x'''*y'^2 + 3*x''^2*y'^4 - 7*x'*x'''*y'^4 )
    /// + y''^2*( -54*x'^3*x''*y' + 18*x'*x''*y'^3 )
    /// + y''^3*( 3*x'^4 - 21*x'^2*y'^2 )
    /// + y''''*( - x'^6 - 2*x'^4*y'^2 - x'^2*y'^4 )
    /// == 0 \f]
    ///
    /// A vertex \f$ v(s) \f$ has a position \f$ X \f$. Next and previous vertices are
    /// denoted by \f$ Xn, Xnn, Xp, Xpp \f$. If \a i the tangent direction and \a k the
    /// orthogonal direction at \f$ X \f$, then \f$ x(s)=X[i], y(s)=X[k] \f$.
    ///
    /// At a vertex \f$ v(s) \f$, the position is parameterized by \a t. Varying
    /// \a t only modifies the vertical position \f$ X[k] \f$, i.e. \a y.
    /// So \f$ y(t) = (1-t)*I[v][k] + t*O[v][k] \f$
    /// (for \f$ I[v] \f$ and \f$ O[v] \f$ inside and outside voxel positions).
    ///
    /// The Euler-Lagrange are factorized as above in order to isolate
    /// at best the parameter \a t of vertex \f$ v(s) \f$.
    ///
    /// - The first four lines do not depend on \a t (except \f$ y''' \f$ but it is
    ///  neglected and the old value is used).
    /// - \f$ y''^2 \f$ and \f$ y''^3 \f$ are linarized as \f$ y'' * y''_{old} \f$ and \f$ y'' * y''_{old}^2 \f$
    /// - \f$ y'''' ~= ( c_0 * Yn - c_1 * y'' + c_2 * Yp ) \f$
    /// - all \f$ y'' \f$ are sumed up, while the rest gives a constant.
    ///
    /// \f$ x' ~= ( Xn[i] - Xp[i] ) / (d(X,Xn)+d(X,Xp) \f$ does not depend on \a t.
    /// \f$ y' ~= ( Xn[k] - Xp[k] ) / (d(X,Xn)+d(X,Xp) \f$ does not depend on \a t.
    /// \f$ x'' ~= ( c_0 * Xn[i] - c_1 * X[i] + c_2 * Xp[i] ) \f$ does not depend on \a t.
    ///
    /// for \f$ c_0, c_1, c_2 \f$ constants depending on \f$ d(X,Xn) \f$ and \f$ d(X,Xp) \f$.
    /// \f$ y'' ~= ( c_0 * Xn[k] - c_1 * X[k] + c_2 * Xp[k] ) \f$ solely \b depend on t.
    /// where \f$ X[k] = y(t) = (1-t)*I[v][k] + t*O[v][k]. \f$
    /// And so on.
    std::pair<double,double> oneStepSquaredCurvatureMinimization
    ( const double randomization = 0.0 );

    /// Forces t to stay in ]0,1[
    void enforceBounds();

    /// @}
    
    // -------------------------- internal methods ------------------------------
  protected:
    /// @name Internal methods
    /// @{
    
    /// This method precomputes the neighbors of each vertex along each
    /// crossing curves. Must be called at shroud initialization.
    void precomputeTopology()
    {
      typedef typename Container::Tracker      Tracker;
      typedef DigitalSurface2DSlice< Tracker > Slice;
      myT       = Scalars( myPtrIdxSurface->nbVertices(), 0.5 );
      myInvalid = myT.size();
      myOrthDir.resize( myT.size() );
      for ( Dimension i = 0; i < 3; ++i )
      {
	myNext[ i ] = std::vector<Vertex>( myT.size(), myInvalid );
	myPrev[ i ] = std::vector<Vertex>( myT.size(), myInvalid );
      }
      // for each vertex, extracts its two slices.
      for ( Vertex v = 0; v < myT.size(); ++v )
	{
	  auto surf      = myPtrIdxSurface->surfel( v );
	  Dimension k    = myPtrK->sOrthDir( surf );
	  myOrthDir[ v ] = k;
	  for ( Dimension i = 0; i < 3; ++i )
	    {
	      if ( k == i )   continue; // not a valid slice
	      if ( myNext[ i ][ v ] != myInvalid ) continue; // already computed
	      Tracker* tracker = myPtrIdxSurface->container().newTracker( surf );
	      Slice    slice( tracker, i );
	      if ( ! slice.isClosed() ) {
		trace.error() << "[ShroudsRegularization::precomputeTopology]"
			      << " Shrouds works solely on closed surfaces."
			      << std::endl;
		return;
	      }
	      auto start = slice.cstart();
	      auto  next = start;
	      auto  prev = next++;
	      Vertex  vp = v;
	      do
		{
		  auto sp = *prev;
		  auto sn = *next;
		  Dimension in = myPtrK->sOrthDir( sn ) == k ? i : k;
		  Dimension ip = myPtrK->sOrthDir( sp ) == k ? i : k;
		  Vertex vn = myPtrIdxSurface->getVertex( sn );
		  myNext[ ip ][ vp ] = vn;
		  myPrev[ in ][ vn ] = vp;
		  prev = next++;
		  vp   = vn;
		}
	      while ( prev != start );	    
	      delete tracker;
	    }
	}
    }

    /// @}
    
    // -------------------------- data ---------------------------------
  private:

    /// the indexed digital surface (internal surface representation).
    CountedPtr<IdxDigitalSurface>      myPtrIdxSurface;
    /// A const pointer to the cellular space in which lives the digital surface.
    const KSpace*                      myPtrK;
    /// The limiting bounds for the displacement of vertices along
    /// their unit dual edge: \f$ \lbrack \epsilon, 1-\epsilon \rbrack \f$
    Scalar                             myEpsilon;
    /// The alpha parameter for Snake first order regularization (~ area)
    Scalar                             myAlpha;
    /// The beta parameter for Snake second order regularization (~ curvature)
    Scalar                             myBeta;
    /// the index of the invalid vertex.
    Vertex                             myInvalid;
    /// the vector of vertex displacements along their dual edge (each
    /// value lies in \f$ \lbrack \epsilon, 1-\epsilon \rbrack \f$),
    /// which is optimized by the shrouds algorithm.
    Scalars                            myT;
    /// the vector of dual points lying inside (inside extremity of
    /// each dual edge).
    RealPoints                         myInsV;
    /// the vector of dual points lying outside (outside extremity of
    /// each dual edge).
    RealPoints                         myOutV;
    /// the direction axis of each dual edge.
    std::vector<Dimension>             myOrthDir;
    /// for each vertex, its successor on the slice of given axis direction.
    std::vector<Vertex>                myNext[ 3 ];
    /// for each vertex, its predessor on the slice of given axis direction.
    std::vector<Vertex>                myPrev[ 3 ];
    /// for each vertex, the estimated distance to its successor on
    /// the slice of given axis direction.
    Scalars                            myNextD[ 3 ];
    /// for each vertex, the estimated distance to its predessor on
    /// the slice of given axis direction.
    Scalars                            myPrevD[ 3 ];
    
  }; // end of class ShroudsRegularization
  
  /// Helper function for constructing a ShroudsRegularization from a
  /// (closed) \a surface.
  ///
  /// @tparam TDigitalSurfaceContainer any digital surface container
  /// (a model concepts::CDigitalSurfaceContainer), for instance a
  /// SetOfSurfels.
  ///
  /// @param surface a counted pointer on an indexed digital surface.
  /// @param eps the bounds for varying the positions of vertices in ]0,1[
  ///
  /// @note Complexity is linear in the number of surfels of \a surface.
  /// @see testShroudsRegularization.cpp
  template < typename TDigitalSurfaceContainer > 
  ShroudsRegularization<TDigitalSurfaceContainer>
  makeShroudsRegularization
  ( CountedPtr< IndexedDigitalSurface< TDigitalSurfaceContainer > > surface,
    double eps = 0.00001 )
  {
    return ShroudsRegularization<TDigitalSurfaceContainer>( surface, eps );
  }
  
} // namespace surfaces


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#include "DGtal/geometry/surfaces/ShroudsRegularization.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ShroudsRegularization_h

#undef ShroudsRegularization_RECURSES
#endif // else defined(ShroudsRegularization_RECURSES)
