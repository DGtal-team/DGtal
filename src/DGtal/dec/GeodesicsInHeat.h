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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/09/21
 *
 * Header file for module GeodesicsInHeat.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GeodesicsInHeat_RECURSES)
#error Recursive header files inclusion detected in GeodesicsInHeat.h
#else // defined(GeodesicsInHeat_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GeodesicsInHeat_RECURSES

#if !defined GeodesicsInHeat_h
/** Prevents repeated inclusion of headers. */
#define GeodesicsInHeat_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/math/linalg/DirichletConditions.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class GeodesicsInHeat
  /**
   * Description of template class 'GeodesicsInHeat' <p>
   * \brief This class implements @cite Crane2013 on polygonal surfaces  (using @ref modulePolygonalCalculus).
   *
   * see @ref moduleGeodesicsInHeat for details and examples.
   *
   * @tparam a model of PolygonalCalculus.
   */
  template <typename TPolygonalCalculus>
  class GeodesicsInHeat
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TPolygonalCalculus PolygonalCalculus;
    typedef typename PolygonalCalculus::SparseMatrix SparseMatrix;
    typedef typename PolygonalCalculus::DenseMatrix DenseMatrix;
    typedef typename PolygonalCalculus::Solver Solver;
    typedef typename PolygonalCalculus::Vector Vector;
    typedef typename PolygonalCalculus::Vertex Vertex;
    typedef typename PolygonalCalculus::LinAlg LinAlgBackend;
    typedef DirichletConditions< LinAlgBackend > Conditions;
    typedef typename Conditions::IntegerVector IntegerVector;
    
    /**
     * Default constructor.
     */
    GeodesicsInHeat() = delete;
    
    /// Constructor from an existing polygonal calculus. T
    /// @param calculus a instance of PolygonalCalculus
    GeodesicsInHeat(ConstAlias<PolygonalCalculus> calculus): myCalculus(&calculus)
    {
      myIsInit=false;
    }
    
    /**
     * Destructor.
     */
    ~GeodesicsInHeat() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GeodesicsInHeat ( const GeodesicsInHeat & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    GeodesicsInHeat ( GeodesicsInHeat && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GeodesicsInHeat & operator= ( const GeodesicsInHeat & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    GeodesicsInHeat & operator= ( GeodesicsInHeat && other ) = delete;


    
    // ----------------------- Interface --------------------------------------
   
    /// Initialize the solvers with @a dt as timestep for the heat
    /// diffusion and @a lambda parameter for the polygonal calculus,
    /// which guarantee definiteness for positive @a lambda.
    ///
    /// @param dt the timestep
    /// @param lambda timestep
    ///
    /// @param boundary_with_mixed_solution when 'true' and when the
    /// surface has boundaries, mix two solutions of the heat
    /// diffusion operation (Neumann and Dirichlet null conditions on
    /// boundary).
    void init( double dt, double lambda = 1.0,
               bool boundary_with_mixed_solution = false  )
    {
      myIsInit = true;
      myLambda = lambda;

      SparseMatrix laplacian = myCalculus->globalLaplaceBeltrami( lambda );
      SparseMatrix mass      = myCalculus->globalLumpedMassMatrix();
      myHeatOpe              = mass - dt*laplacian;
    
      // from https://geometry-central.net
      // NOTE: In theory, it should not be necessary to shift the Laplacian: the Polydec Laplace is always PSD. However, when the
      // matrix is only positive SEMIdefinite, some solvers may not work (ie Eigen's Cholesky solver doesn't work, but
      // Suitesparse does).
      SparseMatrix Id = SparseMatrix(myCalculus->nbVertices(),myCalculus->nbVertices());
      Id.setIdentity();
      laplacian += 1e-6 * Id;
      
      //Prefactorizing
      myPoissonSolver.compute( laplacian );
      myHeatSolver.compute   ( myHeatOpe );
      
      //empty source
      mySource    = Vector::Zero(myCalculus->nbVertices());

      // Manage boundaries
      myManageBoundary = false;
      if ( ! boundary_with_mixed_solution ) return;
      myBoundary = IntegerVector::Zero(myCalculus->nbVertices());
      const auto surfmesh = myCalculus->getSurfaceMeshPtr();
      const auto edges    = surfmesh->computeManifoldBoundaryEdges();
      for ( auto e : edges )
        {
          const auto vtcs = surfmesh->edgeVertices( e );
          myBoundary[ vtcs.first  ] = 1;
          myBoundary[ vtcs.second ] = 1;
        }
      myManageBoundary = ! edges.empty();
      if ( ! myManageBoundary ) return;
      // Prepare solver for a problem with Dirichlet conditions.
      SparseMatrix heatOpe_d = Conditions::dirichletOperator( myHeatOpe, myBoundary );
      // Prefactoring
      myHeatDirichletSolver.compute( heatOpe_d );
    }
    
    /** Adds a source point at a vertex @e aV
     * @param aV the Vertex
     **/
    void addSource(const Vertex aV)
    {
      ASSERT_MSG(aV < myCalculus->nbVertices(), "Vertex is not in the surface mesh vertex range");
      myLastSourceIndex = aV;
      mySource( aV ) = 1.0;
    }
    
    /** Removes all source Diracs.
     */
    void clearSource()
    {
      mySource = Vector::Zero(myCalculus->nbVertices());
    }
    
    /**
     * @returns the source point vector.
     **/
    Vector source() const
    {
      FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
      return mySource;
    }
    
    
    /// Main computation of the Geodesic In Heat
    /// @returns the estimated geodesic distances from the sources.
    Vector compute() const
    {
      FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
      //Heat diffusion
      Vector heatDiffusion = myHeatSolver.solve(mySource);
      ASSERT(myHeatSolver.info()==Eigen::Success);

      // Take care of boundaries
      if ( myManageBoundary )
        {
          Vector bValues  = Vector::Zero( myCalculus->nbVertices() );
          Vector bSources = Conditions::dirichletVector( myHeatOpe, mySource,
                                                         myBoundary, bValues );
          Vector bSol     = myHeatDirichletSolver.solve( bSources );
          Vector heatDiffusionDirichlet
                          = Conditions::dirichletSolution( bSol, myBoundary, bValues );
          heatDiffusion = 0.5 * ( heatDiffusion + heatDiffusionDirichlet );
        }
      Vector divergence    = Vector::Zero(myCalculus->nbVertices());
      auto cpt=0;
      auto surfmesh = myCalculus->getSurfaceMeshPtr();
      
      // Heat, normalization and divergence per face
      for(typename PolygonalCalculus::MySurfaceMesh::Index f=0; f< myCalculus->nbFaces(); ++f)
        {
          Vector faceHeat( myCalculus->degree(f));
          cpt=0;
          auto vertices = surfmesh->incidentVertices(f);
          for(auto v: vertices)
            {
              faceHeat(cpt) = heatDiffusion( v );
              ++cpt;
            }
          // ∇heat / ∣∣∇heat∣∣
          Vector grad = -myCalculus->gradient(f) * faceHeat;
          grad.normalize();
      
          // div
          DenseMatrix   oneForm = myCalculus->flat(f)*grad;
          Vector divergenceFace = myCalculus->divergence( f ) * oneForm;
          cpt=0;
          for(auto v: vertices)
            {
              divergence(v) += divergenceFace(cpt);
              ++cpt;
            }
        }
      
      // Last Poisson solve
      Vector distVec = myPoissonSolver.solve(divergence);
      ASSERT(myPoissonSolver.info()==Eigen::Success);

      //Source val
      auto sourceval = distVec(myLastSourceIndex);
      //shifting the distances to get 0 at sources
      return distVec - sourceval*Vector::Ones(myCalculus->nbVertices());
    }
    
    
    /// @return true if the calculus is valid.
    bool isValid() const
    {
      return myIsInit && myCalculus->isValid();
    }
    
    // ----------------------- Private --------------------------------------

  private:
    
    ///The underlying PolygonalCalculus instance
    const PolygonalCalculus *myCalculus;

    /// The operator for heat diffusion.
    SparseMatrix myHeatOpe;
    
    ///Poisson solver
    Solver myPoissonSolver;

    ///Heat solver
    Solver myHeatSolver;

    ///Source vector
    Vector mySource;

    ///Vertex index to the last source point (to shift the distances)
    Vertex myLastSourceIndex;
  
    ///Validitate flag
    bool myIsInit;

    /// Lambda parameter
    double myLambda;

    /// When 'true', manage boundaries with a mixed solution of
    /// Neumann and Dirichlet conditions.
    bool myManageBoundary;

    /// The boundary characteristic vector
    IntegerVector myBoundary;
    
    ///Heat solver with Dirichlet boundary conditions.
    Solver myHeatDirichletSolver;
  
  
  }; // end of class GeodesicsInHeat
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GeodesicsInHeat_h

#undef GeodesicsInHeat_RECURSES
#endif // else defined(GeodesicsInHeat_RECURSES)
