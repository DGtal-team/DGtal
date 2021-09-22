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
 * Header file for module GeodesicInHeat.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GeodesicInHeat_RECURSES)
#error Recursive header files inclusion detected in GeodesicInHeat.h
#else // defined(GeodesicInHeat_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GeodesicInHeat_RECURSES

#if !defined GeodesicInHeat_h
/** Prevents repeated inclusion of headers. */
#define GeodesicInHeat_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GeodesicInHeat
  /**
   * Description of template class 'GeodesicInHeat' <p>
   * \brief Aim:
   */
  template <typename TPolygonalCalculus>
  class GeodesicInHeat
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TPolygonalCalculus PolygonalCalculus;
    typedef typename PolygonalCalculus::SparseMatrix SparseMatrix;
    typedef typename PolygonalCalculus::DenseMatrix DenseMatrix;
    typedef typename PolygonalCalculus::Solver Solver;
    typedef typename PolygonalCalculus::Vector Vector;
    typedef typename PolygonalCalculus::Vertex Vertex;

    /**
     * Default constructor.
     */
    GeodesicInHeat() = delete;
    
    /// Constructor from an existing polygonal calculus. T
    /// @param calculus a instance of PolygonalCalculus
    GeodesicInHeat(ConstAlias<PolygonalCalculus> calculus): myCalculus(&calculus)
    {
      myIsInit=false;
    }
    
    /**
     * Destructor.
     */
    ~GeodesicInHeat() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GeodesicInHeat ( const GeodesicInHeat & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    GeodesicInHeat ( GeodesicInHeat && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GeodesicInHeat & operator= ( const GeodesicInHeat & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    GeodesicInHeat & operator= ( GeodesicInHeat && other ) = delete;


    
    // ----------------------- Interface --------------------------------------
   
    /// Initialize the solvers with @a dt as timestep for the
    /// heat diffusion
    /// @param dt timestep
    void init(double dt)
    {
      myIsInit=true;
      
      SparseMatrix laplacian = myCalculus->globalLaplaceBeltrami();
      SparseMatrix mass      = myCalculus->globalLumpedMassMatrix();
      SparseMatrix heatOpe   = mass + dt*laplacian;
      
      //Prefactorizing
      myPoissonSolver.compute(laplacian);
      myHeatSolver.compute(heatOpe);
      
      //empty source
      mySource    = Vector::Zero(myCalculus->nbVertices());
    }
    
    /// Adds a source point at a vertex @e aV
    /// @param aV the Vertex
    /// @param dirac the value (def=1.0).
    void addSource(const Vertex aV, double dirac=1.0)
    {
      ASSERT_MSG(aV < myCalculus->nbVertices(), "Vertex not in the surface mesh vertex range");
      mySource( aV ) = dirac;
    }
    
    /// @return the source point vector.
    Vector source() const
    {
      return mySource;
    }
    
    
    /// Main computation of the Geodesic In Heat
    /// @returns the estimated geodesic distance from the sources.
    Vector compute() const
    {
      FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
      //Heat diffusion
      Vector heatDiffusion = myHeatSolver.solve(mySource);
      Vector divergence    = Vector::Zero(myCalculus->nbVertices());
      auto cpt=0;
      
      auto surfmesh = myCalculus->getSurfaceMeshAlias();
      
      // Heat, normalization and divergence per face
      for(auto f=0; f< myCalculus->nbFaces(); ++f)
      {
        Vector faceHeat( myCalculus->degree(f));
        cpt=0;
        auto vertices = surfmesh->incidentVertices(f);
        for(auto v: vertices)
        {
          faceHeat(cpt) = heatDiffusion( v );
          ++cpt;
        }
        // ∇ heat / ∣∣∇ heat∣∣
        Vector grad = myCalculus->gradient(f) * faceHeat;
        grad.normalize();
      
        // div
        DenseMatrix oneForm = myCalculus->V(f)*grad;
        Vector divergenceFace = myCalculus->D(f).transpose()*myCalculus->M(f)*oneForm;
        cpt=0;
        for(auto v: vertices)
        {
          divergence(v) += divergenceFace(cpt);
          ++cpt;
        }
      }
      
      // Last Poisson solve
      Vector distVec = Vector::Ones(myCalculus->nbVertices()) + myPoissonSolver.solve(divergence);
      return distVec;
    }
    
    
    /// @return true if the calculus is valid.
    bool isValid() const
    {
      return myIsInit && myCalculus->isValid();
    }
    
    // ----------------------- Private --------------------------------------

  private:
    
    const PolygonalCalculus *myCalculus;

    Solver myPoissonSolver;
    Solver myHeatSolver;
    Vector mySource;
    
    bool myIsInit;
    
  }; // end of class GeodesicInHeat
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GeodesicInHeat_h

#undef GeodesicInHeat_RECURSES
#endif // else defined(GeodesicInHeat_RECURSES)
