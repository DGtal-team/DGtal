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
 * @author Baptiste GENEST (\c baptistegenest@gmail.com )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/05/20
 *
 * Header file for module VectorsInHeat.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VectorsInHeat_RECURSES)
#error Recursive header files inclusion detected in VectorsInHeat.h
#else // defined(VectorsInHeat_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VectorsInHeat_RECURSES

#if !defined VectorsInHeat_h
/** Prevents repeated inclusion of headers. */
#define VectorsInHeat_h

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
// template class VectorInHeat
/**
 * Description of template class 'VectorsInHeat' <p>
 * \brief This class implements @cite Sharp:2019:VHM on polygonal surfaces  (using @ref modulePolygonalCalculus).
 *
 * see @ref moduleVectorsInHeat for details and examples.
 *
 * @tparam a model of PolygonalCalculus.
 */
template <typename TPolygonalCalculus>
class VectorsInHeat
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
    VectorsInHeat() = delete;

    /// Constructor from an existing polygonal calculus. T
    /// @param calculus a instance of PolygonalCalculus
    VectorsInHeat(ConstAlias<PolygonalCalculus> calculus): myCalculus(&calculus)
    {
        myIsInit=false;
    }

    /**
     * Destructor.
     */
    ~VectorsInHeat() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    VectorsInHeat ( const VectorsInHeat & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    VectorsInHeat ( VectorsInHeat && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    VectorsInHeat & operator= ( const VectorsInHeat & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    VectorsInHeat & operator= ( VectorsInHeat && other ) = delete;



    // ----------------------- Interface --------------------------------------

    /// Initialize the solvers with @a dt as timestep for the
    /// heat diffusion and @a lambda parameter for the polygonal calculus,
    /// which guarantee definiteness for positive @a lambda.
    /// @param dt timestep
    /// @param lambda
    ///
    /// @param boundary_with_mixed_solution when 'true' and when the
    /// surface has boundaries, mix two solutions of the heat
    /// diffusion operation (Neumann and Dirichlet null conditions on
    /// boundary).
    void init( double dt, double lambda = 1.0,
               bool boundary_with_mixed_solution = false )
    {
        myIsInit=true;

        SparseMatrix laplacian = myCalculus->globalLaplaceBeltrami( lambda );

        SparseMatrix connectionLaplacian = myCalculus->globalConnectionLaplace( lambda );

        SparseMatrix mass = myCalculus->globalLumpedMassMatrix();
        SparseMatrix mass2= myCalculus->doubledGlobalLumpedMassMatrix();
        myScalarHeatOpe   =  mass - dt*laplacian;
        myVectorHeatOpe   =  mass2 - dt*connectionLaplacian;

        //Prefactorizing
        myScalarHeatSolver.compute(myScalarHeatOpe);
        myVectorHeatSolver.compute(myVectorHeatOpe);

        //empty sources
        myVectorSource     	= Vector::Zero(2*myCalculus->nbVertices());
        myScalarSource		= Vector::Zero(myCalculus->nbVertices());
        myDiracSource		= Vector::Zero(myCalculus->nbVertices());

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
        SparseMatrix heatOpe_d = Conditions::dirichletOperator( myScalarHeatOpe, myBoundary );
        // Prefactoring
        myHeatDirichletSolver.compute( heatOpe_d );
    }

    /** Adds a source vector (3D extrinsic) at a vertex @e aV
     *  the vector gets projected to the tangent space
    * @param aV the Vertex
    * @param ev the extrinsic R3 vector to add (only keeps the tangential part)
     **/
    void addSource(const Vertex aV,const Vector& ev)
    {
        ASSERT_MSG(aV < myCalculus->nbVertices(), "Vertex is not in the surface mesh vertex range");
        Vector v = myCalculus->Tv(aV).transpose()*ev;
        v = v.normalized()*ev.norm();
        myVectorSource( 2*aV ) = v(0);
        myVectorSource( 2*aV+1 ) = v(1);
        myScalarSource( aV ) = v.norm();
        myDiracSource( aV ) = 1;
    }

    /** Clears all sources
     **/
    void clearSource()
    {
      myVectorSource     	= Vector::Zero(2*myCalculus->nbVertices());
      myScalarSource		= Vector::Zero(myCalculus->nbVertices());
      myDiracSource		= Vector::Zero(myCalculus->nbVertices());
    }

    /**
     * @returns the source points vector.
     **/
    Vector vectorSource() const
    {
        FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
        return myVectorSource;
    }

    ///
    /// \brief extrinsicVectorSourceAtVertex get extrinsic source at vertex
    /// \param aV the vertex
    /// \return 3D source vector
    ///
    Vector extrinsicVectorSourceAtVertex(const Vertex aV){
        FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
        return myCalculus->toExtrinsicVector(aV,intrinsicVectorSourceAtVertex(aV));
    }

    ///
    /// \brief intrinsicVectorSourceAtVertex get intrinsic source at vertex
    /// \param aV the vertex
    /// \return 2D vector expressed in aV tangent frame
    ///
    Vector intrinsicVectorSourceAtVertex(const Vertex aV){
        FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
        Vector s(2);
        s(0) = myVectorSource(2*aV);
        s(1) = myVectorSource(2*aV+1);
        return s;
    }


    /// Main computation of the Vectors In Heat
    /// @returns the estimated heat diffused vectors from the sources expressed
    std::vector<Vector> compute() const
    {
        FATAL_ERROR_MSG(myIsInit, "init() method must be called first");
        //Heat diffusion
        Vector vectorHeatDiffusion = myVectorHeatSolver.solve(myVectorSource);
        Vector scalarHeatDiffusion = myScalarHeatSolver.solve(myScalarSource);
        Vector diracHeatDiffusion = myScalarHeatSolver.solve(myDiracSource);
        auto surfmesh = myCalculus->getSurfaceMeshPtr();


        // Take care of boundaries
        if ( myManageBoundary )
        {
          Vector bValues  = Vector::Zero( myCalculus->nbVertices() );
          Vector bNormSources = Conditions::dirichletVector( myScalarHeatOpe, myScalarSource,
                                                         myBoundary, bValues );
          Vector bSol     = myHeatDirichletSolver.solve( bNormSources );
          Vector heatDiffusionDirichlet
                          = Conditions::dirichletSolution( bSol, myBoundary, bValues );
          scalarHeatDiffusion = 0.5 * ( scalarHeatDiffusion + heatDiffusionDirichlet );
        }

        std::vector<Vector> result(surfmesh->nbVertices());

        for (typename PolygonalCalculus::MySurfaceMesh::Index v = 0;v<surfmesh->nbVertices();v++){
            Vector Y(2);
            Y(0) = vectorHeatDiffusion(2*v);
            Y(1) = vectorHeatDiffusion(2*v+1);
            Y = Y.normalized()*(scalarHeatDiffusion(v)/diracHeatDiffusion(v));
            result[v] = myCalculus->toExtrinsicVector(v,Y);
        }

        return result;
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

    ///The operators for heat diffusion
    SparseMatrix myScalarHeatOpe;
    SparseMatrix myVectorHeatOpe;

    ///Heat solvers
    Solver myScalarHeatSolver;
    Solver myVectorHeatSolver;

    ///Source vectors
    Vector myScalarSource;
    Vector myDiracSource;
    Vector myVectorSource;

    /// When 'true', manage boundaries with a mixed solution of
    /// Neumann and Dirichlet conditions.
    bool myManageBoundary;

    /// The boundary characteristic vector
    IntegerVector myBoundary;

    ///Validitate flag
    bool myIsInit;

    ///Heat solver with Dirichlet boundary conditions.
    Solver myHeatDirichletSolver;

}; // end of class VectorsInHeat
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VectorsInHeat_h

#undef VectorsInHeat_RECURSES
#endif // else defined(VectorsInHeat_RECURSES)
