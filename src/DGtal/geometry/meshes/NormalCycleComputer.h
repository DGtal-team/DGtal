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
 * @file NormalCycleComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/18
 *
 * Header file for module NormalCycleComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NormalCycleComputer_RECURSES)
#error Recursive header files inclusion detected in NormalCycleComputer.h
#else // defined(NormalCycleComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NormalCycleComputer_RECURSES

#if !defined NormalCycleComputer_h
/** Prevents repeated inclusion of headers. */
#define NormalCycleComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
// always include EigenSupport.h before any other Eigen headers
// #include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "SimplifiedMesh.h"
#include "NormalCycleFormula.h"


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class NormalCycleComputer
  /**
     Description of template class 'NormalCycleComputer'
     <p> \brief Aim: Utility class to compute the normal cycle
     over a simplified mesh.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct NormalCycleComputer
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef NormalCycleComputer< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );

    typedef DGtal::SimplifiedMesh< RealPoint, RealVector > SimplifiedMesh;
    typedef typename RealVector::Component Scalar;
    typedef std::vector< Scalar >          Scalars;
    typedef std::vector< RealPoint >       RealPoints;
    typedef std::vector< RealVector >      RealVectors;
    typedef NormalCycleFormula< RealPoint, RealVector > Formula;
    typedef typename Formula::RealTensor   RealTensor;
    typedef std::vector< RealTensor >      RealTensors;
    
    /// The type for counting elements.
    typedef std::size_t                             Size;
    /// The type used for numbering vertices, edges, faces
    typedef std::size_t                             Index;
    typedef Index                                   Edge;
    typedef Index                                   Face;
    typedef Index                                   Vertex;
    typedef std::pair< Edge, Scalar >               WeightedEdge;
    typedef std::pair< Face, Scalar >               WeightedFace;
    /// The type that defines a range of vertices
    typedef std::vector< Vertex >                   Vertices;
    /// The type that defines a range of faces
    typedef std::vector< Edge >                     Edges;
    typedef std::vector< Face >                     Faces;
    typedef std::vector< WeightedEdge >             WeightedEdges;
    typedef std::vector< WeightedEdge >             WeightedFaces;

    enum class Measure { MU0, MU1, MU2, MUXY, MUXY1, MUXY2, ALL_MU };

    /// Constructor from mesh.
    /// @param aMesh any simplified mesh that is referenced in this object.
    NormalCycleComputer( ConstAlias< SimplifiedMesh > aMesh );

    bool computeMeasures( Measure mu );
    bool computeMu0     ();
    bool computeMu1     ();
    bool computeMu2     ();
    bool computeMuXY1   ();
    bool computeMuXY2   ();

    /// Given weighted faces, returns its  mu0 measure.
    Scalar     mu0  ( const WeightedFaces& wfaces ) const;
    /// Given weighted faces, returns its  mu1 measure.
    Scalar     mu1  ( const WeightedEdges& wedges ) const;
    /// Given weighted faces, returns its  mu2 measure.
    Scalar     mu2  ( const Vertices&      vertices ) const;
    /// Given weighted faces, returns its  muXY1 measure.
    RealTensor muXY1( const WeightedEdges& wedges ) const;
    /// Given weighted faces, returns its  muXY2 measure.
    RealTensor muXY2( const WeightedEdges& wedges ) const;

    // ------------------------- Public Datas ------------------------------
  public:
    /// Per-face mu0 measure (area).
    Scalars     localMu0;
    /// Per-edge mu1 measure (mean curvature).
    Scalars     localMu1;
    /// Per-vertex mu2 measure (Gaussian curvature).
    Scalars     localMu2;
    /// Per-edge muXY1 measure (2nd fundamental form).
    RealTensors localMuXY1;
    /// Per-edge muXY2 measure (2nd fundamental form).
    RealTensors localMuXY2;
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// A reference to the mesh over which computations are done.
    const SimplifiedMesh& myMesh;
    
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:
    
    
  }; // end of class NormalCycleComputer
    
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "NormalCycleComputer.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NormalCycleComputer_h

#undef NormalCycleComputer_RECURSES
#endif // else defined(NormalCycleComputer_RECURSES)

