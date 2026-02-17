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
 * @date 2019/06/06
 *
 * Header file for module DECHelpers.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DECHelpers_RECURSES)
#error Recursive header files inclusion detected in DECHelpers.h
#else // defined(DECHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DECHelpers_RECURSES

#if !defined DECHelpers_h
/** Prevents repeated inclusion of headers. */
#define DECHelpers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/dec/Duality.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /// Namespace for functions useful to Discrete Exterior Calculus package.
  namespace dec_helper
  {

    /// Builds a linear diagonal operator from a k-form. Corresponds to Diag(v) if v is a vector.
    /// @param kform any k-dimensional form.
    /// @return the corresponding diagonal linear operator.
    template <typename Calculus, DGtal::Dimension dim, DGtal::Duality duality>
    DGtal::LinearOperator<Calculus, dim, duality, dim, duality>
    diagonal(const DGtal::KForm<Calculus, dim, duality>& kform)
    {
      typedef DGtal::LinearOperator<Calculus,dim, duality, dim, duality> Operator;
      typedef typename Calculus::LinearAlgebraBackend::Triplet Triplet;
      typedef typename Calculus::Index Index;
      typedef std::vector<Triplet> Triplets;

      Triplets triplets;
      for (Index index=0; index<kform.length(); index++)
        triplets.emplace_back( index, index, kform.myContainer(index) );
      Operator ope(kform.myCalculus);
      ope.myContainer.setFromTriplets(triplets.begin(), triplets.end());

      return ope;
    }

    /// Builds the linear operator that brings a primal 0-form to a
    /// primal 1-form by averaging (vertex to edge operator)
    ///
    /// @param calculus any discrete calculus
    template <typename Calculus>
    DGtal::LinearOperator<Calculus, 0, DGtal::PRIMAL, 1, DGtal::PRIMAL>
    averageOperator01(const Calculus& calculus)
    {
      auto M01        = calculus.template derivative<0,DGtal::PRIMAL>();
      M01.myContainer = 0.5 * M01.myContainer.cwiseAbs();
      return M01;
    }

    /// Builds the linear operator that brings a primal 1-form to a
    /// primal 2-form by averaging (edge to face operator)
    ///
    /// @param calculus any discrete calculus
    template <typename Calculus>
    DGtal::LinearOperator<Calculus, 1, DGtal::PRIMAL, 2, DGtal::PRIMAL>
    averageOperator01(const Calculus& calculus)
    {
      auto M12        = calculus.template derivative<1,DGtal::PRIMAL>();
      M12.myContainer = 0.25 * M12.myContainer.cwiseAbs();
      return M12;
    }

    /// Builds the linear operator that brings a primal 2-form to a
    /// primal 0-form by averaging (face to vertex operator)
    ///
    /// @param calculus a discrete calculus over a 2D digital surface in 3D.
    template <typename Calculus>
    DGtal::LinearOperator<Calculus, 2, DGtal::PRIMAL, 0, DGtal::PRIMAL>
    averageOperator20(const Calculus& calculus)
    {
      BOOST_STATIC_ASSERT( Calculus::dimensionEmbedded == 2 );
      BOOST_STATIC_ASSERT( Calculus::dimensionAmbient == 3 );
      using DGtal::PRIMAL;
      using DGtal::DUAL;

      typedef typename Calculus::LinearAlgebraBackend::SparseMatrix SparseMatrix;
      typedef typename Calculus::LinearAlgebraBackend::Triplet Triplet;
      typedef typename Calculus::KSpace KSpace;
      typedef typename Calculus::Index Index;
      typedef typename Calculus::Cell Cell;
      typedef typename Calculus::Scalar Scalar;
      typedef typename Calculus::Point Point;
      typedef DGtal::LinearOperator<Calculus, 2, DGtal::PRIMAL, 0, DGtal::PRIMAL>
	Operator;

      const KSpace& kspace = calculus.myKSpace;

      const std::vector<Point> deltas = {
        Point(0,1,1), Point(0,-1,1), Point(0,-1,-1), Point(0,1,-1),
        Point(1,0,1), Point(-1,0,1), Point(-1,0,-1), Point(1,0,-1),
        Point(1,1,0), Point(-1,1,0), Point(-1,-1,0), Point(1,-1,0)
      };

      std::vector<Triplet> triplets;
      for (Index index_point=0; index_point<calculus.kFormLength(0,PRIMAL); index_point++)
        {
          const Cell point = kspace.unsigns(calculus.getSCell(0, PRIMAL, index_point));
          ASSERT( kspace.uDim(point) == 0 );

          std::vector<Index> indexes_surfel;
          for (const Point delta : deltas)
            {
              const Cell surfel = kspace.uCell((kspace.uKCoords(point)+delta));
              ASSERT( kspace.uDim(surfel) == 2 );
              if (calculus.containsCell(surfel))
                indexes_surfel.push_back(calculus.getCellIndex(surfel));
            }
          ASSERT( indexes_surfel.size() > 2 );

          const double weight = 1/static_cast<Scalar>(indexes_surfel.size());
          for (const Index index_surfel : indexes_surfel)
            triplets.emplace_back( index_point, index_surfel, weight );
        }

      SparseMatrix matrix( calculus.kFormLength(0, DGtal::PRIMAL),
			   calculus.kFormLength(2, DGtal::PRIMAL) );
      matrix.setFromTriplets(triplets.begin(), triplets.end());

      return Operator(calculus, matrix);
    }

  } // namespace dec_helper

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DECHelpers_h

#undef DECHelpers_RECURSES
#endif // else defined(DECHelpers_RECURSES)
