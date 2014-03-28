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
 * @file DiscreteExteriorCalculus.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/27
 *
 * Header file for module DiscreteExteriorCalculus.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DiscreteExteriorCalculus_RECURSES)
#error Recursive header files inclusion detected in DiscreteExteriorCalculus.h
#else // defined(DiscreteExteriorCalculus_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DiscreteExteriorCalculus_RECURSES

#if !defined DiscreteExteriorCalculus_h
/** Prevents repeated inclusion of headers. */
#define DiscreteExteriorCalculus_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <boost/array.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/dec/AllSCellMap.h"
#include "DGtal/dec/Duality.h"
#include "DGtal/dec/KForm.h"
#include "DGtal/dec/LinearOperator.h"
#include "DGtal/dec/VectorField.h"

#include <DGtal/math/linalg/CDynamicMatrix.h>
#include <DGtal/math/linalg/CDynamicVector.h>
#include <DGtal/math/linalg/CLinearAlgebra.h>
#include <DGtal/kernel/domains/CDomain.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DiscreteExteriorCalculus
  /**
   * Description of template class 'DiscreteExteriorCalculus' <p>
   * \brief Aim:
   * DiscreteExteriorCalculus represents a calculus in the dec package.
   * This is the main structure in the dec package.
   * This is used to describe the space on wich the dec is build and to compute various operators.
   * Once operators or kforms are created, this structure should not be modified.
   *
   * @tparam Domain should be a model of CDomain.
   * @tparam LinearAlgebraBackend is linear algebra backend used (i.e. EigenSparseLinearAlgebraBackend).
   */
  template <typename D, typename LAB>
  class DiscreteExteriorCalculus
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef D Domain;

    BOOST_CONCEPT_ASSERT(( DGtal::CDomain<Domain> ));

    typedef LAB LineaAlgebraBackend;
    typedef typename LineaAlgebraBackend::Vector::Index Index;
    typedef typename LineaAlgebraBackend::Vector::Scalar Scalar;
    typedef typename LineaAlgebraBackend::Vector Vector;
    typedef typename LineaAlgebraBackend::Matrix Matrix;

    BOOST_CONCEPT_ASSERT(( DGtal::CDynamicVector<Vector> ));
    BOOST_CONCEPT_ASSERT(( DGtal::CDynamicMatrix<Matrix> ));
    BOOST_CONCEPT_ASSERT(( DGtal::CLinearAlgebra<Vector, Matrix> ));

    typedef typename Domain::Space::Dimension Dimension;

    BOOST_STATIC_ASSERT(( boost::is_same<Dimension, Order>::value ));

    /**
     * Static dimension.
     */
    static const Dimension dimension = Domain::Space::dimension;

    typedef typename DGtal::KhalimskySpaceND<dimension, typename Domain::Space::Integer> KSpace;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::Point Point;

    /**
     * Cells map typedefs.
     */
    typedef AllSCellMap<DiscreteExteriorCalculus, Scalar> Accum;
    typedef AllSCellMap<DiscreteExteriorCalculus, double> SizeRatio;
    typedef AllSCellMap<DiscreteExteriorCalculus, Index> Indexes;

    /**
     * Indexes to cells map typedefs.
     */
    typedef std::vector<SCell> SCells;
    typedef boost::array<SCells, dimension+1> IndexedSCells;

    /**
     * Vector field typedefs.
     */
    typedef VectorField<DiscreteExteriorCalculus, PRIMAL> PrimalVectorField;
    typedef VectorField<DiscreteExteriorCalculus, DUAL> DualVectorField;

    /**
     * KForms typedefs.
     */
    typedef KForm<DiscreteExteriorCalculus, 0, PRIMAL> PrimalForm0;
    typedef KForm<DiscreteExteriorCalculus, 1, PRIMAL> PrimalForm1;
    typedef KForm<DiscreteExteriorCalculus, 2, PRIMAL> PrimalForm2;
    typedef KForm<DiscreteExteriorCalculus, 3, PRIMAL> PrimalForm3;
    typedef KForm<DiscreteExteriorCalculus, 0, DUAL> DualForm0;
    typedef KForm<DiscreteExteriorCalculus, 1, DUAL> DualForm1;
    typedef KForm<DiscreteExteriorCalculus, 2, DUAL> DualForm2;
    typedef KForm<DiscreteExteriorCalculus, 3, DUAL> DualForm3;

    /**
     * Derivative linear operator typedefs.
     */
    typedef LinearOperator<DiscreteExteriorCalculus, 0, PRIMAL, 1, PRIMAL> PrimalDerivative0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, PRIMAL, 2, PRIMAL> PrimalDerivative1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, PRIMAL, 3, PRIMAL> PrimalDerivative2;
    typedef LinearOperator<DiscreteExteriorCalculus, 0, DUAL, 1, DUAL> DualDerivative0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, DUAL, 2, DUAL> DualDerivative1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, DUAL, 3, DUAL> DualDerivative2;

    /**
     * Hodge duality linear operator typedefs.
     */
    typedef LinearOperator<DiscreteExteriorCalculus, 0, PRIMAL, dimension-0, DUAL> PrimalHodge0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, PRIMAL, dimension-1, DUAL> PrimalHodge1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, PRIMAL, dimension-2, DUAL> PrimalHodge2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, PRIMAL, dimension-3, DUAL> PrimalHodge3;
    typedef LinearOperator<DiscreteExteriorCalculus, 0, DUAL, dimension-0, PRIMAL> DualHodge0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, DUAL, dimension-1, PRIMAL> DualHodge1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, DUAL, dimension-2, PRIMAL> DualHodge2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, DUAL, dimension-3, PRIMAL> DualHodge3;

    /**
     * Identity linear operator typedefs.
     */
    typedef LinearOperator<DiscreteExteriorCalculus, 0, PRIMAL, 0, PRIMAL> PrimalIdentity0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, PRIMAL, 1, PRIMAL> PrimalIdentity1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, PRIMAL, 2, PRIMAL> PrimalIdentity2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, PRIMAL, 3, PRIMAL> PrimalIdentity3;
    typedef LinearOperator<DiscreteExteriorCalculus, 0, DUAL, 0, DUAL> DualIdentity0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, DUAL, 1, DUAL> DualIdentity1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, DUAL, 2, DUAL> DualIdentity2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, DUAL, 3, DUAL> DualIdentity3;

    /**
     * Constructor.
     * @param set the initial set copied.
     * Set point get attached to primal n-cell <-> dual 0-cell.
     */
    template <typename DigitalSet>
    DiscreteExteriorCalculus(const DigitalSet& set);

    /**
     * Const iterator typedef.
     */
    typedef typename SizeRatio::ConstIterator ConstIterator;

    /**
     * Begin const iterator.
     */
    ConstIterator begin() const;

    /**
     * End const iterator.
     */
    ConstIterator end() const;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Definition domain.
     */
    Domain domain;

    /**
     * Associated Khalimski space.
     */
    KSpace kspace;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay(std::ostream& out) const;

    /**
     * Get cell size ratio
     */
    SizeRatio
    getSizeRatio() const;

    /**
     * Get cell size indexes
     */
    Indexes
    getIndexes() const;

    /**
     * Identity operator from order-forms to order-forms.
     * @tparam order
     * @tparam duality
     */
    template <Order order, Duality duality>
    LinearOperator<DiscreteExteriorCalculus, order, duality, order, duality>
    identity() const;

    /**
     * Exterior derivative operator from order-forms to (order+1)-forms
     * @tparam order
     * @tparam duality
     */
    template <Order order, Duality duality>
    LinearOperator<DiscreteExteriorCalculus, order, duality, order+1, duality>
    derivative() const;

    /**
     * Primal hodge duality operator from primal order-forms to dual (dim-order)-forms
     * @tparam order
     */
    template <Order order>
    LinearOperator<DiscreteExteriorCalculus, order, PRIMAL, dimension-order, DUAL>
    primalHodge() const;

    /**
     * Dual hodge duality operator from primal order-forms to dual (dim-order)-forms
     * @tparam order
     */
    template <Order order>
    LinearOperator<DiscreteExteriorCalculus, order, DUAL, dimension-order, PRIMAL>
    dualHodge() const;

    /**
     * Construct 1-form from vector field
     * @tparam duality
     * @param vector_field
     */
    template <Duality duality>
    KForm<DiscreteExteriorCalculus, 1, duality>
    flat(const VectorField<DiscreteExteriorCalculus, duality>& vector_field) const;

    /**
     * Construct vector field from 1-form
     * @tparam duality
     * @param one_form
     */
    template <Duality duality>
    VectorField<DiscreteExteriorCalculus, duality>
    sharp(const KForm<DiscreteExteriorCalculus, 1, duality>& one_form) const;

    /**
     * Get cell from index.
     * @param order
     * @param duality
     * @param index
     */
    SCell
    getSCell(const Order& order, const Duality& duality, const Index& index) const;

    /**
     * Return discrete kforms size.
     */
    Index
    kFormLength(const Order& order, const Duality& duality) const;

    /**
     * Return actual order of kform in the dec package representation.
     * @param order
     * @param duality
     * @return order if primal, dimension-order if dual
     */
    Order
    actualOrder(const Order& order, const Duality& duality) const;

    /**
     * Return sign of hodge operator.
     * @param cell
     * @param duality
     */
    Scalar
    hodgeSign(const SCell& cell, const Duality& duality) const;

    /**
     * Return sign of derivative operator.
     * @param cell
     * @param duality
     */
    Scalar
    derivativeSign(const SCell& cell, const Duality& duality) const;

    /**
     * Return positive cell.
     * Useful for looking cells up since all stored cells are positive.
     * @param cell
     */
    SCell
    absoluteCell(const SCell& cell) const;

    /**
     * Return edge direction relative to primal.
     * @param cell
     * @param duality
     */
    Dimension
    edgeDirection(const SCell& cell, const Duality& duality) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Datas --------------------------------
  private:

    /**
     * Cells size ratio (from primal to dual).
     */
    SizeRatio cell_size_ratio;

    /**
     * Cells indexes to map operator index to geometrical cell.
     */
    Indexes cell_indexes;

    /**
     * Cells indexes by they order.
     * Usefull for finding cell form index and order.
     */
    IndexedSCells index_cells;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DiscreteExteriorCalculus();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DiscreteExteriorCalculus(const DiscreteExteriorCalculus& other);

    ///**
    // * Assignment.
    // * @param other the object to copy.
    // * @return a reference on 'this'.
    // * Forbidden by default.
    // */
    //DiscreteExteriorCalculus& operator=(const DiscreteExteriorCalculus& other);

    // ------------------------- Internals ------------------------------------
  private:

    typedef std::list<Point> Points;

    static
    void
    neighborhoodRecurseSizeMask(const Dimension& dimension, const Point& current_point, Points& neighborhood);

  }; // end of class DiscreteExteriorCalculus


  /**
   * Overloads 'operator<<' for displaying objects of class 'DiscreteExteriorCalculus'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DiscreteExteriorCalculus' to write.
   * @return the output stream after the writing.
   */
  template <typename D, typename LAB>
  std::ostream&
  operator<<(std::ostream& out, const DiscreteExteriorCalculus<D, LAB>& object);

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/dec/DiscreteExteriorCalculus.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DiscreteExteriorCalculus_h

#undef DiscreteExteriorCalculus_RECURSES
#endif // else defined(DiscreteExteriorCalculus_RECURSES)
