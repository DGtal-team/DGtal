#if !defined(__DEC_CALCULUS_H__)
#define __DEC_CALCULUS_H__

#include "AllSCellMap.h"
#include "Duality.h"
#include "KForm.h"
#include "LinearOperator.h"
#include "VectorField.h"

#include "SuiteSparseLinearAlgebraBackend.h"

#include <boost/array.hpp>
#include <DGtal/kernel/domains/CDomain.h>
#include <DGtal/topology/KhalimskySpaceND.h>

template <typename D>
struct DiscreteExteriorCalculus
{
    // template typedef
    typedef D Domain;
    BOOST_CONCEPT_ASSERT(( DGtal::CDomain<Domain> ));
    typedef typename Domain::Space::Dimension Dimension;
    static const Dimension dimension = Domain::Space::dimension;

    // general typedef
    typedef typename DGtal::KhalimskySpaceND<dimension, typename Domain::Space::Integer> KSpace;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::Point Point;

    typedef SuiteSparseLinearAlgebraBackend LinearAlgebraBackend;
    typedef LinearAlgebraBackend::Index Index;
    BOOST_STATIC_ASSERT(( boost::is_same<Dimension, Order>::value ));

    // general members
    const Domain domain;
    const KSpace kspace;

    // common vector fields typedef
    typedef VectorField<DiscreteExteriorCalculus, PRIMAL> PrimalVectorField;
    typedef VectorField<DiscreteExteriorCalculus, DUAL> DualVectorField;

    // common k-forms typedef
    typedef KForm<DiscreteExteriorCalculus, 0, PRIMAL> PrimalForm0;
    typedef KForm<DiscreteExteriorCalculus, 1, PRIMAL> PrimalForm1;
    typedef KForm<DiscreteExteriorCalculus, 2, PRIMAL> PrimalForm2;
    typedef KForm<DiscreteExteriorCalculus, 3, PRIMAL> PrimalForm3;
    typedef KForm<DiscreteExteriorCalculus, 0, DUAL> DualForm0;
    typedef KForm<DiscreteExteriorCalculus, 1, DUAL> DualForm1;
    typedef KForm<DiscreteExteriorCalculus, 2, DUAL> DualForm2;
    typedef KForm<DiscreteExteriorCalculus, 3, DUAL> DualForm3;

    // linear exterior derivative operator typedef
    typedef LinearOperator<DiscreteExteriorCalculus, 0, PRIMAL, 1, PRIMAL> PrimalDerivative0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, PRIMAL, 2, PRIMAL> PrimalDerivative1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, PRIMAL, 3, PRIMAL> PrimalDerivative2;
    typedef LinearOperator<DiscreteExteriorCalculus, 0, DUAL, 1, DUAL> DualDerivative0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, DUAL, 2, DUAL> DualDerivative1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, DUAL, 3, DUAL> DualDerivative2;

    // linear hodge operator typedef
    typedef LinearOperator<DiscreteExteriorCalculus, 0, PRIMAL, dimension-0, DUAL> PrimalHodge0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, PRIMAL, dimension-1, DUAL> PrimalHodge1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, PRIMAL, dimension-2, DUAL> PrimalHodge2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, PRIMAL, dimension-3, DUAL> PrimalHodge3;
    typedef LinearOperator<DiscreteExteriorCalculus, 0, DUAL, dimension-0, PRIMAL> DualHodge0;
    typedef LinearOperator<DiscreteExteriorCalculus, 1, DUAL, dimension-1, PRIMAL> DualHodge1;
    typedef LinearOperator<DiscreteExteriorCalculus, 2, DUAL, dimension-2, PRIMAL> DualHodge2;
    typedef LinearOperator<DiscreteExteriorCalculus, 3, DUAL, dimension-3, PRIMAL> DualHodge3;

    // cells size ratio (from primal to dual)
    typedef AllSCellMap<DiscreteExteriorCalculus, double> AllSCellSizeRatio;
    AllSCellSizeRatio cell_size_ratio;

    // cells indexes to map operator index to geometrical cell
    typedef AllSCellMap<DiscreteExteriorCalculus, Index> AllSCellIndexes;
    AllSCellIndexes cell_indexes;

    // cells from indexes by cell dimension
    typedef std::vector<SCell> SCells;
    typedef boost::array<SCells, dimension+1> IndexSCells;
    IndexSCells index_cells;

    // Constructor
    // digital set point get attached to primal n-cell <-> dual 0-cell
    template <typename DigitalSet>
    DiscreteExteriorCalculus(const DigitalSet& _set);

    // return exterior derivative operator from order-forms to (order+1)-forms
    template <Order order, Duality duality>
    LinearOperator<DiscreteExteriorCalculus, order, duality, order+1, duality>
    derivative() const;

    // return primal hodge duality operator from primal order-forms to dual (dim-order)-forms
    template <Order order>
    LinearOperator<DiscreteExteriorCalculus, order, PRIMAL, dimension-order, DUAL>
    primalHodge() const;

    // return dual hodge duality operator from primal order-forms to primal (dim-order)-forms
    template <Order order>
    LinearOperator<DiscreteExteriorCalculus, order, DUAL, dimension-order, PRIMAL>
    dualHodge() const;

    // return 1-form from vector field
    template <Duality duality>
    KForm<DiscreteExteriorCalculus, 1, duality>
    flat(const VectorField<DiscreteExteriorCalculus, duality>& vector_field) const;

    // return vector field from 1-form
    template <Duality duality>
    VectorField<DiscreteExteriorCalculus, duality>
    sharp(const KForm<DiscreteExteriorCalculus, 1, duality>& one_form) const;

    // return discrete kforms size
    Index
    kFormLength(const Order& order, const Duality& duality) const;

    // return actual order of kform : return order if primal, dimension-order if dual
    Order
    actualOrder(const Order& order, const Duality& duality) const;

    // return sign of hodge operator
    double
    hodgeSign(const SCell& cell, const Duality& duality) const;

    // return sign of derivative operator
    double
    derivativeSign(const SCell& cell, const Duality& duality) const;

    // return positive cell. usefull for looking cells up since all stored cells are positive
    SCell
    absoluteCell(const SCell& cell) const;

    // return edge direction relative to primal
    Dimension
    edgeDirection(const SCell& cell, const Duality& duality) const;

    protected:

    typedef std::list<Point> Points;

    static
    void
    neighborhoodRecurseSizeMask(const Dimension& dimension, const Point& current_point, Points& neighborhood);
};

// equality operator

// print discrete exterior calculus to ouput stream
template <typename D>
std::ostream&
operator<<(std::ostream& os, const DiscreteExteriorCalculus<D>& calculus);

#include "Calculus.ih"

#endif
