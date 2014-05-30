#if !defined(__DEC_TESTS_COMMON_H__)
#define __DEC_TESTS_COMMON_H__

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/CDiscreteExteriorCalculusVectorSpace.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;

using std::endl;

template <typename Container>
bool
is_all_zero(const Container& container)
{
    for (typename Container::Index ii=0; ii<container.rows(); ii++)
        for (typename Container::Index jj=0; jj<container.cols(); jj++)
            if (container(ii,jj) != 0)
                return false;
    return true;
}

template <typename Container, typename Value>
bool
is_identity(const Container& container, const Value& value)
{
    for (typename Container::Index ii=0; ii<container.rows(); ii++)
        for (typename Container::Index jj=0; jj<container.cols(); jj++)
            if ((ii != jj && container(ii,jj) != 0) || (ii == jj && container(ii,jj) != value))
                return false;
    return true;
}

//DC: changing order to int (to match with -1 specialization)
template <typename Calculus, int order>
struct HodgeTester
{
    BOOST_STATIC_ASSERT(( order <= Calculus::dimension ));

    static bool test(const Calculus& calculus)
    {
        trace.info() << "testing identity operators " << order << endl;

        { // test identity operator
            typedef LinearOperator<Calculus, order, PRIMAL, order, PRIMAL> PrimalIdentity;
            PrimalIdentity primal_identity = calculus.template identity<order, PRIMAL>();
            if (!is_identity(primal_identity.myContainer, 1)) return false;

            typedef KForm<Calculus, order, PRIMAL> SolveForm;
            SolveForm input(calculus);
            SolveForm output = primal_identity * input;
            typedef typename Calculus::LinearAlgebraBackend LinearAlgebraBackend;
            typedef typename LinearAlgebraBackend::SolverConjugateGradient LinearSolver;
            typedef DiscreteExteriorCalculusSolver<Calculus, LinearSolver, order, PRIMAL, order, PRIMAL> Solver;
            Solver solver;
            SolveForm input_solved = solver.compute(primal_identity).solve(output);
            //if (input_solved != input) return false;

            typedef LinearOperator<Calculus, order, DUAL, order, DUAL> DualIdentity;
            DualIdentity dual_identity = calculus.template identity<order, DUAL>();
            if (!is_identity(dual_identity.myContainer, 1)) return false;
        }

        typedef LinearOperator<Calculus, order, PRIMAL, Calculus::dimension-order, DUAL> PrimalHodge;
        typedef LinearOperator<Calculus, Calculus::dimension-order, DUAL, order, PRIMAL> DualHodge;
        const PrimalHodge primal_hodge = calculus.template primalHodge<order>();
        const DualHodge dual_hodge = calculus.template dualHodge<Calculus::dimension-order>();

        trace.info() << "testing primal to primal hodge composition order " << order << endl;

        { // test primal to primal composition
            typedef LinearOperator<Calculus, order, PRIMAL, order, PRIMAL> PrimalPrimal;
            PrimalPrimal primal_primal = dual_hodge * primal_hodge;
            if (!is_identity(primal_primal.myContainer, pow(-1, order*(Calculus::dimension-order)))) return false;
        }

        trace.info() << "testing dual to dual hodge composition order " << order << endl;

        { // test dual to dual composition
            typedef LinearOperator<Calculus, Calculus::dimension-order, DUAL, Calculus::dimension-order, DUAL> DualDual;
            DualDual dual_dual = primal_hodge * dual_hodge;
            if (!is_identity(dual_dual.myContainer, pow(-1, order*(Calculus::dimension-order)))) return false;
        }

        return HodgeTester<Calculus, order-1>::test(calculus);
    }
};

template <typename Calculus>
struct HodgeTester<Calculus, -1>
{
    static bool test(const Calculus& calculus)
    {
        return true;
    }
};

template <typename DigitalSet, typename LinearAlgebraBackend>
void
test_hodge(int domain_size=5)
{
    BOOST_CONCEPT_ASSERT(( CDigitalSet<DigitalSet> ));

    typedef typename DigitalSet::Domain Domain;
    typedef typename DigitalSet::Point Point;
    trace.info() << "dimension=" << Point::dimension << endl;
    Domain domain(Point(), Point::diagonal(domain_size-1));
    trace.info() << "domain=" << domain << endl;

    DigitalSet set(domain);
    for (typename Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        if (random()%4!=0) continue;
        const typename Domain::Point& point = *di;
        set.insertNew(point);
    }
    trace.info() << "domain.size()=" << domain.size() << endl;
    trace.info() << "set.size()=" << set.size() << endl;

    typedef DiscreteExteriorCalculus<Domain::Space::dimension, LinearAlgebraBackend> Calculus;
    Calculus calculus(set);
    {
        trace.beginBlock("testing indexes");

        {
            typename Calculus::Properties properties = calculus.getProperties();
            trace.info() << "properties.size()=" << properties.size() << endl;
        }

        typedef typename Calculus::ConstIterator ConstIterator;
        typedef typename Calculus::SCell SCell;
        typedef typename Calculus::Index Index;
        bool test_result = true;
        for (ConstIterator iter = calculus.begin(), iter_end = calculus.end(); test_result && iter!=iter_end; iter++)
        {
            const SCell& cell = iter->first;
            const Index& index = calculus.getSCellIndex(cell);
            test_result &= (iter->second.index == index);
            const SCell& primal_cell = calculus.getSCell(calculus.myKSpace.sDim(cell), PRIMAL, index);
            test_result &= (cell == primal_cell);
            const SCell& dual_cell = calculus.getSCell(calculus.dimension-calculus.myKSpace.sDim(cell), DUAL, index);
            test_result &= (cell == dual_cell);
        }
        trace.endBlock();

        FATAL_ERROR(test_result);
    }

    trace.beginBlock("testing hodge");
    bool test_result = HodgeTester<Calculus, Calculus::dimension>::test(calculus);
    trace.endBlock();

    FATAL_ERROR(test_result);
}

//DC Order->int (see above)
template <typename Calculus, int order>
struct DerivativeTester
{
    BOOST_STATIC_ASSERT(( order < (int)Calculus::dimension - 1 ));

    static bool test(const Calculus& calculus)
    {
        trace.info() << "testing primal derivative composition order " << order << endl;

        { // test primal composition
            typedef LinearOperator<Calculus, order, PRIMAL, order+1, PRIMAL> FirstDerivative;
            FirstDerivative first_derivative = calculus.template derivative<order, PRIMAL>();
            typedef LinearOperator<Calculus, order+1, PRIMAL, order+2, PRIMAL> SecondDerivative;
            SecondDerivative second_derivative = calculus.template derivative<order+1, PRIMAL>();
            typedef LinearOperator<Calculus, order, PRIMAL, order+2, PRIMAL> DoubleDerivative;
            DoubleDerivative double_derivative = second_derivative * first_derivative;
            if (!is_all_zero(double_derivative.myContainer)) return false;
        }

        trace.info() << "testing dual derivative composition order " << order << endl;

        { // test dual composition
            typedef LinearOperator<Calculus, order, DUAL, order+1, DUAL> FirstDerivative;
            FirstDerivative first_derivative = calculus.template derivative<order, DUAL>();
            typedef LinearOperator<Calculus, order+1, DUAL, order+2, DUAL> SecondDerivative;
            SecondDerivative second_derivative = calculus.template derivative<order+1, DUAL>();
            typedef LinearOperator<Calculus, order, DUAL, order+2, DUAL> DoubleDerivative;
            DoubleDerivative double_derivative = second_derivative * first_derivative;
            if (!is_all_zero(double_derivative.myContainer)) return false;
        }

        /*
        trace.info() << "testing liebnitz rule order " << order << endl;

        {
            typedef LinearOperator<Calculus, order, PRIMAL, order+1, PRIMAL> Derivative;
            Derivative derivative = calculus.template derivative<order, PRIMAL>();

            typedef KForm<Calculus, order, PRIMAL> InputForm;
            typedef KForm<Calculus, order+1, PRIMAL> OutputForm;
            InputForm alpha(calculus), beta(calculus), gamma(calculus);

            for (int kk=0; kk<calculus.kFormLength(order, PRIMAL); kk++)
            {
                const double ak = static_cast<double>(random())/RAND_MAX;
                const double bk = static_cast<double>(random())/RAND_MAX;
                alpha.myContainer(kk) = ak;
                beta.myContainer(kk) = bk;
                gamma.myContainer(kk) = ak*bk;
            }

        }
        */

        return DerivativeTester<Calculus, order-1>::test(calculus);
    }
};

template <typename Calculus>
struct DerivativeTester<Calculus, -1>
{
    static bool test(const Calculus& calculus)
    {
        return true;
    }
};

template <typename DigitalSet, typename LinearAlgebraBackend>
void
test_derivative(int domain_size=10)
{
    BOOST_CONCEPT_ASSERT(( CDigitalSet<DigitalSet> ));

    typedef typename DigitalSet::Domain Domain;
    typedef typename DigitalSet::Point Point;
    trace.info() << "dimension=" << Point::dimension << endl;
    Domain domain(Point(), Point::diagonal(domain_size-1));
    trace.info() << "domain=" << domain << endl;

    DigitalSet set(domain);
    for (typename Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        if (random()%4!=0) continue;
        const typename Domain::Point& point = *di;
        set.insertNew(point);
    }
    trace.info() << "domain.size()=" << domain.size() << endl;
    trace.info() << "set.size()=" << set.size() << endl;

    typedef DiscreteExteriorCalculus<Domain::Space::dimension, LinearAlgebraBackend> Calculus;
    Calculus calculus(set);
    {
        typename Calculus::Properties properties = calculus.getProperties();
        trace.info() << "properties.size()=" << properties.size() << endl;
    }

    trace.beginBlock("testing derivative");
    bool test_result = DerivativeTester<Calculus, Calculus::dimension-2>::test(calculus);
    trace.endBlock();

    FATAL_ERROR(test_result);
}

template <typename LinearAlgebraBackend>
void
test_concepts()
{
    trace.beginBlock("concepts");

    { // 2d
        typedef DiscreteExteriorCalculus<2, LinearAlgebraBackend> Calculus;
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm2> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalVectorField> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualVectorField> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative1> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge2> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity2> ));
    }

    { // 3d
        typedef DiscreteExteriorCalculus<3, LinearAlgebraBackend> Calculus;
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm3> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm3> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalVectorField> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualVectorField> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative2> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge3> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge3> ));

        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity3> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity0> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity1> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity2> ));
        BOOST_CONCEPT_ASSERT(( CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity3> ));
    }

    trace.endBlock();
}

template <typename LinearAlgebraBackend>
void
test_hodge_sign()
{
    trace.beginBlock("testing hodge sign");

    {
        typedef DiscreteExteriorCalculus<2, LinearAlgebraBackend> Calculus;
        const Z2i::Domain domain;
        const Z2i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef Z2i::Point Point;
        typedef typename Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::NEG), DUAL ) == -1 );
        // primal horizontal edge, dual vertical edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::POS), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::NEG), DUAL ) == 1 );
        // primal vectical edge, dual horizontal edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::POS), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::NEG), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::NEG), DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::NEG), DUAL ) == -1 );
    }

    {
        typedef DiscreteExteriorCalculus<3, LinearAlgebraBackend> Calculus;
        const Z3i::Domain domain;
        const Z3i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef Z3i::Point Point;
        typedef typename Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::NEG), DUAL ) == -1 );
        // primal edge, dual surfel
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::NEG), DUAL ) == -1 );
        // primal surfel, dual edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::NEG), DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::NEG), DUAL ) == -1 );
    }

    trace.endBlock();
}

template <typename LinearAlgebraBackend>
void
test_backend(const int& ntime, const int& maxdim)
{
    srandom(0);

    test_hodge_sign<LinearAlgebraBackend>();

    for (int kk=0; kk<ntime; kk++)
    {
        typedef SpaceND<4, int> Space4;
        typedef HyperRectDomain<Space4> Domain4;
        typedef DigitalSetBySTLSet<Domain4> DigitalSet4;

        typedef SpaceND<5, int> Space5;
        typedef HyperRectDomain<Space5> Domain5;
        typedef DigitalSetBySTLSet<Domain5> DigitalSet5;

        if (maxdim>=2) test_hodge<Z2i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=3) test_hodge<Z3i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=4) test_hodge<DigitalSet4, LinearAlgebraBackend>(5);
        if (maxdim>=5) test_hodge<DigitalSet5, LinearAlgebraBackend>(3);

        if (maxdim>=2) test_derivative<Z2i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=3) test_derivative<Z3i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=4) test_derivative<DigitalSet4, LinearAlgebraBackend>(5);
        if (maxdim>=5) test_derivative<DigitalSet5, LinearAlgebraBackend>(3);
    }

    test_concepts<LinearAlgebraBackend>();
}

#endif

