#if !defined(__DEC_TESTS_COMMON_H__)
#define __DEC_TESTS_COMMON_H__

#include <list>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/CDiscreteExteriorCalculusVectorSpace.h"

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
        DGtal::trace.info() << "testing identity operators " << order << std::endl;

        { // test identity operator
            typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, order, DGtal::PRIMAL> PrimalIdentity;
						PrimalIdentity primal_identity = calculus.template identity<order, DGtal::PRIMAL>();
            if (!is_identity(primal_identity.myContainer, 1)) return false;

            typedef DGtal::KForm<Calculus, order, DGtal::PRIMAL> SolveForm;
            SolveForm input(calculus);
            SolveForm output = primal_identity * input;
            typedef typename Calculus::LinearAlgebraBackend LinearAlgebraBackend;
            typedef typename LinearAlgebraBackend::SolverConjugateGradient LinearSolver;
            typedef DGtal::DiscreteExteriorCalculusSolver<Calculus, LinearSolver, order, DGtal::PRIMAL, order, DGtal::PRIMAL> Solver;
            Solver solver;
            SolveForm input_solved = solver.compute(primal_identity).solve(output);
            //if (input_solved != input) return false;

            typedef DGtal::LinearOperator<Calculus, order, DGtal::DUAL, order, DGtal::DUAL> DualIdentity;
            DualIdentity dual_identity = calculus.template identity<order, DGtal::DUAL>();
            if (!is_identity(dual_identity.myContainer, 1)) return false;
        }

        typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, Calculus::dimension-order, DGtal::DUAL> PrimalHodge;
        typedef DGtal::LinearOperator<Calculus, Calculus::dimension-order, DGtal::DUAL, order, DGtal::PRIMAL> DualHodge;
        const PrimalHodge primal_hodge = calculus.template primalHodge<order>();
        const DualHodge dual_hodge = calculus.template dualHodge<Calculus::dimension-order>();

        DGtal::trace.info() << "testing primal to primal hodge composition order " << order << std::endl;

        { // test primal to primal composition
            typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, order, DGtal::PRIMAL> PrimalPrimal;
            PrimalPrimal primal_primal = dual_hodge * primal_hodge;
            if (!is_identity(primal_primal.myContainer, pow(-1, order*(Calculus::dimension-order)))) return false;
        }

        DGtal::trace.info() << "testing dual to dual hodge composition order " << order << std::endl;

        { // test dual to dual composition
            typedef DGtal::LinearOperator<Calculus, Calculus::dimension-order, DGtal::DUAL, Calculus::dimension-order, DGtal::DUAL> DualDual;
            DualDual dual_dual = primal_hodge * dual_hodge;
            if (!is_identity(dual_dual.myContainer, pow(-1, order*(Calculus::dimension-order)))) return false;
        }

        return HodgeTester<Calculus, order-1>::test(calculus);
    }
};

template <typename Calculus>
struct HodgeTester<Calculus, -1>
{
  static bool test(const Calculus& UNUSED(calculus))
  {
    return true;
  }
};

template <typename DigitalSet, typename LinearAlgebraBackend>
void
test_hodge(int domain_size=5)
{
    BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDigitalSet<DigitalSet> ));

    typedef typename DigitalSet::Domain Domain;
    typedef typename DigitalSet::Point Point;
    DGtal::trace.info() << "dimension=" << Point::dimension << std::endl;
    Domain domain(Point(), Point::diagonal(domain_size-1));
    DGtal::trace.info() << "domain=" << domain << std::endl;

    DigitalSet set(domain);
    for (typename Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        if (random()%4!=0) continue;
        const typename Domain::Point& point = *di;
        set.insertNew(point);
    }
    DGtal::trace.info() << "domain.size()=" << domain.size() << std::endl;
    DGtal::trace.info() << "set.size()=" << set.size() << std::endl;

    typedef DGtal::DiscreteExteriorCalculus<Domain::Space::dimension, LinearAlgebraBackend> Calculus;
    Calculus calculus(set);
    {
        DGtal::trace.beginBlock("testing indexes");

        {
            typename Calculus::Properties properties = calculus.getProperties();
            DGtal::trace.info() << "properties.size()=" << properties.size() << std::endl;
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
            const SCell& primal_cell = calculus.getSCell(calculus.myKSpace.sDim(cell), DGtal::PRIMAL, index);
            test_result &= (cell == primal_cell);
            const SCell& dual_cell = calculus.getSCell(calculus.dimension-calculus.myKSpace.sDim(cell), DGtal::DUAL, index);
            test_result &= (cell == dual_cell);
        }
        DGtal::trace.endBlock();

        FATAL_ERROR(test_result);
    }

    DGtal::trace.beginBlock("testing hodge");
    bool test_result = HodgeTester<Calculus, Calculus::dimension>::test(calculus);
    DGtal::trace.endBlock();

    FATAL_ERROR(test_result);
}

//DC Order->int (see above)
template <typename Calculus, int order>
struct DerivativeTester
{
    BOOST_STATIC_ASSERT(( order < (int)Calculus::dimension - 1 ));

    static bool test(const Calculus& calculus)
    {
        DGtal::trace.info() << "testing primal derivative composition order " << order << std::endl;

        { // test primal composition
            typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, order+1, DGtal::PRIMAL> FirstDerivative;
            FirstDerivative first_derivative = calculus.template derivative<order, DGtal::PRIMAL>();
            typedef DGtal::LinearOperator<Calculus, order+1, DGtal::PRIMAL, order+2, DGtal::PRIMAL> SecondDerivative;
            SecondDerivative second_derivative = calculus.template derivative<order+1, DGtal::PRIMAL>();
            typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, order+2, DGtal::PRIMAL> DoubleDerivative;
            DoubleDerivative double_derivative = second_derivative * first_derivative;
            if (!is_all_zero(double_derivative.myContainer)) return false;
        }

        DGtal::trace.info() << "testing dual derivative composition order " << order << std::endl;

        { // test dual composition
            typedef DGtal::LinearOperator<Calculus, order, DGtal::DUAL, order+1, DGtal::DUAL> FirstDerivative;
            FirstDerivative first_derivative = calculus.template derivative<order, DGtal::DUAL>();
            typedef DGtal::LinearOperator<Calculus, order+1, DGtal::DUAL, order+2, DGtal::DUAL> SecondDerivative;
            SecondDerivative second_derivative = calculus.template derivative<order+1, DGtal::DUAL>();
            typedef DGtal::LinearOperator<Calculus, order, DGtal::DUAL, order+2, DGtal::DUAL> DoubleDerivative;
            DoubleDerivative double_derivative = second_derivative * first_derivative;
            if (!is_all_zero(double_derivative.myContainer)) return false;
        }

        /*
        DGtal::trace.info() << "testing liebnitz rule order " << order << std::endl;

        {
            typedef DGtal::LinearOperator<Calculus, order, DGtal::PRIMAL, order+1, DGtal::PRIMAL> Derivative;
            Derivative derivative = calculus.template derivative<order, DGtal::PRIMAL>();

            typedef DGtal::KForm<Calculus, order, DGtal::PRIMAL> InputForm;
            typedef DGtal::KForm<Calculus, order+1, DGtal::PRIMAL> OutputForm;
            InputForm alpha(calculus), beta(calculus), gamma(calculus);

            for (int kk=0; kk<calculus.kFormLength(order, DGtal::PRIMAL); kk++)
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
    static bool test(const Calculus& )
    {
        return true;
    }
};

template <typename DigitalSet, typename LinearAlgebraBackend>
void
test_derivative(int domain_size=10)
{
    BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDigitalSet<DigitalSet> ));

    typedef typename DigitalSet::Domain Domain;
    typedef typename DigitalSet::Point Point;
    DGtal::trace.info() << "dimension=" << Point::dimension << std::endl;
    Domain domain(Point(), Point::diagonal(domain_size-1));
    DGtal::trace.info() << "domain=" << domain << std::endl;

    DigitalSet set(domain);
    for (typename Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        if (random()%4!=0) continue;
        const typename Domain::Point& point = *di;
        set.insertNew(point);
    }
    DGtal::trace.info() << "domain.size()=" << domain.size() << std::endl;
    DGtal::trace.info() << "set.size()=" << set.size() << std::endl;

    typedef DGtal::DiscreteExteriorCalculus<Domain::Space::dimension, LinearAlgebraBackend> Calculus;
    Calculus calculus(set);
    {
        typename Calculus::Properties properties = calculus.getProperties();
        DGtal::trace.info() << "properties.size()=" << properties.size() << std::endl;
    }

    DGtal::trace.beginBlock("testing derivative");
    bool test_result = DerivativeTester<Calculus, Calculus::dimension-2>::test(calculus);
    DGtal::trace.endBlock();

    FATAL_ERROR(test_result);
}

template <typename LinearAlgebraBackend>
void
test_concepts()
{
    DGtal::trace.beginBlock("concepts");

    { // 2d
        typedef DGtal::DiscreteExteriorCalculus<2, LinearAlgebraBackend> Calculus;
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm2> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalVectorField> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualVectorField> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative1> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge2> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity2> ));
    }

    { // 3d
        typedef DGtal::DiscreteExteriorCalculus<3, LinearAlgebraBackend> Calculus;
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalForm3> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualForm3> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalVectorField> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualVectorField> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalDerivative2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualDerivative2> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalHodge3> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualHodge3> ));

        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::PrimalIdentity3> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity0> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity1> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity2> ));
        BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDiscreteExteriorCalculusVectorSpace<typename Calculus::DualIdentity3> ));
    }

    DGtal::trace.endBlock();
}

template <typename LinearAlgebraBackend>
void
test_hodge_sign()
{
    DGtal::trace.beginBlock("testing hodge sign");

    {
        typedef DGtal::DiscreteExteriorCalculus<2, LinearAlgebraBackend> Calculus;
        const DGtal::Z2i::Domain domain;
        const DGtal::Z2i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef DGtal::Z2i::Point Point;
        typedef typename Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0), KSpace::NEG), DGtal::DUAL ) == -1 );
        // primal horizontal edge, dual vertical edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::POS), DGtal::DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0), KSpace::NEG), DGtal::DUAL ) == 1 );
        // primal vectical edge, dual horizontal edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::POS), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::NEG), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1), KSpace::NEG), DGtal::DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1), KSpace::NEG), DGtal::DUAL ) == -1 );
    }

    {
        typedef DGtal::DiscreteExteriorCalculus<3, LinearAlgebraBackend> Calculus;
        const DGtal::Z3i::Domain domain;
        const DGtal::Z3i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef DGtal::Z3i::Point Point;
        typedef typename Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,0), KSpace::NEG), DGtal::DUAL ) == -1 );
        // primal edge, dual surfel
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,0), KSpace::NEG), DGtal::DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,0), KSpace::NEG), DGtal::DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,0,1), KSpace::NEG), DGtal::DUAL ) == -1 );
        // primal surfel, dual edge
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,0), KSpace::NEG), DGtal::DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(0,1,1), KSpace::NEG), DGtal::DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,0,1), KSpace::NEG), DGtal::DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::POS), DGtal::PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::NEG), DGtal::PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::POS), DGtal::DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.myKSpace.sCell(Point(1,1,1), KSpace::NEG), DGtal::DUAL ) == -1 );
    }

    DGtal::trace.endBlock();
}

template <typename LinearAlgebraBackend>
void
test_backend(const int& ntime, const int& maxdim)
{
    srandom(0);

    test_hodge_sign<LinearAlgebraBackend>();

    for (int kk=0; kk<ntime; kk++)
    {
        typedef DGtal::SpaceND<4, int> Space4;
        typedef DGtal::HyperRectDomain<Space4> Domain4;
        typedef DGtal::DigitalSetBySTLSet<Domain4> DigitalSet4;

        typedef DGtal::SpaceND<5, int> Space5;
        typedef DGtal::HyperRectDomain<Space5> Domain5;
        typedef DGtal::DigitalSetBySTLSet<Domain5> DigitalSet5;

        if (maxdim>=2) test_hodge<DGtal::Z2i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=3) test_hodge<DGtal::Z3i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=4) test_hodge<DigitalSet4, LinearAlgebraBackend>(5);
        if (maxdim>=5) test_hodge<DigitalSet5, LinearAlgebraBackend>(3);

        if (maxdim>=2) test_derivative<DGtal::Z2i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=3) test_derivative<DGtal::Z3i::DigitalSet, LinearAlgebraBackend>();
        if (maxdim>=4) test_derivative<DigitalSet4, LinearAlgebraBackend>(5);
        if (maxdim>=5) test_derivative<DigitalSet5, LinearAlgebraBackend>(3);
    }

    test_concepts<LinearAlgebraBackend>();
}

#endif

