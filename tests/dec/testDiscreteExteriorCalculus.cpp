
#include "DGtal/dec/Calculus.h"
#include "DGtal/dec/LinearOperator.h"
#include "DGtal/dec/KForm.h"
#include "DGtal/dec/Duality.h"
#include "DGtal/dec/CDualityLinearAlgebra.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;

template <typename Container>
bool
all_zero(const Container& container)
{
    for (typename Container::Index ii=0; ii<container.rows(); ii++)
        for (typename Container::Index jj=0; jj<container.cols(); jj++)
            if (container(ii,jj) != 0)
                return false;
    return true;
}

template <typename Calculus, Order order>
struct HodgeTester
{
    BOOST_STATIC_ASSERT(( order <= Calculus::dimension ));

    static bool test(const Calculus& calculus)
    {
        typedef LinearOperator<Calculus, order, PRIMAL, Calculus::dimension-order, DUAL> PrimalHodge;
        typedef LinearOperator<Calculus, Calculus::dimension-order, DUAL, order, PRIMAL> DualHodge;
        const PrimalHodge primal_hodge = calculus.template primalHodge<order>();
        const DualHodge dual_hodge = calculus.template dualHodge<Calculus::dimension-order>();

        trace.info() << "testing primal to primal hodge composition order " << order << endl;

        { // test primal to primal composition
            typedef LinearOperator<Calculus, order, PRIMAL, order, PRIMAL> PrimalPrimal;
            PrimalPrimal primal_primal = dual_hodge * primal_hodge;
            trace.info() << primal_primal.container << endl;
            primal_primal.container -= pow(-1, order*(Calculus::dimension-order)) * PrimalPrimal::Container::Identity(primal_primal.container.rows(), primal_primal.container.cols());
            if (!all_zero(primal_primal.container)) return false;
        }

        trace.info() << "testing dual to dual hodge composition order " << order << endl;

        { // test dual to dual composition
            typedef LinearOperator<Calculus, Calculus::dimension-order, DUAL, Calculus::dimension-order, DUAL> DualDual;
            DualDual dual_dual = primal_hodge * dual_hodge;
            dual_dual.container -= pow(-1, order*(Calculus::dimension-order)) * DualDual::Container::Identity(dual_dual.container.rows(), dual_dual.container.cols());
            if (!all_zero(dual_dual.container)) return false;
        }

        return HodgeTester<Calculus, order-1>::test(calculus);
    }
};

template <>
template <typename Calculus>
struct HodgeTester<Calculus, -1>
{
    static bool test(const Calculus& calculus)
    {
        return true;
    }
};

template <typename DigitalSet>
void
test_hodge(int domain_size=10)
{
    BOOST_CONCEPT_ASSERT(( CDigitalSet<DigitalSet> ));
    trace.beginBlock("testing hodge");

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

    typedef DiscreteExteriorCalculus<Domain> Calculus;
    Calculus calculus(set);

    bool test_result = HodgeTester<Calculus, Calculus::dimension>::test(calculus);
    trace.endBlock();

    FATAL_ERROR(test_result);
}

template <typename Calculus, Order order>
struct DerivativeTester
{
    BOOST_STATIC_ASSERT(( order < Calculus::dimension-1 ));

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
            if (!all_zero(double_derivative.container)) return false;
        }

        trace.info() << "testing dual derivative composition order " << order << endl;

        { // test dual composition
            typedef LinearOperator<Calculus, order, DUAL, order+1, DUAL> FirstDerivative;
            FirstDerivative first_derivative = calculus.template derivative<order, DUAL>();
            typedef LinearOperator<Calculus, order+1, DUAL, order+2, DUAL> SecondDerivative;
            SecondDerivative second_derivative = calculus.template derivative<order+1, DUAL>();
            typedef LinearOperator<Calculus, order, DUAL, order+2, DUAL> DoubleDerivative;
            DoubleDerivative double_derivative = second_derivative * first_derivative;
            if (!all_zero(double_derivative.container)) return false;
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
                alpha.container(kk) = ak;
                beta.container(kk) = bk;
                gamma.container(kk) = ak*bk;
            }

        }
        */

        return DerivativeTester<Calculus, order-1>::test(calculus);
    }
};

template <>
template <typename Calculus>
struct DerivativeTester<Calculus, -1>
{
    static bool test(const Calculus& calculus)
    {
        return true;
    }
};

template <typename DigitalSet>
void
test_derivative(int domain_size=10)
{
    BOOST_CONCEPT_ASSERT(( CDigitalSet<DigitalSet> ));
    trace.beginBlock("testing derivative");

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

    typedef DiscreteExteriorCalculus<Domain> Calculus;
    Calculus calculus(set);

    bool test_result = DerivativeTester<Calculus, Calculus::dimension-2>::test(calculus);
    trace.endBlock();

    FATAL_ERROR(test_result);
}

void
test_concepts()
{
    trace.beginBlock("concepts");

    { // 2d
        typedef DiscreteExteriorCalculus<Z2i::Domain> Calculus;
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm2> ));

        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualDerivative1> ));

        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge2> ));
    }

    { // 3d
        typedef DiscreteExteriorCalculus<Z3i::Domain> Calculus;
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalForm3> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualForm3> ));

        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalDerivative2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualDerivative0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualDerivative1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualDerivative2> ));

        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::PrimalHodge3> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge0> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge1> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge2> ));
        BOOST_CONCEPT_ASSERT(( CDualityLinearAlgebra<Calculus::DualHodge3> ));
    }

    trace.endBlock();
}

void
test_hodge_sign()
{
    trace.beginBlock("testing hodge sign");

    {
        typedef DiscreteExteriorCalculus<Z2i::Domain> Calculus;
        const Z2i::Domain domain;
        const Z2i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef Z2i::Point Point;
        typedef Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0), KSpace::NEG), DUAL ) == -1 );
        // primal horizontal edge, dual vertical edge
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0), KSpace::POS), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0), KSpace::NEG), DUAL ) == 1 );
        // primal vectical edge, dual horizontal edge
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1), KSpace::POS), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1), KSpace::NEG), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1), KSpace::NEG), DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1), KSpace::NEG), DUAL ) == -1 );
    }

    {
        typedef DiscreteExteriorCalculus<Z3i::Domain> Calculus;
        const Z3i::Domain domain;
        const Z3i::DigitalSet set(domain);
        const Calculus calculus(set);
        typedef Z3i::Point Point;
        typedef Calculus::KSpace KSpace;
        // primal point, dual cell
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,0), KSpace::NEG), DUAL ) == -1 );
        // primal edge, dual surfel
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,0,1), KSpace::NEG), DUAL ) == -1 );
        // primal surfel, dual edge
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,0), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,0), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,0), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,0), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(0,1,1), KSpace::NEG), DUAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,0,1), KSpace::NEG), DUAL ) == -1 );
        // primal cell, dual point
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,1), KSpace::POS), PRIMAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,1), KSpace::NEG), PRIMAL ) == -1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,1), KSpace::POS), DUAL ) == 1 );
        FATAL_ERROR( calculus.hodgeSign( calculus.kspace.sCell(Point(1,1,1), KSpace::NEG), DUAL ) == -1 );
    }

    trace.endBlock();
}

int
main(int argc, char* argv[])
{
    test_hodge_sign();

    srandom(0);
    for (int kk=0; kk<2; kk++)
    {
        typedef SpaceND<4, int> Space4;
        typedef HyperRectDomain<Space4> Domain4;
        typedef DigitalSetBySTLSet<Domain4> DigitalSet4;

        typedef SpaceND<5, int> Space5;
        typedef HyperRectDomain<Space5> Domain5;
        typedef DigitalSetBySTLSet<Domain5> DigitalSet5;

        test_hodge<Z2i::DigitalSet>();
        test_hodge<Z3i::DigitalSet>();
        test_hodge<DigitalSet4>(5);
        test_hodge<DigitalSet5>(3);

        test_derivative<Z2i::DigitalSet>();
        test_derivative<Z3i::DigitalSet>();
        test_derivative<DigitalSet4>(5);
        test_derivative<DigitalSet5>(3);
    }

    test_concepts();

    return 0;
}

