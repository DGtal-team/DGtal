#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/DiscreteExteriorCalculusFactory.h"

#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

using namespace DGtal;
using namespace std;
using namespace Eigen;

template <typename OperatorAA, typename OperatorBB>
bool
equal(const OperatorAA& aa, const OperatorBB& bb)
{
    return MatrixXd(aa.myContainer) == MatrixXd(bb.myContainer);
}

int main(int argc, char* argv[])
{
    typedef Viewer3D<Z3i::Space, Z3i::KSpace> Viewer;
    typedef DiscreteExteriorCalculusFactory<EigenLinearAlgebraBackend> CalculusFactory;

    QApplication app(argc, argv);
    Z3i::KSpace kspace_3d;

    Viewer viewer1(kspace_3d);
    viewer1.show();
    viewer1.setWindowTitle("embedding_1d_calculus_3d");
    viewer1.camera()->setPosition( Vec(2,2,2) );
    viewer1.camera()->setUpVector( Vec(0,0,1), false );
    viewer1.camera()->lookAt( Vec(0,0,0) );

    Viewer viewer2(kspace_3d);
    viewer2.show();
    viewer2.setWindowTitle("embedding_2d_calculus_3d");
    viewer2.camera()->setPosition( Vec(2,2,2) );
    viewer2.camera()->setUpVector( Vec(0,0,2), false );
    viewer2.camera()->lookAt( Vec(0,0,0) );

    {
        trace.beginBlock("1d manifold embedding");

        typedef DiscreteExteriorCalculus<1, 1, EigenLinearAlgebraBackend> Calculus1D;
        typedef DiscreteExteriorCalculus<1, 2, EigenLinearAlgebraBackend> Calculus2D;
        typedef DiscreteExteriorCalculus<1, 3, EigenLinearAlgebraBackend> Calculus3D;

        typedef std::set<Calculus1D::SCell> SCells1D;
        Calculus1D calculus_1d_manual;
        SCells1D cells_1d;
        for (int kk=0; kk<31; kk++)
        {
            Calculus1D::KSpace::Point point;
            point[0] = kk;
            const Calculus1D::SCell cell = calculus_1d_manual.myKSpace.sCell(point);
            calculus_1d_manual.insertSCell( cell, kk == 0 || kk == 30 ? 1/2. : 1 );
            if (kk%2 != 0) cells_1d.insert(cell);
        }
        trace.info() << "calculus_1d_manual=" << calculus_1d_manual << endl;

        const Calculus1D calculus_1d_factory = CalculusFactory::createFromNSCells<1>(cells_1d.begin(), cells_1d.end(), true);
        trace.info() << "calculus_1d_factory=" << calculus_1d_factory << endl;

        Calculus2D calculus_2d_manual;
        {
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(6,0)), 1/2. );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(6,1), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(6,2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(7,2), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,1), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,0)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,-1), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(7,-2), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(6,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(5,-2), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(4,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(3,-2), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(2,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,-2), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(0,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-1,-2), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,-2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,-1), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,0)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,1), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-1,2), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(0,2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,2), Calculus2D::KSpace::POS) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(2,2)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(2,1), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(2,0)) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,0), Calculus2D::KSpace::NEG) );
            calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(0,0)), 1/2.);
        }
        trace.info() << "calculus_2d_manual=" << calculus_2d_manual << endl;

        {
            Board2D board;
            board << Z2i::Domain(Z2i::Point(-2,-2), Z2i::Point(4,1));
            board << calculus_2d_manual;
            board.saveSVG("embedding_1d_calculus_2d.svg");
        }

        typedef std::list<Calculus2D::SCell> SCells2D;
        SCells2D cells_2d;
        {
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(6,1), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(7,2), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,1), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(8,-1), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(7,-2), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(5,-2), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(3,-2), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,-2), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-1,-2), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,-1), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-2,1), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(-1,2), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,2), Calculus2D::KSpace::POS) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(2,1), Calculus2D::KSpace::NEG) );
            cells_2d.push_back( calculus_2d_manual.myKSpace.sCell(Z2i::Point(1,0), Calculus2D::KSpace::NEG) );
        }
        const Calculus2D calculus_2d_factory = CalculusFactory::createFromNSCells<1>(cells_2d.begin(), cells_2d.end());
        trace.info() << "calculus_2d_factory=" << calculus_2d_factory << endl;

        Calculus3D calculus_3d_manual;
        {
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(0,0,0)), 1/2. );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,0,0), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,0,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(3,0,0), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,0,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,1,0), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,2,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,3,0), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,4,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(3,4,0), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,4,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,4,0), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(0,4,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(0,3,0), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(0,2,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,2,0), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,2,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,2,1), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,2,2)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,3,2), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,4,2)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,5,2), Calculus3D::KSpace::POS) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,2)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,1), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,0)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,-1), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,-2)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,5,-2), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,4,-2)) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,3,-2), Calculus3D::KSpace::NEG) );
            calculus_3d_manual.insertSCell( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,2,-2)), 1/2. );
        }
        trace.info() << "calculus_3d_manual=" << calculus_3d_manual << endl;

        Display3DFactory<Calculus3D::KSpace::Space, Calculus3D::KSpace>::draw(viewer1, calculus_3d_manual);
        viewer1 << Viewer::updateDisplay;

        typedef std::vector<Calculus3D::SCell> SCells3D;
        SCells3D cells_3d;
        {
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,0,0), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(3,0,0), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,1,0), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(4,3,0), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(3,4,0), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,4,0), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(0,3,0), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(1,2,0), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,2,1), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,3,2), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,5,2), Calculus3D::KSpace::POS) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,1), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,6,-1), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,5,-2), Calculus3D::KSpace::NEG) );
            cells_3d.push_back( calculus_3d_manual.myKSpace.sCell(Z3i::Point(2,3,-2), Calculus3D::KSpace::NEG) );
        }
        const Calculus3D calculus_3d_factory = CalculusFactory::createFromNSCells<1>(cells_3d.begin(), cells_3d.end(), true);
        trace.info() << "calculus_3d_factory=" << calculus_3d_factory << endl;

        trace.beginBlock("checking operators");

        { // testing primal operators
            const Calculus1D::PrimalIdentity0 primal_laplace_1d = calculus_1d_manual.laplace<PRIMAL>();
            const Calculus2D::PrimalIdentity0 primal_laplace_2d = calculus_2d_manual.laplace<PRIMAL>();
            const Calculus3D::PrimalIdentity0 primal_laplace_3d = calculus_3d_manual.laplace<PRIMAL>();
            trace.info() << "primal_laplace_1d=" << primal_laplace_1d << endl;
            trace.info() << "primal_laplace_2d=" << primal_laplace_2d << endl;
            trace.info() << "primal_laplace_3d=" << primal_laplace_3d << endl;
            trace.info() << "primal_laplace_container=" << endl << MatrixXd(primal_laplace_1d.myContainer) << endl;
            FATAL_ERROR( equal(calculus_1d_manual.hodge<0, PRIMAL>(), calculus_2d_manual.hodge<0, PRIMAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<0, PRIMAL>(), calculus_3d_manual.hodge<0, PRIMAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<1, PRIMAL>(), calculus_2d_manual.hodge<1, PRIMAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<1, PRIMAL>(), calculus_3d_manual.hodge<1, PRIMAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.derivative<0, PRIMAL>(), calculus_3d_manual.derivative<0, PRIMAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.derivative<0, PRIMAL>(), calculus_2d_manual.derivative<0, PRIMAL>()) );
            FATAL_ERROR( equal(primal_laplace_1d, primal_laplace_2d) );
            FATAL_ERROR( equal(primal_laplace_1d, primal_laplace_3d) );
        }

        { // testing dual operators
            const Calculus1D::DualIdentity0 dual_laplace_1d = calculus_1d_manual.laplace<DUAL>();
            const Calculus2D::DualIdentity0 dual_laplace_2d = calculus_2d_manual.laplace<DUAL>();
            const Calculus3D::DualIdentity0 dual_laplace_3d = calculus_3d_manual.laplace<DUAL>();
            trace.info() << "dual_laplace_1d=" << dual_laplace_1d << endl;
            trace.info() << "dual_laplace_2d=" << dual_laplace_2d << endl;
            trace.info() << "dual_laplace_3d=" << dual_laplace_3d << endl;
            trace.info() << "dual_laplace_container=" << endl << MatrixXd(dual_laplace_1d.myContainer) << endl;
            FATAL_ERROR( equal(calculus_1d_manual.hodge<0, DUAL>(), calculus_2d_manual.hodge<0, DUAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<0, DUAL>(), calculus_3d_manual.hodge<0, DUAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<1, DUAL>(), calculus_2d_manual.hodge<1, DUAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.hodge<1, DUAL>(), calculus_3d_manual.hodge<1, DUAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.derivative<0, DUAL>(), calculus_2d_manual.derivative<0, DUAL>()) );
            FATAL_ERROR( equal(calculus_1d_manual.derivative<0, DUAL>(), calculus_3d_manual.derivative<0, DUAL>()) );
            FATAL_ERROR( equal(dual_laplace_1d, dual_laplace_2d) );
            FATAL_ERROR( equal(dual_laplace_1d, dual_laplace_3d) );
        }

        { // checking dual laplace factory calculus vs manual calculus
            FATAL_ERROR( equal(calculus_1d_manual.laplace<DUAL>(), calculus_1d_factory.laplace<DUAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.laplace<DUAL>(), calculus_2d_factory.laplace<DUAL>()) );
            FATAL_ERROR( equal(calculus_3d_manual.laplace<DUAL>(), calculus_3d_factory.laplace<DUAL>()) );
        }

        trace.endBlock();

        trace.beginBlock("checking border");

        { // 2d ambient border
            const Calculus2D calculus_2d_factory_no_border = CalculusFactory::createFromNSCells<1>(cells_2d.begin(), cells_2d.end(), false);
            trace.info() << "calculus_2d_factory_no_border=" << calculus_2d_factory_no_border << endl;
            trace.info() << "calculus_2d_factory=" << calculus_2d_factory << endl;
            FATAL_ERROR( calculus_2d_factory.containsCell(calculus_2d_factory.myKSpace.uCell(Z2i::Point(6,0))) );
            FATAL_ERROR( !calculus_2d_factory_no_border.containsCell(calculus_2d_factory_no_border.myKSpace.uCell(Z2i::Point(6,0))) );
            FATAL_ERROR( calculus_2d_factory.containsCell(calculus_2d_factory.myKSpace.uCell(Z2i::Point(0,0))) );
            FATAL_ERROR( !calculus_2d_factory_no_border.containsCell(calculus_2d_factory_no_border.myKSpace.uCell(Z2i::Point(0,0))) );
        }

        { // 3d ambient border
            const Calculus3D calculus_3d_factory_no_border = CalculusFactory::createFromNSCells<1>(cells_3d.begin(), cells_3d.end(), false);
            trace.info() << "calculus_3d_factory_no_border=" << calculus_3d_factory_no_border << endl;
            trace.info() << "calculus_3d_factory=" << calculus_3d_factory << endl;
            FATAL_ERROR( calculus_3d_factory.containsCell(calculus_3d_factory.myKSpace.uCell(Z3i::Point(2,2,-2))) );
            FATAL_ERROR( !calculus_3d_factory_no_border.containsCell(calculus_3d_factory_no_border.myKSpace.uCell(Z3i::Point(2,2,-2))) );
            FATAL_ERROR( calculus_3d_factory.containsCell(calculus_3d_factory.myKSpace.uCell(Z3i::Point(0,0,0))) );
            FATAL_ERROR( !calculus_3d_factory_no_border.containsCell(calculus_3d_factory_no_border.myKSpace.uCell(Z3i::Point(0,0,0))) );
        }

        trace.endBlock();


        trace.endBlock();
    }

    {
        trace.beginBlock("2d manifold embedding");

        typedef DiscreteExteriorCalculus<2, 2, EigenLinearAlgebraBackend> Calculus2D;
        typedef DiscreteExteriorCalculus<2, 3, EigenLinearAlgebraBackend> Calculus3D;

        Calculus2D calculus_2d_manual;
        {
            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx,yy)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx,yy+4)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx,yy+8)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx,yy+12)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx,yy+16)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx+4,yy)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx+8,yy)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx+12,yy)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_2d_manual.insertSCell( calculus_2d_manual.myKSpace.sCell(Z2i::Point(xx+16,yy)) );
        }
        trace.info() << "calculus_2d_manual=" << calculus_2d_manual << endl;

        {
            Board2D board;
            board << Z2i::Domain(Z2i::Point(-1,-1), Z2i::Point(2,10));
            board << calculus_2d_manual;
            board.saveSVG("embedding_2d_calculus_2d.svg");
        }

        typedef std::list<Calculus2D::SCell> SCells2D;
        SCells2D cells_2d;
        for (int kk=0; kk<calculus_2d_manual.kFormLength(2, PRIMAL); kk++) cells_2d.push_back( calculus_2d_manual.getSCell(2, PRIMAL, kk) );
        Calculus2D calculus_2d_factory = CalculusFactory::createFromNSCells<2>(cells_2d.begin(), cells_2d.end(), true);
        calculus_2d_factory.resetSizeRatios();
        trace.info() << "calculus_2d_factory=" << calculus_2d_factory << endl;

        Calculus3D calculus_3d;
        {
            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(xx,yy,0)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(xx,4,yy)) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(xx,4-yy,4),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    yy%2 != 0 ? Calculus3D::KSpace::NEG : // y-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(xx,-yy,4),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    yy%2 != 0 ? Calculus3D::KSpace::NEG : // y-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(xx,-4,4-yy),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    yy%2 != 0 ? Calculus3D::KSpace::NEG : // y-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,yy,-xx),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::POS : // surfels
                    xx%2 != 0 ? Calculus3D::KSpace::NEG : // x-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4-xx,yy,-4),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    xx%2 != 0 ? Calculus3D::KSpace::NEG : // x-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(-xx,yy,-4),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    xx%2 != 0 ? Calculus3D::KSpace::NEG : // x-edge
                    Calculus3D::KSpace::POS) );

            for (int xx=0; xx<=4; xx++)
            for (int yy=0; yy<=4; yy++)
                calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(-4,yy,-4+xx),
                    xx%2 != 0 && yy%2 != 0 ? Calculus3D::KSpace::NEG : // surfels
                    xx%2 != 0 ? Calculus3D::KSpace::POS : // x-edge
                    Calculus3D::KSpace::POS) );
        }
        trace.info() << "calculus_3d=" << calculus_3d << endl;

        Display3DFactory<Calculus3D::KSpace::Space, Calculus3D::KSpace>::draw(viewer2, calculus_3d);
        viewer2 << Viewer::updateDisplay;

        { // check primal operators
            const Calculus2D::PrimalIdentity0 primal_laplace_2d = calculus_2d_manual.laplace<PRIMAL>();
            const Calculus3D::PrimalIdentity0 primal_laplace_3d = calculus_3d.laplace<PRIMAL>();
            trace.info() << "primal_laplace_2d=" << primal_laplace_2d << endl;
            trace.info() << "primal_laplace_3d=" << primal_laplace_3d << endl;
            trace.info() << "primal_laplace_container=" << endl << MatrixXd(primal_laplace_2d.myContainer) << endl;
            FATAL_ERROR( equal(calculus_2d_manual.hodge<0,PRIMAL>(), calculus_3d.hodge<0,PRIMAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.hodge<1,PRIMAL>(), calculus_3d.hodge<1,PRIMAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.hodge<2,PRIMAL>(), calculus_3d.hodge<2,PRIMAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.derivative<0,PRIMAL>(), calculus_3d.derivative<0,PRIMAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.derivative<1,PRIMAL>(), calculus_3d.derivative<1,PRIMAL>()) );
            FATAL_ERROR( equal(primal_laplace_2d, primal_laplace_3d) );
        }

        { // check dual operators
            const Calculus2D::DualIdentity0 dual_laplace_2d = calculus_2d_manual.laplace<DUAL>();
            const Calculus3D::DualIdentity0 dual_laplace_3d = calculus_3d.laplace<DUAL>();
            trace.info() << "dual_laplace_2d=" << dual_laplace_2d << endl;
            trace.info() << "dual_laplace_3d=" << dual_laplace_3d << endl;
            trace.info() << "dual_laplace_container=" << endl << MatrixXd(dual_laplace_2d.myContainer) << endl;
            FATAL_ERROR( equal(calculus_2d_manual.hodge<0,DUAL>(), calculus_3d.hodge<0,DUAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.hodge<1,DUAL>(), calculus_3d.hodge<1,DUAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.hodge<2,DUAL>(), calculus_3d.hodge<2,DUAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.derivative<0,DUAL>(), calculus_3d.derivative<0,DUAL>()) );
            FATAL_ERROR( equal(calculus_2d_manual.derivative<1,DUAL>(), calculus_3d.derivative<1,DUAL>()) );
            FATAL_ERROR( equal(dual_laplace_2d, dual_laplace_3d) );
        }

        { // checking dual laplace factory calculus vs manual calculus
            FATAL_ERROR( equal(calculus_2d_manual.laplace<DUAL>(), calculus_2d_factory.laplace<DUAL>()) );
            //FATAL_ERROR( equal(calculus_3d_manual.laplace<DUAL>(), calculus_3d_factory.laplace<DUAL>()) );
        }

        trace.endBlock();
    }

    return app.exec();
}

