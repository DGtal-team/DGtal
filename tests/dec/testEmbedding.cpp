#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"

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
    typedef DiscreteExteriorCalculus<1, 1, EigenLinearAlgebraBackend> Calculus1D;
    typedef DiscreteExteriorCalculus<1, 2, EigenLinearAlgebraBackend> Calculus2D;
    typedef DiscreteExteriorCalculus<1, 3, EigenLinearAlgebraBackend> Calculus3D;
    typedef Viewer3D<Calculus3D::KSpace::Space, Calculus3D::KSpace> Viewer;

    QApplication app(argc, argv);
    Z3i::KSpace kspace_3d;

    Viewer viewer1(kspace_3d);
    viewer1.show();
    viewer1.setWindowTitle("embedding_1d_calculus_3d");

    {
        trace.beginBlock("1d manifold embedding");

        Calculus1D calculus_1d;
        for (int kk=0; kk<31; kk++)
        {
            Calculus1D::KSpace::Point point;
            point[0] = kk;
            calculus_1d.insertSCell( calculus_1d.myKSpace.sCell(point), kk == 0 || kk == 30 ? 1/2. : 1 );
        }
        trace.info() << "calculus_1d=" << calculus_1d << endl;

        Calculus2D calculus_2d;
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(6,0)), 1/2. );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(6,1), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(6,2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(7,2), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(8,2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(8,1), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(8,0)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(8,-1), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(8,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(7,-2), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(6,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(5,-2), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(4,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(3,-2), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(2,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(1,-2), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(0,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-1,-2), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-2,-2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-2,-1), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-2,0)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-2,1), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-2,2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(-1,2), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(0,2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(1,2), Calculus2D::KSpace::NEG) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(2,2)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(2,1), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(2,0)) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(1,0), Calculus2D::KSpace::POS) );
        calculus_2d.insertSCell( calculus_2d.myKSpace.sCell(Z2i::Point(0,0)), 1/2.);
        trace.info() << "calculus_2d=" << calculus_2d << endl;

        {
            Board2D board;
            board << Z2i::Domain(Z2i::Point(-2,-2), Z2i::Point(4,1));
            board << calculus_2d;
            board.saveSVG("embedding_1d_calculus_2d.svg");
        }

        Calculus3D calculus_3d;
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(0,0,0)), 1/2. );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(1,0,0), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,0,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(3,0,0), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,0,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,1,0), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,2,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,3,0), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(4,4,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(3,4,0), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,4,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(1,4,0), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(0,4,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(0,3,0), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(0,2,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(1,2,0), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,2,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,2,1), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,2,2)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,3,2), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,4,2)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,5,2), Calculus3D::KSpace::NEG) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,6,2)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,6,1), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,6,0)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,6,-1), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,6,-2)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,5,-2), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,4,-2)) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,3,-2), Calculus3D::KSpace::POS) );
        calculus_3d.insertSCell( calculus_3d.myKSpace.sCell(Z3i::Point(2,2,-2)), 1/2. );
        trace.info() << "calculus_3d=" << calculus_3d << endl;

        Display3DFactory<Calculus3D::KSpace::Space, Calculus3D::KSpace>::draw(viewer1, calculus_3d);
        viewer1.camera()->setPosition( Vec(2,2,2) );
        viewer1.camera()->setUpVector( Vec(0,0,1), false );
        viewer1.camera()->lookAt( Vec(0,0,0) );
        viewer1 << Viewer::updateDisplay;

        const Calculus1D::PrimalIdentity0 primal_laplace_1d = calculus_1d.laplace<PRIMAL>();
        const Calculus2D::PrimalIdentity0 primal_laplace_2d = calculus_2d.laplace<PRIMAL>();
        const Calculus3D::PrimalIdentity0 primal_laplace_3d = calculus_3d.laplace<PRIMAL>();
        trace.info() << "primal_laplace_1d=" << primal_laplace_1d << endl;
        trace.info() << "primal_laplace_2d=" << primal_laplace_2d << endl;
        trace.info() << "primal_laplace_3d=" << primal_laplace_3d << endl;
        trace.info() << "primal_laplace_container=" << endl << MatrixXd(primal_laplace_1d.myContainer) << endl;
        FATAL_ERROR( equal(primal_laplace_1d, primal_laplace_2d) );
        FATAL_ERROR( equal(primal_laplace_1d, primal_laplace_3d) );

        const Calculus1D::DualIdentity0 dual_laplace_1d = calculus_1d.laplace<DUAL>();
        const Calculus2D::DualIdentity0 dual_laplace_2d = calculus_2d.laplace<DUAL>();
        const Calculus3D::DualIdentity0 dual_laplace_3d = calculus_3d.laplace<DUAL>();
        trace.info() << "dual_laplace_1d=" << dual_laplace_1d << endl;
        trace.info() << "dual_laplace_2d=" << dual_laplace_2d << endl;
        trace.info() << "dual_laplace_3d=" << dual_laplace_3d << endl;
        trace.info() << "dual_laplace_container=" << endl << MatrixXd(dual_laplace_1d.myContainer) << endl;
        FATAL_ERROR( equal(dual_laplace_1d, dual_laplace_2d) );
        FATAL_ERROR( equal(dual_laplace_1d, dual_laplace_3d) );

        trace.endBlock();
    }

    return app.exec();
}

