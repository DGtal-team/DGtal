#include <string>
using namespace std;

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
using namespace DGtal;

#include <Eigen/Eigenvalues>

int main(int argc, char* argv[])
{
    trace.beginBlock("building calculus");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));

    typedef DigitalSetBySTLSet<Z2i::Domain> DigitalSet;
    DigitalSet set(domain);

    for (int kk=1; kk<9; kk++)
        for (int ll=1; ll<9; ll++)
            set.insertNew(Z2i::Point(kk,ll));

    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(set);

    trace.info() << calculus << endl;

    trace.endBlock();

    trace.beginBlock("building laplacian");

    Calculus::DualDerivative0 d0 = calculus.derivative<0, DUAL>();
    Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    Calculus::DualHodge1 hodge1 = calculus.dualHodge<1>();
    Calculus::PrimalHodge2 hodge2p = calculus.primalHodge<2>();
    Calculus::DualIdentity0 laplacian = -1 * hodge2p * d1p * hodge1 * d0;

    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d1p = " << d1p << endl;
    trace.info() << "hodge2p = " << hodge2p << endl;
    trace.info() << "laplacian = " << laplacian << endl;

    trace.info() << laplacian.myContainer << endl;

    {
        Calculus::Accum accum(calculus);

        Board2D board;
        board << domain;
        board << accum;
        board.saveSVG("chladni_laplacian_calculus.svg");
    }

    trace.endBlock();

    trace.beginBlock("finding laplacian");

    typedef Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
    //typedef Eigen::EigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
    EigenSolverMatrix eigen_solver(laplacian.myContainer, true);
    trace.info() << eigen_solver.eigenvalues() << endl;

    trace.endBlock();

    return 0;
}

