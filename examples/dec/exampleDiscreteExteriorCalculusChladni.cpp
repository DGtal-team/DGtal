#include <sstream>
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

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(10,10));

    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(domain);

    // bottom linear structure
    // left and right Dirichlet boundary condition
    for (int kk=2; kk<=20; kk++)
        calculus.insertSCell( calculus.kspace.sCell(Z2i::Point(kk, 1)) );

    // top linear structure
    // left Neumann boundary condition, right Dirichlet boundary condition
    for (int kk=3; kk<=20; kk++)
        calculus.insertSCell( calculus.kspace.sCell(Z2i::Point(kk, 21)) );

    for (int kk=3; kk<20; kk++)
        for (int ll=3; ll<20; ll++)
        {
            if (kk==11 && ll==11) continue;
            calculus.insertSCell( calculus.kspace.sCell(Z2i::Point(kk, ll)) );
        }

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

    {
        Calculus::DualForm0 aa(calculus);
        aa.myContainer.fill(0);
        Calculus::DualForm2 bb(calculus);
        bb.myContainer.fill(1);

        Calculus::Accum accum(calculus);
        aa.applyToAccum(accum);
        bb.applyToAccum(accum);

        Board2D board;
        board << domain;
        board << accum;
        board.saveSVG("chladni_calculus.svg");
    }

    trace.endBlock();

    trace.beginBlock("finding laplacian eigen vectors");

    typedef Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
    const EigenSolverMatrix eigen_solver(laplacian.myContainer);

    const Eigen::VectorXd eigen_values = eigen_solver.eigenvalues();
    const Eigen::MatrixXd eigen_vectors = eigen_solver.eigenvectors();
    for (int kk=0; kk<laplacian.myContainer.rows(); kk++)
    {
        Calculus::Scalar eigen_value = eigen_values(kk, 0);

        const Eigen::VectorXd eigen_vector = eigen_vectors.col(kk);
        const Calculus::DualForm0 eigen_form = Calculus::DualForm0(calculus, eigen_vector);
        std::stringstream ss;
        ss << "chladni_eigen_" << kk << ".svg";
        const std::string filename = ss.str();
        ss << "chladni_eigen_vector_" << kk << ".svg";
        trace.info() << kk << " " << eigen_value << " " << sqrt(eigen_value) << " " << eigen_vector.minCoeff() << "0" << eigen_vector.maxCoeff() << endl;

        Calculus::Accum accum(calculus);
        eigen_form.applyToAccum(accum);

        Board2D board;
        board << domain;
        board << CustomStyle("AllSCellMap", new AllSCellMapStyle2D(eigen_vectors.minCoeff(),eigen_vectors.maxCoeff()));
        board << accum;
        board.saveSVG(filename.c_str());
    }

    trace.endBlock();

    return 0;
}

