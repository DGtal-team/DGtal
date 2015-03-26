#include "common.h"

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/io/boards/Board2D.h"

using namespace DGtal;
using std::endl;

void propa_2d()
{
    trace.beginBlock("2d propagation");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(29,29));

    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;
    //const Calculus calculus(generateRingSet(domain), false);
    const Calculus calculus(generateDiskSet(domain), false);

    {
        Board2D board;
        board << domain;
        board << calculus;
        board.saveSVG("propagation_calculus.svg");
    }

    const Calculus::DualIdentity0 laplace = calculus.dualLaplace() + 1e-8 * calculus.identity<0, DUAL>();
    trace.info() << "laplace = " << laplace << endl;

    trace.beginBlock("finding eigen pairs");
    typedef Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
    const EigenSolverMatrix eigen_solver(laplace.myContainer);

    const Eigen::VectorXd eigen_values = eigen_solver.eigenvalues();
    const Eigen::MatrixXd eigen_vectors = eigen_solver.eigenvectors();
    trace.endBlock();

    const Calculus::Scalar cc = 8; // px/s
    const Eigen::VectorXd angular_frequencies = cc * eigen_values.array().sqrt();

    Eigen::VectorXcd initial_wave(calculus.kFormLength(0, DUAL));
    //for (int xx=3; xx<7; xx++)
    //for (int yy=10; yy<20; yy++)
    for (int xx=13; xx<17; xx++)
    for (int yy=13; yy<17; yy++)
    {
        const Z2i::Point point(xx,yy);
        const Calculus::Cell cell = calculus.myKSpace.uSpel(point);
        const Calculus::Index index = calculus.getCellIndex(cell);
        //initial_wave(index) = std::exp(std::complex<double>(0,M_PI/2.*((xx-14.5)/2.-(yy-14.5)/4.)));
        initial_wave(index) = std::exp(std::complex<double>(0,M_PI/2.*(xx-14.5)/1.5));
    }

    {
        Board2D board;
        board << domain;
        board << CustomStyle("KForm", new KFormStyle2D(-1, 1));
        board << Calculus::DualForm0(calculus, (initial_wave.array().conjugate()*initial_wave.array()).real().sqrt());
        board.saveSVG("propagation_wave_initial_coarse.svg");
    }

    Eigen::VectorXcd initial_projections = eigen_vectors.transpose() * initial_wave;

    // low pass
    const Calculus::Scalar lambda_cutoff = 5.;
    const Calculus::Scalar angular_frequency_cutoff = 2*M_PI * cc / lambda_cutoff;
    int cutted = 0;
    for (int kk=0; kk<initial_projections.rows(); kk++)
    {
        const Calculus::Scalar angular_frequency = angular_frequencies(kk);
        if (angular_frequency < angular_frequency_cutoff) continue;
        initial_projections(kk) = 0;
        cutted ++;
    }
    trace.info() << "cutted = " << cutted << "/" << initial_projections.rows() << endl;

    {
        const Eigen::VectorXcd wave = eigen_vectors * initial_projections;
        Board2D board;
        board << domain;
        board << CustomStyle("KForm", new KFormStyle2D(-1, 1));
        board << Calculus::DualForm0(calculus, (wave.array().conjugate()*wave.array()).real().sqrt());
        board.saveSVG("propagation_wave_initial.svg");
    }

    trace.progressBar(0,100);
    for (int kk=0; kk<100; kk++)
    {
        const Calculus::Scalar time = kk/20.;
        const Eigen::VectorXcd current_projections = (angular_frequencies * std::complex<double>(0,time)).array().exp() * initial_projections.array();
        const Eigen::VectorXcd current_wave = eigen_vectors * current_projections;

        std::stringstream ss;
        ss << "propagation_wave_" << kk << ".svg";

        Board2D board;
        board << domain;
        board << CustomStyle("KForm", new KFormStyle2D(-1, 1));
        board << Calculus::DualForm0(calculus, (current_wave.array().conjugate()*current_wave.array()).real().sqrt());
        board.saveSVG(ss.str().c_str());

        trace.progressBar(kk+1,100);
    }
    trace.info() << endl;

    trace.endBlock();
}

int main(int argc, char* argv[])
{
    propa_2d();
    return 0;
}
