#include "common.h"

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/io/boards/Board2D.h"

template <typename Calculus>
typename Calculus::Scalar
sample_dual_0_form(const typename Calculus::Properties& properties, const typename Calculus::DualForm0& form, const typename Calculus::Point& point)
{
    const typename Calculus::Cell cell = form.myCalculus->myKSpace.uSpel(point);
    const typename Calculus::Properties::const_iterator iter = properties.find(cell);
    if (iter == properties.end()) return 0;

    const typename Calculus::Index index = iter->second.index;
    return form.myContainer(index);
}

using namespace DGtal;
using std::endl;

void propa_2d()
{
    trace.beginBlock("2d propagation");

    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;

    const Calculus::Scalar cc = 8; // px/s
    trace.info() << "cc = " << cc << endl;

    {
        trace.beginBlock("solving time dependent equation");

        const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(29,29));
        const Calculus calculus(generateDiskSet(domain), false);

        const Calculus::DualIdentity0 laplace = calculus.dualLaplace() + 1e-8 * calculus.identity<0, DUAL>();
        trace.info() << "laplace = " << laplace << endl;

        trace.beginBlock("finding eigen pairs");
        typedef Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
        const EigenSolverMatrix eigen_solver(laplace.myContainer);

        const Eigen::VectorXd eigen_values = eigen_solver.eigenvalues();
        const Eigen::MatrixXd eigen_vectors = eigen_solver.eigenvectors();
        trace.endBlock();

        const Eigen::VectorXd angular_frequencies = cc * eigen_values.array().sqrt();

        Eigen::VectorXcd initial_wave = Eigen::VectorXcd::Zero(calculus.kFormLength(0, DUAL));
        //for (int xx=3; xx<7; xx++)
        //for (int yy=10; yy<20; yy++)
        for (int xx=13; xx<17; xx++)
        for (int yy=13; yy<17; yy++)
        {
            const Z2i::Point point(xx,yy);
            const Calculus::Cell cell = calculus.myKSpace.uSpel(point);
            const Calculus::Index index = calculus.getCellIndex(cell);
            //initial_wave(index) = std::exp(std::complex<double>(0,M_PI/2.*((xx-14.5)/2.-(yy-14.5)/4.)));
            //initial_wave(index) = std::exp(std::complex<double>(0,M_PI/2.*(xx-14.5)/1.5));
            initial_wave(index) = 1;
        }

        {
            Board2D board;
            board << domain;
            board << CustomStyle("KForm", new KFormStyle2D(-1, 1));
            board << Calculus::DualForm0(calculus, (initial_wave.array().conjugate()*initial_wave.array()).real().sqrt());
            board.saveSVG("propagation_time_wave_initial_coarse.svg");
        }

        Eigen::VectorXcd initial_projections = eigen_vectors.transpose() * initial_wave;

        // low pass
        const Calculus::Scalar lambda_cutoff = 4.5;
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
            board.saveSVG("propagation_time_wave_initial_smoothed.svg");
        }

        trace.progressBar(0,100);
        for (int kk=0; kk<100; kk++)
        {
            const Calculus::Scalar time = kk/20.;
            const Eigen::VectorXcd current_projections = (angular_frequencies * std::complex<double>(0,time)).array().exp() * initial_projections.array();
            const Eigen::VectorXcd current_wave = eigen_vectors * current_projections;

            std::stringstream ss;
            ss << "propagation_time_wave_solution_" << kk << ".svg";

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

    {
        trace.beginBlock("forced oscillations");

        const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(49,49));
        const Calculus calculus(generateDiskSet(domain), true);

        const Calculus::Scalar lambda_0 = 4*23./5;
        trace.info() << "lambda_0 = " << lambda_0 << endl;

        const Calculus::DualIdentity0 dalembert = calculus.dualLaplace() - (2*M_PI/lambda_0)*(2*M_PI/lambda_0) * calculus.identity<0, DUAL>();
        trace.info() << "dalembert = " << dalembert << endl;

        trace.beginBlock("finding eigen pairs");
        typedef Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolverMatrix;
        const EigenSolverMatrix eigen_solver(dalembert.myContainer);

        const Eigen::VectorXd eigen_values = eigen_solver.eigenvalues();
        const Eigen::MatrixXd eigen_vectors = eigen_solver.eigenvectors();
        trace.endBlock();

        const Eigen::MatrixXd concentration_to_wave = eigen_vectors * eigen_values.array().inverse().matrix().asDiagonal() * eigen_vectors.transpose();

        Calculus::DualForm0 concentration(calculus);
        for (int xx=24; xx<26; xx++)
        for (int yy=24; yy<26; yy++)
        {
            const Z2i::Point point(xx,yy);
            const Calculus::Cell cell = calculus.myKSpace.uSpel(point);
            const Calculus::Index index = calculus.getCellIndex(cell);
            concentration.myContainer(index) = 1;
        }

        {
            Board2D board;
            board << domain;
            board << CustomStyle("KForm", new KFormStyle2D(-1, 1));
            board << concentration;
            board.saveSVG("propagation_forced_concentration.svg");
        }

        const Calculus::DualForm0 wave(calculus, concentration_to_wave * concentration.myContainer);

        {
            trace.info() << "saving samples" << endl;
            const Calculus::Properties properties = calculus.getProperties();
            {
                std::ofstream handle("propagation_samples_hv.dat");
                for (int kk=0; kk<50; kk++)
                {
                    const Z2i::Point point_horizontal(kk,24);
                    const Z2i::Point point_vertical(24,kk);
                    const Calculus::Scalar xx = kk/49. - .5;
                    handle << xx << " ";
                    handle << sample_dual_0_form<Calculus>(properties, wave, point_horizontal) << " ";
                    handle << sample_dual_0_form<Calculus>(properties, wave, point_vertical) << endl;
                }
            }

            {
                std::ofstream handle("propagation_samples_diag.dat");
                for (int kk=0; kk<50; kk++)
                {
                    const Z2i::Point point_diag_pos(kk,kk);
                    const Z2i::Point point_diag_neg(kk,49-kk);
                    const Calculus::Scalar xx = sqrt(2) * (kk/49. - .5);
                    handle << xx << " ";
                    handle << sample_dual_0_form<Calculus>(properties, wave, point_diag_pos) << " ";
                    handle << sample_dual_0_form<Calculus>(properties, wave, point_diag_neg) << endl;
                }
            }
        }

        {

            Board2D board;
            board << domain;
            board << wave;
            board.saveSVG("propagation_forced_wave.svg");
        }


        //trace.info() << eigen_values << endl;
        //trace.info() << ( (eigen_vectors.transpose() * eigen_vectors).array().abs() > 1e-5 ) << endl;


        trace.endBlock();
    }
    trace.endBlock();
}

int main(int argc, char* argv[])
{
    propa_2d();
    return 0;
}
