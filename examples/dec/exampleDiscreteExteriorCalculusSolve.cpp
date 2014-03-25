#include <string>
using namespace std;

#include "common.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/readers/GenericReader.h"
using namespace DGtal;

void solve2d()
{
    trace.beginBlock("2d discrete exterior calculus solve");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));

    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus = generateRing(domain);

    Calculus::DualDerivative0 d0 = calculus.derivative<0, DUAL>();
    Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    Calculus::DualHodge1 hodge1 = calculus.dualHodge<1>();
    Calculus::PrimalHodge2 hodge2p = calculus.primalHodge<2>();
    LinearOperator<Calculus, 0, DUAL, 0, DUAL> laplacian = hodge2p *d1p * hodge1 * d0 + 0.1 * calculus.identity<0, DUAL>();
    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d1p = " << d1p << endl;
    trace.info() << "hodge2p = " << hodge2p << endl;
    trace.info() << "laplacian = " << laplacian << endl;

    Calculus::DualForm0 dirac(calculus);
    dirac.container(0) = 1;

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        dirac.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_calculus.svg");
    }

    { // simplicial llt
        trace.beginBlock("simplicial llt");

        Eigen::SimplicialLLT<Calculus::Matrix> solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_simplicialllt.svg");
    }

    { // simplicial ldlt
        trace.beginBlock("simplicial ldlt");

        Eigen::SimplicialLLT<Calculus::Matrix> solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_simplicialldlt.svg");
    }

    { // conjugate gradient
        trace.beginBlock("conjugate gradient");

        Eigen::ConjugateGradient<Calculus::Matrix> solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_conjugategradient.svg");
    }

    { // biconjugate gradient stabilized
        trace.beginBlock("biconjugate gradient stabilized");

        Eigen::BiCGSTAB<Calculus::Matrix> solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_bicgstab.svg");
    }

    { // sparselu
        trace.beginBlock("sparse lu");

        Eigen::SparseLU<Calculus::Matrix> solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_sparselu.svg");
    }

    { // sparseqr
        trace.beginBlock("sparse qr");

        Eigen::SparseQR<Calculus::Matrix, Eigen::COLAMDOrdering<Calculus::Matrix::Index> > solver;
        solver.compute(laplacian.container);
        Calculus::DualForm0 solution = Calculus::DualForm0(calculus, solver.solve(dirac.container));

        trace.info() << solver.info() << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_sparseqr.svg");
    }

    trace.endBlock();
}

int main(int argc, char* argv[])
{
    solve2d();

    return 0;
}

