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

    // create discrete exterior calculus from set
    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(generateRingSet(domain));
    trace.info() << calculus << endl;

    Calculus::DualDerivative0 d0 = calculus.derivative<0, DUAL>();
    Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    Calculus::DualHodge1 hodge1 = calculus.dualHodge<1>();
    Calculus::PrimalHodge2 hodge2p = calculus.primalHodge<2>();
    Calculus::DualIdentity0 laplacian = hodge2p *d1p * hodge1 * d0 + 0.1 * calculus.identity<0, DUAL>();
    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d1p = " << d1p << endl;
    trace.info() << "hodge2p = " << hodge2p << endl;
    trace.info() << "laplacian = " << laplacian << endl;

    Calculus::DualForm0 dirac(calculus);
    dirac.myContainer(0) = 1;
    //dirac.myContainer(5) = 1;

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

        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_simplicial_llt.svg");
    }

    { // simplicial ldlt
        trace.beginBlock("simplicial ldlt");

        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLDLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_simplicial_ldlt.svg");
    }

    { // conjugate gradient
        trace.beginBlock("conjugate gradient");

        typedef EigenSparseLinearAlgebraBackend::SolverConjugateGradient LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_conjugate_gradient.svg");
    }

    { // biconjugate gradient stabilized
        trace.beginBlock("biconjugate gradient stabilized (bicgstab)");

        typedef EigenSparseLinearAlgebraBackend::SolverBiCGSTAB LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
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

        typedef EigenSparseLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_sparse_lu.svg");
    }

    { // sparseqr
        trace.beginBlock("sparse qr");

        typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_sparse_qr.svg");
    }

    trace.endBlock();
}

int main(int argc, char* argv[])
{
    solve2d();

    return 0;
}

