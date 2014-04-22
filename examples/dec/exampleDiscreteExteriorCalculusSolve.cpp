#include <string>
using namespace std;

#include "common.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/readers/GenericReader.h"
using namespace DGtal;

void solve2d_laplacian()
{
    trace.beginBlock("2d discrete exterior calculus solve laplacian");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));

    // create discrete exterior calculus from set
    //! [calculus_creation]
    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(generateRingSet(domain));
    //! [calculus_creation]
    trace.info() << calculus << endl;

    //! [laplacian_definition]
    Calculus::DualDerivative0 d0 = calculus.derivative<0, DUAL>();
    Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    Calculus::DualHodge1 hodge1 = calculus.dualHodge<1>();
    Calculus::PrimalHodge2 hodge2p = calculus.primalHodge<2>();
    Calculus::DualIdentity0 laplacian = hodge2p *d1p * hodge1 * d0 + 0.1 * calculus.identity<0, DUAL>();
    //! [laplacian_definition]
    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d1p = " << d1p << endl;
    trace.info() << "hodge2p = " << hodge2p << endl;
    trace.info() << "laplacian = " << laplacian << endl;

    //! [dirac_definition]
    Calculus::DualForm0 dirac(calculus);
    dirac.myContainer(calculus.getIndex(calculus.kspace.sSpel(Z2i::Point(2,5)))) = 1;
    //! [dirac_definition]
    //dirac.myContainer(5) = 1;

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(0,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        dirac.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_calculus.svg");
    }

    { // simplicial llt
        trace.beginBlock("simplicial llt");

        //! [solve_llt]
        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_llt]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_simplicial_llt.svg");
    }

    { // simplicial ldlt
        trace.beginBlock("simplicial ldlt");

        //! [solve_ldlt]
        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLDLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_ldlt]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_simplicial_ldlt.svg");
    }

    { // conjugate gradient
        trace.beginBlock("conjugate gradient");

        //! [solve_conjugate_gradient]
        typedef EigenSparseLinearAlgebraBackend::SolverConjugateGradient LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_conjugate_gradient]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_conjugate_gradient.svg");
    }

    { // biconjugate gradient stabilized
        trace.beginBlock("biconjugate gradient stabilized (bicgstab)");

        //! [solve_biconjugate_gradient]
        typedef EigenSparseLinearAlgebraBackend::SolverBiCGSTAB LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_biconjugate_gradient]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_bicgstab.svg");
    }

    { // sparselu
        trace.beginBlock("sparse lu");

        //! [solve_sparse_lu]
        typedef EigenSparseLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_sparse_lu]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_sparse_lu.svg");
    }

    { // sparseqr
        trace.beginBlock("sparse qr");

        //! [solve_sparse_qr]
        typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_sparse_qr]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_laplacian_sparse_qr.svg");
    }

    trace.endBlock();
}

void solve2d_decomposition()
{
    trace.beginBlock("2d discrete exterior calculus solve helmoltz decomposition");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));

    // create discrete exterior calculus from set
    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(generateRingSet(domain));
    trace.info() << calculus << endl;

    //! [decomposition_operator_definition]
    const Calculus::DualDerivative0 derivative = calculus.derivative<0, DUAL>();
    const Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    const Calculus::DualHodge1 hodge1 = calculus.dualHodge<1>();
    const Calculus::PrimalHodge2 hodge2p = calculus.primalHodge<2>();
    const LinearOperator<Calculus, 1, DUAL, 0, DUAL> anti_derivative = hodge2p *d1p * hodge1;
    //! [decomposition_operator_definition]

    //! [sink_definition]
    Calculus::DualForm1 input_one_form(calculus);
    input_one_form.myContainer(calculus.getIndex(calculus.kspace.sSpel(Z2i::Point(2,5)))) = 1;
    input_one_form.myContainer(calculus.getIndex(calculus.kspace.sSpel(Z2i::Point(6,7)))) = -1;
    //! [sink_definition]

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(-1,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        dirac.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_calculus.svg");
    }

    /*
    { // simplicial llt
        trace.beginBlock("simplicial llt");

        //! [solve_llt]
        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 1, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(divergence);
        Calculus::DualForm1 solution = solver.solve(dirac);
        //! [solve_llt]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_simplicial_llt.svg");
    }

    { // simplicial ldlt
        trace.beginBlock("simplicial ldlt");

        typedef EigenSparseLinearAlgebraBackend::SolverSimplicialLDLT LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 1, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(divergence);
        Calculus::DualForm1 solution = solver.solve(dirac);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_simplicial_ldlt.svg");
    }

    { // conjugate gradient
        trace.beginBlock("conjugate gradient");

        //! [solve_conjugate_gradient]
        typedef EigenSparseLinearAlgebraBackend::SolverConjugateGradient LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_conjugate_gradient]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_conjugate_gradient.svg");
    }

    { // biconjugate gradient stabilized
        trace.beginBlock("biconjugate gradient stabilized (bicgstab)");

        //! [solve_biconjugate_gradient]
        typedef EigenSparseLinearAlgebraBackend::SolverBiCGSTAB LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_biconjugate_gradient]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_bicgstab.svg");
    }

    { // sparselu
        trace.beginBlock("sparse lu");

        //! [solve_sparse_lu]
        typedef EigenSparseLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::DualForm0 solution = solver.solve(dirac);
        //! [solve_sparse_lu]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_sparse_lu.svg");
    }
    */

    /*
    { // sparseqr
        trace.beginBlock("sparse qr");

        typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 1, DUAL, 0, DUAL> Solver;

        Solver solver;
        solver.compute(divergence);
        Calculus::DualForm1 solution = solver.solve(dirac);
        Calculus::DualVectorField solution_field = calculus.sharp(solution);

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << solution << endl;
        trace.endBlock();

        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution.myContainer.minCoeff(),solution.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution.applyToAccum(accum);
        accum.display2D(board, colormap);
        board.saveSVG("solve_decomposition_sparse_qr.svg");
    }
    */

    trace.endBlock();
}
int main(int argc, char* argv[])
{
    solve2d_laplacian();
    solve2d_decomposition();

    return 0;
}

