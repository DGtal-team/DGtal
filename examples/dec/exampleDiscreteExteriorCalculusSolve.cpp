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

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(44,29));

    // create discrete exterior calculus from set
    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(generateDoubleRingSet(domain));
    trace.info() << calculus << endl;

    // choose linear solver
    typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;

    //! [decomposition_operator_definition]
    const Calculus::DualDerivative0 d0 = calculus.derivative<0, DUAL>();
    const Calculus::DualDerivative1 d1 = calculus.derivative<1, DUAL>();
    const Calculus::PrimalDerivative0 d0p = calculus.derivative<0, PRIMAL>();
    const Calculus::PrimalDerivative1 d1p = calculus.derivative<1, PRIMAL>();
    const Calculus::DualHodge1 h1 = calculus.dualHodge<1>();
    const Calculus::DualHodge2 h2 = calculus.dualHodge<2>();
    const Calculus::PrimalHodge1 h1p = calculus.primalHodge<1>();
    const Calculus::PrimalHodge2 h2p = calculus.primalHodge<2>();
    const LinearOperator<Calculus, 1, DUAL, 0, DUAL> ad1 = h2p * d1p * h1;
    const LinearOperator<Calculus, 2, DUAL, 1, DUAL> ad2 = h1p * d0p * h2;
    //! [decomposition_operator_definition]

    //! [decomposition_input_field_definition]
    Calculus::DualVectorField input_vector_field(calculus);
    for (Calculus::Index ii=0; ii<calculus.kFormLength(0, DUAL); ii++)
    {
        const Z2i::RealPoint cell_center = Z2i::RealPoint(calculus.getSCell(0, DUAL, ii).myCoordinates)/2.;
        input_vector_field.myCoordinates[0](ii) = cos(-.5*cell_center[0]+ .3*cell_center[1]);
        input_vector_field.myCoordinates[1](ii) = cos(.4*cell_center[0]+ .8*cell_center[1]);
    }

    const Calculus::DualForm1 input_one_form = calculus.flat(input_vector_field);
    const Calculus::DualForm0 input_one_form_anti_derivated = ad1 * input_one_form;
    const Calculus::DualForm2 input_one_form_derivated = d1 * input_one_form;
    //! [decomposition_input_field_definition]

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(-1,1);
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        input_one_form.applyToAccum(accum);
        input_one_form_anti_derivated.applyToAccum(accum);
        input_one_form_derivated.applyToAccum(accum);
        accum.display2D(board, colormap);
        input_vector_field.display2D(board);
        board.saveSVG("solve_decomposition_calculus.svg");
    }


    Calculus::DualForm0 solution_curl_free(calculus);
    { // solve curl free problem
        trace.beginBlock("solving curl free component");

        //! [decomposition_curl_free_solve]
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> Solver;
        Solver solver;
        solver.compute(ad1 * d0);
        solution_curl_free = solver.solve(input_one_form_anti_derivated);
        //! [decomposition_curl_free_solve]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << "min=" << solution_curl_free.myContainer.minCoeff() << " max=" << solution_curl_free.myContainer.maxCoeff() << endl;
        trace.endBlock();
    }

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution_curl_free.myContainer.minCoeff(),solution_curl_free.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution_curl_free.applyToAccum(accum);
        accum.display2D(board, colormap);
        calculus.sharp(d0 * solution_curl_free).display2D(board);
        board.saveSVG("solve_decomposition_curl_free.svg");
    }

    Calculus::DualForm2 solution_div_free(calculus);
    { // solve divergence free problem
        trace.beginBlock("solving divergence free component");

        //! [decomposition_div_free_solve]
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 2, DUAL, 2, DUAL> Solver;
        Solver solver;
        solver.compute(d1 * ad2);
        solution_div_free = solver.solve(input_one_form_derivated);
        //! [decomposition_div_free_solve]

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;
        trace.info() << "min=" << solution_div_free.myContainer.minCoeff() << " max=" << solution_div_free.myContainer.maxCoeff() << endl;
        trace.endBlock();
    }

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution_div_free.myContainer.minCoeff(),solution_div_free.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution_div_free.applyToAccum(accum);
        accum.display2D(board, colormap);
        calculus.sharp(ad2 * solution_div_free).display2D(board);
        board.saveSVG("solve_decomposition_div_free.svg");
    }

    //! [decomposition_solution]
    const Calculus::DualForm1 solution_harmonic = input_one_form - d0*solution_curl_free - ad2*solution_div_free;
    //! [decomposition_solution]

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(solution_harmonic.myContainer.minCoeff(),solution_harmonic.myContainer.maxCoeff());
        Board2D board;
        board << domain;
        Calculus::Accum accum(calculus);
        solution_harmonic.applyToAccum(accum);
        accum.display2D(board, colormap);
        calculus.sharp(solution_harmonic).display2D(board);
        board.saveSVG("solve_decomposition_harmonic.svg");
    }

    trace.endBlock();
}
int main(int argc, char* argv[])
{
    solve2d_laplacian();
    solve2d_decomposition();

    return 0;
}

