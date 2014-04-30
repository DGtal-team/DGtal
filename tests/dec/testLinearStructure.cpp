/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testLinearStructure.cpp
 * @ingroup Tests
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 * This file is part of the DGtal library
 */

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

using namespace DGtal;
using namespace Z2i;
using std::endl;

int
main(int argc, char* argv[])
{
    trace.beginBlock("creating dec problem with neumann border condition");

    //! [neumann-creation]
    const Domain domain(Point(-1,-1), Point(10,10));

    typedef DiscreteExteriorCalculus<Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(domain);

    for (int kk=20; kk>0; kk--)
        calculus.insertSCell(calculus.kspace.sCell(Point(0,kk), kk%2 == 1 ? Calculus::KSpace::NEG : Calculus::KSpace::POS));
    for (int kk=0; kk<10; kk++)
        calculus.insertSCell(calculus.kspace.sCell(Point(kk,0)));
    for (int kk=0; kk<10; kk++)
        calculus.insertSCell(calculus.kspace.sCell(Point(10,kk)));
    calculus.insertSCell(calculus.kspace.sCell(Point(10,10)));
    calculus.insertSCell(calculus.kspace.sCell(Point(9,10), Calculus::KSpace::NEG));
    for (int kk=10; kk<20; kk++)
        calculus.insertSCell(calculus.kspace.sCell(Point(8,kk)));
    calculus.insertSCell(calculus.kspace.sCell(Point(8,20)));
    calculus.insertSCell(calculus.kspace.sCell(Point(9,20)));
    calculus.insertSCell(calculus.kspace.sCell(Point(10,20)));
    calculus.insertSCell(calculus.kspace.sCell(Point(11,20)));
    for (int kk=20; kk>0; kk--)
        calculus.insertSCell(calculus.kspace.sCell(Point(12,kk), kk%2 == 1 ? Calculus::KSpace::NEG : Calculus::KSpace::POS));
    calculus.insertSCell(calculus.kspace.sCell(Point(12,0)));
    //! [neumann-creation]

    trace.info() << calculus << endl;

    //! [input-dirac]
    const Calculus::Index dirac_position = 17;
    Calculus::PrimalForm0 dirac(calculus);
    dirac.myContainer(dirac_position) = 1;
    //! [input-dirac]

    trace.info() << "dirac_position = " << dirac_position << endl;

    {
        Calculus::Accum accum(calculus);
        dirac.applyToAccum(accum);

        Board2D board;
        board << domain;
        board << accum;
        board.saveSVG("linear_structure_neumann_dirac.svg");
    }

    trace.endBlock();

    {
        trace.beginBlock("solving problem with neumann border condition using sparse qr solver");

        //! [neumann-laplacian-definition]
        Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
        Calculus::DualDerivative1 d1p = calculus.derivative<1, DUAL>();
        Calculus::PrimalHodge1 hodge1 = calculus.primalHodge<1>();
        Calculus::DualHodge2 hodge2p = calculus.dualHodge<2>();
        Calculus::PrimalIdentity0 laplacian = hodge2p *d1p * hodge1 * d0;
        //! [neumann-laplacian-definition]
        trace.info() << "d0 = " << d0 << endl;
        trace.info() << "hodge1 = " << hodge1 << endl;
        trace.info() << "d1p = " << d1p << endl;
        trace.info() << "hodge2p = " << hodge2p << endl;
        trace.info() << "laplacian = " << laplacian << endl;
        trace.info() << laplacian.myContainer << endl;

        //! [neumann-solve]
        typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::PrimalForm0 solved_solution = solver.solve(dirac);
        //! [neumann-solve]
        solved_solution.myContainer /= solved_solution.myContainer.maxCoeff();

        Calculus::PrimalForm0 analytic_solution(calculus);
        for (Calculus::Index kk=0; kk<calculus.kFormLength(0, PRIMAL); kk++)
        {
            Calculus::Scalar alpha = 1. * (kk)/dirac_position * (kk+1.)/(dirac_position+1.);
            if (kk>dirac_position)
            {
                alpha = 1. * (calculus.kFormLength(0, PRIMAL)-kk)/dirac_position * (calculus.kFormLength(0, PRIMAL)-kk-1.)/(dirac_position+1);
                alpha -= 1. * (calculus.kFormLength(0, PRIMAL)-dirac_position)/dirac_position * (calculus.kFormLength(0, PRIMAL)-dirac_position-1.)/(dirac_position+1);
                alpha += 1;
            }
            analytic_solution.myContainer(kk) = alpha;
        }

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;

        for (Calculus::Index kk=0; kk<calculus.kFormLength(0, PRIMAL); kk++)
        {
            trace.info() << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
            FATAL_ERROR(abs(solved_solution.myContainer(kk) - analytic_solution.myContainer(kk)) < 1e-5);
        }

        {
            Calculus::Accum accum(calculus);
            solved_solution.applyToAccum(accum);

            Board2D board;
            board << domain;
            board << CustomStyle("AllSCellMap", new AllSCellMapStyle2D(solved_solution.myContainer.minCoeff(), solved_solution.myContainer.maxCoeff()));
            board << accum;
            board.saveSVG("linear_structure_neumann_solution.svg");
        }

        {
            Calculus::PrimalForm1 solved_solution_gradient = d0 * solved_solution;
            Calculus::Accum accum(calculus);
            solved_solution_gradient.applyToAccum(accum);

            Board2D board;
            board << domain;
            board << CustomStyle("AllSCellMap", new AllSCellMapStyle2D(solved_solution_gradient.myContainer.minCoeff(), solved_solution_gradient.myContainer.maxCoeff()));
            board << accum;
            board << CustomStyle("VectorField", new VectorFieldStyle2D(1));
            board << calculus.sharp(solved_solution_gradient);
            board.saveSVG("linear_structure_neumann_solution_gradient.svg");
        }

        trace.endBlock();
    }

    trace.beginBlock("creating dec problem with dirichlet border condition");

    //! [dirichlet-creation]
    calculus.insertSCell(calculus.kspace.sCell(Point(13,0)));
    calculus.insertSCell(calculus.kspace.sCell(Point(1,20), Calculus::KSpace::NEG));
    //! [dirichlet-creation]

    {
        Calculus::Accum accum(calculus);
        dirac.applyToAccum(accum);

        Board2D board;
        board << domain;
        board << accum;
        board.saveSVG("linear_structure_dirichlet_dirac.svg");
    }

    trace.endBlock();

    {
        trace.beginBlock("solving problem with dirichlet border condition using sparse qr solver");

        //! [dirichlet-laplacian-definition]
        Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
        Calculus::DualDerivative1 d1p = calculus.derivative<1, DUAL>();
        Calculus::PrimalHodge1 hodge1 = calculus.primalHodge<1>();
        Calculus::DualHodge2 hodge2p = calculus.dualHodge<2>();
        Calculus::PrimalIdentity0 laplacian = hodge2p *d1p * hodge1 * d0;
        //! [dirichlet-laplacian-definition]
        trace.info() << "d0 = " << d0 << endl;
        trace.info() << "hodge1 = " << hodge1 << endl;
        trace.info() << "d1p = " << d1p << endl;
        trace.info() << "hodge2p = " << hodge2p << endl;
        trace.info() << "laplacian = " << laplacian << endl;
        trace.info() << laplacian.myContainer << endl;

        //! [dirichlet-solve]
        typedef EigenSparseLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> Solver;

        Solver solver;
        solver.compute(laplacian);
        Calculus::PrimalForm0 solved_solution = solver.solve(dirac);
        //! [dirichlet-solve]
        solved_solution.myContainer /= solved_solution.myContainer.maxCoeff();

        Calculus::PrimalForm0 analytic_solution(calculus);
        for (Calculus::Index kk=0; kk<calculus.kFormLength(0, PRIMAL); kk++)
        {
            Calculus::Scalar alpha = (kk+1.)/(dirac_position+1.);
            if (kk>dirac_position)
            {
                alpha = 1. - (kk-dirac_position)/(1.*calculus.kFormLength(0, PRIMAL)-dirac_position);
            }
            analytic_solution.myContainer(kk) = alpha;
        }

        trace.info() << solver.isValid() << " " << solver.solver.info() << endl;

        for (Calculus::Index kk=0; kk<calculus.kFormLength(0, PRIMAL); kk++)
        {
            trace.info() << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
            FATAL_ERROR(abs(solved_solution.myContainer(kk) - analytic_solution.myContainer(kk)) < 1e-5);
        }

        {
            Calculus::Accum accum(calculus);
            solved_solution.applyToAccum(accum);

            Board2D board;
            board << domain;
            board << CustomStyle("AllSCellMap", new AllSCellMapStyle2D(solved_solution.myContainer.minCoeff(), solved_solution.myContainer.maxCoeff()));
            board << accum;
            board.saveSVG("linear_structure_dirichlet_solution.svg");
        }

        {
            Calculus::PrimalForm1 solved_solution_gradient = d0 * solved_solution;
            Calculus::Accum accum(calculus);
            solved_solution_gradient.applyToAccum(accum);

            Board2D board;
            board << domain;
            board << CustomStyle("AllSCellMap", new AllSCellMapStyle2D(solved_solution_gradient.myContainer.minCoeff(), solved_solution_gradient.myContainer.maxCoeff()));
            board << accum;
            board << calculus.sharp(solved_solution_gradient);
            board.saveSVG("linear_structure_dirichlet_solution_gradient.svg");
        }

        trace.endBlock();
    }

    return 0;
}
