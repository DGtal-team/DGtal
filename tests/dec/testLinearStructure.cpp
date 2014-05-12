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

void test_linear_structure()
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
    dirac.myContainer(dirac_position) = -1;
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

}

void test_laplace_operator()
{
    trace.beginBlock("testing operators");

    const Domain input_domain(Point(0,0), Point(3,4));
    DigitalSet input_set(input_domain);
    input_set.insertNew(Point(1,2));
    input_set.insertNew(Point(1,1));

    typedef DiscreteExteriorCalculus<Domain, EigenSparseLinearAlgebraBackend> Calculus;
    Calculus calculus(input_set);

    {
        Calculus::Accum accum(calculus);

        Board2D board;
        board << input_domain;
        board << accum;
        board.saveSVG("laplace_structure.svg");
    }

    const Calculus::Properties properties = calculus.getProperties();
    for (Calculus::Properties::ConstIterator iter_property=properties.begin(), iter_property_end=properties.end(); iter_property!=iter_property_end; iter_property++)
        trace.info() << iter_property->first << " " << iter_property->second.size_ratio << " " << iter_property->second.index << endl;
    trace.info() << endl;

    const Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
    trace.info() << "d0" << endl << d0.myContainer << endl;
    const Calculus::PrimalDerivative1 d1 = calculus.derivative<1, PRIMAL>();
    trace.info() << "d1" << endl << d1.myContainer << endl;
    const Calculus::DualDerivative0 d0p = calculus.derivative<0, DUAL>();
    trace.info() << "d0p" << endl << d0p.myContainer << endl;
    const Calculus::DualDerivative1 d1p = calculus.derivative<1, DUAL>();
    trace.info() << "d1p" << endl << d1p.myContainer << endl;
    const Calculus::PrimalHodge0 h0 = calculus.primalHodge<0>();
    const Calculus::DualHodge2 h2p = calculus.dualHodge<2>();
    trace.info() << "h0" << endl << h0.myContainer << endl;
    trace.info() << "h2p" << endl << h2p.myContainer << endl;
    const Calculus::PrimalHodge1 h1 = calculus.primalHodge<1>();
    const Calculus::DualHodge1 h1p = calculus.dualHodge<1>();
    trace.info() << "h1" << endl << h1.myContainer << endl;
    trace.info() << "h1p" << endl << h1p.myContainer << endl;
    const Calculus::PrimalHodge2 h2 = calculus.primalHodge<2>();
    const Calculus::DualHodge0 h0p = calculus.dualHodge<0>();
    trace.info() << "h2" << endl << h2.myContainer << endl;
    trace.info() << "h0p" << endl << h0p.myContainer << endl;
    //const LinearOperator<Calculus, 1, PRIMAL, 0, PRIMAL> ad1 = h2p * d1p * h1;
    //const LinearOperator<Calculus, 2, PRIMAL, 1, PRIMAL> ad2 = h1p * d0p * h2;

    trace.endBlock();
}


int
main(int argc, char* argv[])
{
    test_linear_structure();
    test_laplace_operator();
    return 0;
}

