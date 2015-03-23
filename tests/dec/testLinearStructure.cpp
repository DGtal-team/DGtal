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

    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;
    Calculus calculus;
    calculus.initKSpace(domain);

    for (int kk=20; kk>0; kk--)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(0,kk), kk%2 != 0 ? Calculus::KSpace::NEG : Calculus::KSpace::POS) );
    for (int kk=0; kk<10; kk++)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(kk,0)) );
    for (int kk=0; kk<10; kk++)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(10,kk)) );
    calculus.insertSCell( calculus.myKSpace.sCell(Point(10,10)) );
    calculus.insertSCell( calculus.myKSpace.sCell(Point(9,10), Calculus::KSpace::NEG) );
    for (int kk=10; kk<20; kk++)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(8,kk)) );
    for (int kk=8; kk<12; kk++)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(kk,20)) );
    for (int kk=20; kk>0; kk--)
        calculus.insertSCell( calculus.myKSpace.sCell(Point(12,kk), kk%2 != 0 ? Calculus::KSpace::NEG : Calculus::KSpace::POS) );
    calculus.insertSCell( calculus.myKSpace.sCell(Point(12,0)) );
    //! [neumann-creation]

    trace.info() << calculus << endl;

    //! [input-dirac]
    const Calculus::Index dirac_position = 17;
    Calculus::PrimalForm0 dirac(calculus);
    dirac.myContainer(dirac_position) = -1;
    //! [input-dirac]

    trace.info() << "dirac_position = " << dirac_position << endl;

    {
        Board2D board;
        board << domain;
        board << calculus;
        board << dirac;
        board.saveSVG("linear_structure_neumann_dirac.svg");
    }

    trace.endBlock();

    {
        trace.beginBlock("solving problem with neumann border condition using sparse qr solver");

        //! [neumann-laplace-definition]
        const Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
        const Calculus::PrimalIdentity0 laplace = calculus.primalLaplace();
        trace.info() << "d0 = " << d0 << endl;
        trace.info() << "laplace = " << laplace << endl;
        trace.info() << Eigen::MatrixXd(laplace.myContainer) << endl;
        //! [neumann-laplace-definition]

        //! [neumann-solve]
        typedef EigenLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> Solver;

        Solver solver;
        solver.compute(laplace);
        Calculus::PrimalForm0 solved_solution = solver.solve(dirac);
        //! [neumann-solve]
        solved_solution.myContainer.array() -= solved_solution.myContainer(0);
        solved_solution.myContainer.array() /= solved_solution.myContainer.maxCoeff();

        Calculus::PrimalForm0 analytic_solution(calculus);
        {
            const Calculus::Index length = analytic_solution.length();
            for (Calculus::Index kk=0; kk<length; kk++)
            {
                Calculus::Scalar alpha = 1. * (kk)/dirac_position * (kk+1.)/(dirac_position+1.);
                if (kk>dirac_position)
                {
                    alpha = 1. * (length-kk)/dirac_position * (length-kk-1.)/(dirac_position+1);
                    alpha -= 1. * (length-dirac_position)/dirac_position * (length-dirac_position-1.)/(dirac_position+1);
                    alpha += 1;
                }
                analytic_solution.myContainer(kk) = alpha;
            }
        }

        trace.info() << solver.isValid() << " " << solver.myLinearAlgebraSolver.info() << endl;

        {
            std::ofstream handle("linear_structure_neumann.dat");
            for (Calculus::Index kk=0; kk<analytic_solution.length(); kk++)
                handle << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
        }

        for (Calculus::Index kk=0; kk<analytic_solution.length(); kk++)
        {
            trace.info() << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
            FATAL_ERROR( abs(solved_solution.myContainer(kk) - analytic_solution.myContainer(kk)) < 1e-5 );
        }

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << solved_solution;
            board.saveSVG("linear_structure_neumann_solution.svg");
        }

        {
            Calculus::PrimalForm1 solved_solution_gradient = d0 * solved_solution;
            Board2D board;
            board << domain;
            board << calculus;
            board << solved_solution_gradient;
            board << CustomStyle("VectorField", new VectorFieldStyle2D(1));
            board << calculus.sharp(solved_solution_gradient);
            board.saveSVG("linear_structure_neumann_solution_gradient.svg");
        }

        trace.endBlock();
    }

    trace.beginBlock("creating dec problem with dirichlet border condition");

    //! [dirichlet-creation]
    calculus.insertSCell(calculus.myKSpace.sCell(Point(13,0)));
    calculus.insertSCell(calculus.myKSpace.sCell(Point(1,20), Calculus::KSpace::NEG));
    //! [dirichlet-creation]

    {
        Board2D board;
        board << domain;
        board << calculus;
        board << dirac;
        board.saveSVG("linear_structure_dirichlet_dirac.svg");
    }

    trace.endBlock();

    {
        trace.beginBlock("solving problem with dirichlet border condition using sparse qr solver");

        //! [dirichlet-laplace-definition]
        const Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
        const Calculus::PrimalIdentity0 laplace = calculus.primalLaplace();
        trace.info() << "d0 = " << d0 << endl;
        trace.info() << "laplace = " << laplace << endl;
        trace.info() << Eigen::MatrixXd(laplace.myContainer) << endl;
        //! [dirichlet-laplace-definition]

        //! [dirichlet-solve]
        typedef EigenLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
        typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> Solver;

        Solver solver;
        solver.compute(laplace);
        Calculus::PrimalForm0 solved_solution = solver.solve(dirac);
        //! [dirichlet-solve]
        solved_solution.myContainer.array() /= solved_solution.myContainer.maxCoeff();

        Calculus::PrimalForm0 analytic_solution(calculus);
        {
            const Calculus::Index length = analytic_solution.length();
            for (Calculus::Index kk=0; kk<length; kk++)
            {
                Calculus::Scalar alpha = (kk+1.)/(dirac_position+1.);
                if (kk>dirac_position)
                {
                    alpha = 1. - (kk-dirac_position)/(1.*length-dirac_position);
                }
                analytic_solution.myContainer(kk) = alpha;
            }
        }

        trace.info() << solver.isValid() << " " << solver.myLinearAlgebraSolver.info() << endl;

        {
            std::ofstream handle("linear_structure_dirichlet.dat");
            for (Calculus::Index kk=0; kk<analytic_solution.length(); kk++)
                handle << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
        }

        for (Calculus::Index kk=0; kk<analytic_solution.length(); kk++)
        {
            trace.info() << solved_solution.myContainer(kk) << " " << analytic_solution.myContainer(kk) << endl;
            FATAL_ERROR( abs(solved_solution.myContainer(kk) - analytic_solution.myContainer(kk)) < 1e-5 );
        }

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << solved_solution;
            board.saveSVG("linear_structure_dirichlet_solution.svg");
        }

        {
            Calculus::PrimalForm1 solved_solution_gradient = d0 * solved_solution;

            Board2D board;
            board << domain;
            board << calculus;
            board << solved_solution_gradient;
            board << calculus.sharp(solved_solution_gradient);
            board.saveSVG("linear_structure_dirichlet_solution_gradient.svg");
        }

        trace.endBlock();
    }

}

template <typename Operator>
void display_operator_info(const std::string& name, const Operator& op)
{
    trace.info() << name << endl << op << endl << Eigen::MatrixXd(op.myContainer) << endl;
}

void test_linear_ring()
{
    trace.beginBlock("linear ring");

    const Domain domain(Point(-5,-5), Point(5,5));

    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;
    Calculus calculus;
    calculus.initKSpace(domain);

    for (int kk=-8; kk<10; kk++) calculus.insertSCell( calculus.myKSpace.sCell(Point(-8,kk), kk%2 == 0 ? Calculus::KSpace::POS : Calculus::KSpace::NEG) );
    for (int kk=-8; kk<10; kk++) calculus.insertSCell( calculus.myKSpace.sCell(Point(kk,10), kk%2 == 0 ? Calculus::KSpace::POS : Calculus::KSpace::NEG) );
    for (int kk=10; kk>-8; kk--) calculus.insertSCell( calculus.myKSpace.sCell(Point(10,kk)) );
    for (int kk=10; kk>-8; kk--) calculus.insertSCell( calculus.myKSpace.sCell(Point(kk,-8)) );

    {
        trace.info() << calculus << endl;
        Board2D board;
        board << domain;
        board << calculus;
        board.saveSVG("ring_structure.svg");
    }

    const Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
    display_operator_info("d0", d0);

    const Calculus::PrimalHodge0 h0 = calculus.primalHodge<0>();
    display_operator_info("h0", h0);

    const Calculus::DualDerivative1 d1p = calculus.derivative<1, DUAL>();
    display_operator_info("d1p", d1p);

    const Calculus::PrimalHodge1 h1 = calculus.primalHodge<1>();
    display_operator_info("h1", h1);

    const Calculus::PrimalIdentity0 laplace = calculus.primalLaplace();
    display_operator_info("laplace", laplace);

    const int laplace_size = calculus.kFormLength(0, PRIMAL);
    Eigen::MatrixXd laplace_th = Eigen::MatrixXd::Zero(laplace_size, laplace_size);
    for (int ii=0; ii<laplace_size; ii++)
    for (int jj=0; jj<laplace_size; jj++)
    {
        int delta = ii>jj ? ii-jj : jj-ii;
        if (delta == 0) { laplace_th(ii,jj) = -2; continue; }
        if (delta == 1 || delta == laplace_size-1) { laplace_th(ii,jj) = 1; continue; }
    }

    trace.endBlock();

    FATAL_ERROR( Eigen::MatrixXd(laplace.myContainer) == laplace_th );
}


void test_manual_operators()
{
    trace.beginBlock("testing operators");

    const Domain domain(Point(0,0), Point(5,4));

    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;

    Calculus primal_calculus;
    primal_calculus.initKSpace(domain);

    { // filling primal calculus
        // 0-cells
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(2,2)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(4,2)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(2,4)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(4,4)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(2,6)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(4,6)) );

        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(1,2)) ); // insert cell

        // 1-cells
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,2), Calculus::KSpace::NEG) ); // insert negative cell
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,2), Calculus::KSpace::POS) ); // then reinserting negative cell in structure
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(2,3), Calculus::KSpace::NEG) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(4,3), Calculus::KSpace::POS) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,4), Calculus::KSpace::NEG) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(2,5), Calculus::KSpace::POS) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(4,5), Calculus::KSpace::NEG) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,6), Calculus::KSpace::POS) );

        primal_calculus.eraseCell( primal_calculus.myKSpace.uCell(Point(1,2)) ); // then remove it

        // 2-cells
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,3)) );
        primal_calculus.insertSCell( primal_calculus.myKSpace.sCell(Point(3,5)) );

        trace.info() << primal_calculus << endl;
    }

    Calculus dual_calculus;
    dual_calculus.initKSpace(domain);

    { // filling dual calculus
        // 2-cells
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(7,3)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(9,3)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(7,5)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(9,5)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(7,7)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(9,7)) );

        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(6,3)) ); // insert cell

        // 1-cells
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,3), Calculus::KSpace::NEG) ); // insert negative cell
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,3), Calculus::KSpace::POS) ); // then reinserting negative cell in structure
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(7,4), Calculus::KSpace::POS) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(9,4), Calculus::KSpace::NEG) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,5), Calculus::KSpace::NEG) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(7,6), Calculus::KSpace::NEG) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(9,6), Calculus::KSpace::POS) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,7), Calculus::KSpace::POS) );

        dual_calculus.eraseCell( dual_calculus.myKSpace.uCell(Point(6,3)) ); // then remove it

        // 0-cells
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,4)) );
        dual_calculus.insertSCell( dual_calculus.myKSpace.sCell(Point(8,6)) );

        trace.info() << dual_calculus << endl;
    }


    {
        Board2D board;
        board << domain;
        board << primal_calculus;
        board << dual_calculus;
        board.saveSVG("operators_structure.svg");
    }

    const Calculus::Properties properties = primal_calculus.getProperties();
    for (Calculus::ConstIterator iter_property=properties.begin(), iter_property_end=properties.end(); iter_property!=iter_property_end; iter_property++)
    {
        const Calculus::Cell cell = iter_property->first;
        const Calculus::Property property = iter_property->second;
        const Dimension dim = primal_calculus.myKSpace.uDim(cell);
        const Calculus::SCell signed_cell = primal_calculus.myKSpace.signs(cell, property.flipped ? Calculus::KSpace::NEG : Calculus::KSpace::POS);

        ASSERT( signed_cell == primal_calculus.getSCell(dim, PRIMAL, property.index) );

        trace.info() << cell
            << " " << dim
            << " " << signed_cell
            << " " << property.size_ratio
            << " " << property.index
            << " " << (property.flipped ? "flipped" : "normal")
            << endl;
    }

    trace.beginBlock("base operators");

    const Calculus::PrimalDerivative0 primal_d0 = primal_calculus.derivative<0, PRIMAL>();
    const Calculus::DualDerivative0 dual_d0p = dual_calculus.derivative<0, DUAL>();
    {
        display_operator_info("primal d0", primal_d0);
        display_operator_info("dual d0p", dual_d0p);

        Eigen::MatrixXd d0_th(7, 6);
        d0_th <<
             1, -1,  0,  0,  0,  0,
            -1,  0,  1,  0,  0,  0,
             0,  1,  0, -1,  0,  0,
             0,  0, -1,  1,  0,  0,
             0,  0,  1,  0, -1,  0,
             0,  0,  0, -1,  0,  1,
             0,  0,  0,  0,  1, -1;

        FATAL_ERROR( Eigen::MatrixXd(primal_d0.myContainer) == d0_th );
        FATAL_ERROR( Eigen::MatrixXd(dual_d0p.myContainer) == -d0_th );
    }

    const Calculus::PrimalDerivative1 primal_d1 = primal_calculus.derivative<1, PRIMAL>();
    const Calculus::DualDerivative1 dual_d1p = dual_calculus.derivative<1, DUAL>();
    {
        display_operator_info("primal d1", primal_d1);
        display_operator_info("dual d1p", dual_d1p);

        Eigen::MatrixXd d1_th(2, 7);
        d1_th <<
            -1, -1, -1, -1,  0,  0,  0,
             0,  0,  0,  1,  1,  1,  1;

        FATAL_ERROR( Eigen::MatrixXd(primal_d1.myContainer) == d1_th );
        FATAL_ERROR( Eigen::MatrixXd(dual_d1p.myContainer) == d1_th );
    }

    {
        display_operator_info("primal d1*d0", primal_d1*primal_d0);
        display_operator_info("dual d1p*d0p", dual_d1p*dual_d0p);

        FATAL_ERROR( Eigen::MatrixXd((primal_d1*primal_d0).myContainer) == Eigen::MatrixXd::Zero(2,6) );
        FATAL_ERROR( Eigen::MatrixXd((dual_d1p*dual_d0p).myContainer) == Eigen::MatrixXd::Zero(2,6) );
    }

    const Calculus::PrimalHodge0 primal_h0 = primal_calculus.primalHodge<0>();
    const Calculus::DualHodge0 dual_h0p = dual_calculus.dualHodge<0>();
    const Calculus::DualHodge2 primal_h2p = primal_calculus.dualHodge<2>();
    const Calculus::PrimalHodge2 dual_h2 = dual_calculus.primalHodge<2>();
    {
        display_operator_info("primal h0", primal_h0);
        display_operator_info("dual h0p", dual_h0p);
        display_operator_info("primal h2p", primal_h2p);
        display_operator_info("dual h2", dual_h2);

        FATAL_ERROR( Eigen::MatrixXd(primal_h0.myContainer) == Eigen::MatrixXd::Identity(6,6) );
        FATAL_ERROR( Eigen::MatrixXd(dual_h0p.myContainer) == Eigen::MatrixXd::Identity(6,6) );
        FATAL_ERROR( Eigen::MatrixXd(primal_h2p.myContainer) == Eigen::MatrixXd::Identity(6,6) );
        FATAL_ERROR( Eigen::MatrixXd(dual_h2.myContainer) == Eigen::MatrixXd::Identity(6,6) );
    }

    const Calculus::PrimalHodge2 primal_h2 = primal_calculus.primalHodge<2>();
    const Calculus::DualHodge2 dual_h2p = dual_calculus.dualHodge<2>();
    const Calculus::DualHodge0 primal_h0p = primal_calculus.dualHodge<0>();
    const Calculus::PrimalHodge0 dual_h0 = dual_calculus.primalHodge<0>();
    {
        display_operator_info("primal h2", primal_h2);
        display_operator_info("dual h2p", dual_h2p);
        display_operator_info("primal h0p", primal_h0p);
        display_operator_info("dual h0", dual_h0);

        FATAL_ERROR( Eigen::MatrixXd(primal_h2.myContainer) == Eigen::MatrixXd::Identity(2,2) );
        FATAL_ERROR( Eigen::MatrixXd(dual_h2p.myContainer) == Eigen::MatrixXd::Identity(2,2) );
        FATAL_ERROR( Eigen::MatrixXd(primal_h0p.myContainer) == Eigen::MatrixXd::Identity(2,2) );
        FATAL_ERROR( Eigen::MatrixXd(dual_h0.myContainer) == Eigen::MatrixXd::Identity(2,2) );
    }

    const Calculus::DualDerivative0 primal_d0p = primal_calculus.derivative<0, DUAL>();
    const Calculus::PrimalDerivative0 dual_d0 = dual_calculus.derivative<0, PRIMAL>();
    {
        display_operator_info("primal d0p", primal_d0p);
        display_operator_info("dual d0p", dual_d0);

        Eigen::MatrixXd d0p_th_transpose(2, 7);
        d0p_th_transpose <<
            -1, -1, -1, -1,  0,  0,  0,
             0,  0,  0,  1,  1,  1,  1;

        FATAL_ERROR( Eigen::MatrixXd(primal_d0p.myContainer) == d0p_th_transpose.transpose() );
        FATAL_ERROR( Eigen::MatrixXd(dual_d0.myContainer) == d0p_th_transpose.transpose() );
    }

    const Calculus::DualDerivative1 primal_d1p = primal_calculus.derivative<1, DUAL>();
    const Calculus::PrimalDerivative1 dual_d1 = dual_calculus.derivative<1, PRIMAL>();
    {
        display_operator_info("primal d1p", primal_d1p);
        display_operator_info("dual d1", dual_d1);

        Eigen::MatrixXd minus_d1p_th_transpose(7, 6);
        minus_d1p_th_transpose <<
             1, -1,  0,  0,  0,  0,
            -1,  0,  1,  0,  0,  0,
             0,  1,  0, -1,  0,  0,
             0,  0, -1,  1,  0,  0,
             0,  0,  1,  0, -1,  0,
             0,  0,  0, -1,  0,  1,
             0,  0,  0,  0,  1, -1;

        FATAL_ERROR( Eigen::MatrixXd(primal_d1p.myContainer) == -minus_d1p_th_transpose.transpose() );
        FATAL_ERROR( Eigen::MatrixXd(dual_d1.myContainer) == -minus_d1p_th_transpose.transpose() );
    }

    /*

    const Calculus::PrimalHodge1 h1 = primal_calculus.primalHodge<1>();
    const Calculus::DualHodge1 h1p = primal_calculus.dualHodge<1>();
    {
        display_operator_info("h1", h1);
        display_operator_info("h1p", h1p);

        FATAL_ERROR( Eigen::MatrixXd(h1.myContainer) == Eigen::MatrixXd::Identity(7,7) );
        FATAL_ERROR( Eigen::MatrixXd((h1p*h1).myContainer) == -Eigen::MatrixXd::Identity(7,7) );
        FATAL_ERROR( Eigen::MatrixXd((h1*h1p).myContainer) == -Eigen::MatrixXd::Identity(7,7) );
    }

    trace.endBlock();

    trace.beginBlock("anti derivative");
    const LinearOperator<Calculus, 1, PRIMAL, 0, PRIMAL> ad1 = h2p * d1p * h1;
    const LinearOperator<Calculus, 2, PRIMAL, 1, PRIMAL> ad2 = h1p * d0p * h2;
    display_operator_info("ad1", ad1);
    display_operator_info("ad2", ad2);
    const LinearOperator<Calculus, 1, DUAL, 0, DUAL> ad1p = h2 * d1 * h1p;
    const LinearOperator<Calculus, 2, DUAL, 1, DUAL> ad2p = h1 * d0 * h2p;
    display_operator_info("ad1p", ad1p);
    display_operator_info("ad2p", ad2p);
    trace.endBlock();

    trace.beginBlock("laplace operators");
    const Calculus::PrimalIdentity0 lap_alpha = ad1 * d0;
    const Calculus::DualIdentity0 lap_alphap = ad1p * d0p;
    display_operator_info("lap_alpha", lap_alpha);
    display_operator_info("lap_alphap", lap_alphap);
    const Calculus::PrimalIdentity2 lap_beta = d1 * ad2;
    const Calculus::DualIdentity2 lap_betap = d1p * ad2p;
    display_operator_info("lap_beta", lap_beta);
    display_operator_info("lap_betap", lap_betap);
    trace.endBlock();

    trace.beginBlock("sharp and flat operator");

    { // primal sharp
        display_operator_info("#x", primal_calculus.sharpDirectional<PRIMAL, 0>());
        display_operator_info("#y", primal_calculus.sharpDirectional<PRIMAL, 1>());

        {
            Calculus::PrimalForm1::Container dx_container(7);
            dx_container << -1, 0, 0, 1, 0, 0, -1;
            const Calculus::PrimalForm1 dx(primal_calculus, dx_container);
            const Calculus::PrimalVectorField dx_field = primal_calculus.sharp(dx);

            {
                Board2D board;
                board << domain;
                board << primal_calculus;
                board << dx << dx_field;
                board.saveSVG("operators_dx_primal.svg");
            }

            FATAL_ERROR( dx_field.myCoordinates.col(0) == Eigen::VectorXd::Ones(6) );
            FATAL_ERROR( dx_field.myCoordinates.col(1) == Eigen::VectorXd::Zero(6) );
        }

        {
            Calculus::PrimalForm1::Container dy_container(7);
            dy_container << 0, 1, -1, 0, -1, 1, 0;
            const Calculus::PrimalForm1 dy(primal_calculus, dy_container);
            const Calculus::PrimalVectorField dy_field = primal_calculus.sharp(dy);

            {
                Board2D board;
                board << domain;
                board << primal_calculus;
                board << dy << dy_field;
                board.saveSVG("operators_dy_primal.svg");
            }

            FATAL_ERROR( dy_field.myCoordinates.col(0) == Eigen::VectorXd::Zero(6) );
            FATAL_ERROR( dy_field.myCoordinates.col(1) == Eigen::VectorXd::Ones(6) );
        }
    }

    { // dual sharp
        display_operator_info("#xp", primal_calculus.sharpDirectional<DUAL, 0>());
        display_operator_info("#yp", primal_calculus.sharpDirectional<DUAL, 1>());

        {
            Calculus::DualForm1::Container dx_container(7);
            dx_container << 0, -1, 1, 0, 1, -1, 0;
            const Calculus::DualForm1 dx(primal_calculus, dx_container);
            const Calculus::DualVectorField dx_field = primal_calculus.sharp(dx);

            {
                Board2D board;
                board << domain;
                board << primal_calculus;
                board << dx << dx_field;
                board.saveSVG("operators_dx_dual.svg");
            }

            //FATAL_ERROR( dx_field.myCoordinates.col(0) == Eigen::VectorXd::Ones(6) );
            //FATAL_ERROR( dx_field.myCoordinates.col(1) == Eigen::VectorXd::Zero(6) );
        }

        {
            Calculus::DualForm1::Container dy_container(7);
            dy_container << -1, 0, 0, 1, 0, 0, -1;
            const Calculus::DualForm1 dy(primal_calculus, dy_container);
            const Calculus::DualVectorField dy_field = primal_calculus.sharp(dy);

            {
                Board2D board;
                board << domain;
                board << primal_calculus;
                board << dy << dy_field;
                board.saveSVG("operators_dy_dual.svg");
            }

            FATAL_ERROR( dy_field.myCoordinates.col(0) == Eigen::VectorXd::Zero(2) );
            FATAL_ERROR( dy_field.myCoordinates.col(1) == Eigen::VectorXd::Ones(2) );
        }
    }
    */

    /*
    Calculus::PrimalVectorField primal_vector_field(primal_calculus);
    primal_vector_field.myCoordinates.col(0) = cos(2*M_PI/3) * Eigen::VectorXd::Ones(primal_vector_field.length());
    primal_vector_field.myCoordinates.col(1) = sin(2*M_PI/3) * Eigen::VectorXd::Ones(primal_vector_field.length());

    Calculus::DualVectorField dual_vector_field(primal_calculus);
    dual_vector_field.myCoordinates.col(0) = cos(2*M_PI/3) * Eigen::VectorXd::Ones(dual_vector_field.length());
    dual_vector_field.myCoordinates.col(1) = sin(2*M_PI/3) * Eigen::VectorXd::Ones(dual_vector_field.length());

    {
        Board2D board;
        board << domain;
        board << primal_calculus;
        board << primal_vector_field;
        board << dual_vector_field;
        board.saveSVG("operators_vector_field.svg");
    }
    */

    trace.endBlock();

    trace.endBlock();
}


int
main()
{
    test_manual_operators();
    //test_linear_ring();
    //test_linear_structure();
    return 0;
}

