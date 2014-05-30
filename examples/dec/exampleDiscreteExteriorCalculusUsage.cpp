#include <string>
using namespace std;

#include "common.h"

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/readers/GenericReader.h"
using namespace DGtal;

void usage2d()
{
    trace.beginBlock("2d discrete exterior calculus usage");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));

    // create discrete exterior calculus from set
    typedef DiscreteExteriorCalculus<2, EigenLinearAlgebraBackend> Calculus;
    Calculus calculus(generateRingSet(domain));

    trace.info() << calculus.eraseSCell(calculus.myKSpace.sSpel(Z2i::Point(8, 5))) << endl;
    trace.info() << calculus.eraseSCell(calculus.myKSpace.sCell(Z2i::Point(18, 11))) << endl;

    trace.info() << calculus << endl;

    {
        Board2D board;
        board << domain;
        board << calculus;
        board.saveSVG("usage_calculus.svg");
    }

    const Z2i::Point center(13,7);

    // primal path
    {
        trace.info() << "primal path" << endl;

        // create primal 0-form and fill it with eucledian metric
        Calculus::PrimalForm0 primal_zero_form(calculus);
        for (Calculus::Index index=0; index<primal_zero_form.myContainer.rows(); index++)
        {
            const Calculus::SCell& cell = primal_zero_form.getSCell(index);
            const double value = Z2i::l2Metric(cell.myCoordinates, center)/2;
            //trace.info() << cell << " " << value << endl;
            primal_zero_form.myContainer(index) = value;
        }
        // one can do linear algebra operation between equaly typed kforms
        const Calculus::PrimalForm0 prout = 2 * primal_zero_form + primal_zero_form;

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(0, 15));
            board << primal_zero_form;
            board.saveSVG("usage_primal_zero_form.svg");
        }

        // create primal gradient vector field and primal derivative one form
        const Calculus::PrimalDerivative0 primal_zero_derivative = calculus.derivative<0, PRIMAL>();
        const Calculus::PrimalForm1 primal_one_form = primal_zero_derivative * primal_zero_form;
        const Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(-2, 2));
            board << primal_one_form;
            board << primal_vector_field;
            board.saveSVG("usage_primal_one_form.svg");
        }

        // test primal flat and sharp
        const Calculus::PrimalForm1 flat_sharp_primal_one_form = calculus.flat(primal_vector_field);
        const Calculus::PrimalVectorField sharp_flat_primal_vector_field = calculus.sharp(flat_sharp_primal_one_form);

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(-2, 2));
            board << flat_sharp_primal_one_form;
            board << sharp_flat_primal_vector_field;
            board.saveSVG("usage_primal_one_form_sharp_flat.svg");
        }

        // create dual gradient vector field and hodge*d dual one form
        const Calculus::PrimalHodge1 primal_one_hodge = calculus.primalHodge<1>();
        const Calculus::DualForm1 dual_one_form = primal_one_hodge * primal_zero_derivative * primal_zero_form;
        const Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(-2, 2));
            board << dual_one_form;
            board << dual_vector_field;
            board << primal_vector_field;
            board.saveSVG("usage_primal_one_form_hodge.svg");
        }
    }

    // dual path
    {
        trace.info() << "dual path" << endl;

        // create dual 0-form and fill it with eucledian metric
        Calculus::DualForm0 dual_zero_form(calculus);
        for (Calculus::Index index=0; index<dual_zero_form.myContainer.rows(); index++)
        {
            const Calculus::SCell& cell = dual_zero_form.getSCell(index);
            const double value = Z2i::l2Metric(cell.myCoordinates, center)/2;
            //trace.info() << cell << " " << value << endl;
            dual_zero_form.myContainer(index) = value;
        }

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(0, 15));
            board << dual_zero_form;
            board.saveSVG("usage_dual_zero_form.svg");
        }

        // create dual gradient vector field and dual derivative one form
        const Calculus::DualDerivative0 dual_zero_derivative = calculus.derivative<0, DUAL>();
        const Calculus::DualForm1 dual_one_form = dual_zero_derivative * dual_zero_form;
        const Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(-2, 2));
            board << dual_one_form;
            board << dual_vector_field;
            board.saveSVG("usage_dual_one_form.svg");
        }

        // create primal gradient vector field and hodge*d primal one form
        const Calculus::DualHodge1 dual_one_hodge = calculus.dualHodge<1>();
        const Calculus::PrimalForm1 primal_one_form = dual_one_hodge * dual_zero_derivative * dual_zero_form;
        const Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);

        {
            Board2D board;
            board << domain;
            board << calculus;
            board << CustomStyle("KForm", new KFormStyle2D(-2, 2));
            board << primal_one_form;
            board << primal_vector_field;
            board << dual_vector_field;
            board.saveSVG("usage_dual_one_form_hodge.svg");
        }
    }

    trace.endBlock();
}

int main(int argc, char* argv[])
{
    usage2d();

    return 0;
}

