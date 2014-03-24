#include <string>
using namespace std;

#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/math/linalg/EigenLinearAlgebra.h"
#include "DGtal/dec/LinearOperator.h"
#include "DGtal/dec/KForm.h"
#include "DGtal/dec/VectorField.h"
#include "DGtal/dec/Duality.h"

//#include <QApplication>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
//#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/readers/GenericReader.h"
using namespace DGtal;

void demo2d()
{
    trace.beginBlock("2d discrete exterior calculus demo");

    const Z2i::Domain domain(Z2i::Point(0,0), Z2i::Point(9,9));
    Z2i::DigitalSet set(domain);

    // create initial set
    for (int ii=0; ii<3; ii++)
        for (int jj=2; jj<8; jj++)
        {
            set.insert(Z2i::Point(jj,ii+1));
            set.insert(Z2i::Point(jj,ii+6));
            set.insert(Z2i::Point(ii+1,jj));
            set.insert(Z2i::Point(ii+6,jj));
        }

    //for (int ii=0; ii<10; ii++)
    //    for (int jj=0; jj<10; jj++)
    //        set.insert(Z2i::Point(ii,jj));

    //set.insert(Z2i::Point(6,2));
    //set.insert(Z2i::Point(5,3));
    //set.insert(Z2i::Point(7,3));
    //set.insert(Z2i::Point(6,4));

    //set.insert(Z2i::Point(4,4));
    //set.insert(Z2i::Point(4,5));
    //set.insert(Z2i::Point(5,4));
    //set.insert(Z2i::Point(5,5));

    {
        Board2D board;
        board << domain;
        board << set;
        board.saveSVG("demo_set.svg");
    }

    // create discrete exterior calculus from set
    typedef DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebra> Calculus;
    Calculus calculus(set);
    trace.info() << calculus;

    typedef AllSCellMap<Calculus, double> Accum;

    {
        typedef GradientColorMap<double, CMAP_JET> Colormap;
        Colormap colormap(-1,1);
        Board2D board;
        board << domain;
        Accum accum(calculus);
        accum.display2D(board, colormap);
        board.saveSVG("demo_calculus.svg");
    }

    const Z2i::Point center(13,7);

    // primal path
    {
        trace.info() << "primal path" << endl;

        // create primal 0-form and fill it with eucledian metric
        Calculus::PrimalForm0 primal_zero_form(calculus);
        for (Calculus::Index index=0; index<primal_zero_form.container.rows(); index++)
        {
            const Calculus::SCell& cell = primal_zero_form.getSCell(index);
            const double value = Z2i::l2Metric(cell.myCoordinates, center)/2;
            //trace.info() << cell << " " << value << endl;
            primal_zero_form.container(index) = value;
        }
        // one can do linear algebra operation between equaly typed kforms
        const Calculus::PrimalForm0 prout = 2 * primal_zero_form + primal_zero_form;

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(0,15);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            primal_zero_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            board.saveSVG("demo_primal_zero_form.svg");
        }

        // create primal gradient vector field and primal derivative one form
        const Calculus::PrimalDerivative0 primal_zero_derivative = calculus.derivative<0, PRIMAL>();
        const Calculus::PrimalForm1 primal_one_form = primal_zero_derivative * primal_zero_form;
        const Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(-2,2);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            primal_one_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            primal_vector_field.display2D(board);
            board.saveSVG("demo_primal_one_form.svg");
        }

        // test primal flat and sharp
        const Calculus::PrimalForm1 flat_sharp_primal_one_form = calculus.flat(primal_vector_field);
        const Calculus::PrimalVectorField sharp_flat_primal_vector_field = calculus.sharp(flat_sharp_primal_one_form);

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(-2,2);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            flat_sharp_primal_one_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            sharp_flat_primal_vector_field.display2D(board);
            board.saveSVG("demo_primal_one_form_sharp_flat.svg");
        }

        // create dual gradient vector field and hodge*d dual one form
        const Calculus::PrimalHodge1 primal_one_hodge = calculus.primalHodge<1>();
        const Calculus::DualForm1 dual_one_form = primal_one_hodge * primal_zero_derivative * primal_zero_form;
        const Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(-2,2);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            dual_one_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            dual_vector_field.display2D(board);
            primal_vector_field.display2D(board);
            board.saveSVG("demo_primal_one_form_hodge.svg");
        }
    }

    // dual path
    {
        trace.info() << "dual path" << endl;

        // create dual 0-form and fill it with eucledian metric
        Calculus::DualForm0 dual_zero_form(calculus);
        for (Calculus::Index index=0; index<dual_zero_form.container.rows(); index++)
        {
            const Calculus::SCell& cell = dual_zero_form.getSCell(index);
            const double value = Z2i::l2Metric(cell.myCoordinates, center)/2;
            //trace.info() << cell << " " << value << endl;
            dual_zero_form.container(index) = value;
        }

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(0,15);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            dual_zero_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            board.saveSVG("demo_dual_zero_form.svg");
        }

        // create dual gradient vector field and dual derivative one form
        const Calculus::DualDerivative0 dual_zero_derivative = calculus.derivative<0, DUAL>();
        const Calculus::DualForm1 dual_one_form = dual_zero_derivative * dual_zero_form;
        const Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(-2,2);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            dual_one_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            dual_vector_field.display2D(board);
            board.saveSVG("demo_dual_one_form.svg");
        }

        // create primal gradient vector field and hodge*d primal one form
        const Calculus::DualHodge1 dual_one_hodge = calculus.dualHodge<1>();
        const Calculus::PrimalForm1 primal_one_form = dual_one_hodge * dual_zero_derivative * dual_zero_form;
        const Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);

        {
            typedef GradientColorMap<double, CMAP_JET> Colormap;
            Colormap colormap(-2,2);
            Board2D board;
            board << domain;
            Accum accum(calculus);
            primal_one_form.applyToAccum(accum);
            accum.display2D(board, colormap);
            primal_vector_field.display2D(board);
            dual_vector_field.display2D(board);
            board.saveSVG("demo_dual_one_form_hodge.svg");
        }
    }

    trace.endBlock();
}

/*
void test2d(const Options& options)
{
    trace.beginBlock("2d discrete exterior calculus test");

    typedef Z2i::Domain Domain;
    typedef Z2i::DigitalSet DigitalSet;
    typedef Z2i::KSpace KSpace;
    typedef ImageSelector<Domain, int>::Type Input;

    const Input input = GenericReader<Input>::import(options.input2d);
    const Domain domain = input.domain();
    trace.info() << "domain=" << domain << endl;

    DigitalSet set(domain);
    int input_threshold = 127;
    for (Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Domain::Point& point = *di;
        const Input::Value value = input(point);
        if (value<input_threshold) set.insertNew(point);
    }
    trace.info() << "set.size()=" << set.size() << endl;

    typedef DiscreteExteriorCalculus<Domain, EigenSparseLinearAlgebra> Calculus;
    Calculus calculus(set);

    calculus.cell_size_ratio.writeImage("size2d.mha");
    calculus.cell_indexes.writeImage("index2d.mha");
    trace.info() << calculus;

    typedef AllSCellMap<Calculus, double> AllSCellAccum;
    AllSCellAccum accum(calculus);

    Calculus::PrimalForm0 scalar_field(calculus);
    scalar_field.container(0) = -1;
    scalar_field.container(1) = 1;
    scalar_field.applyToAccum(accum);
    trace.info() << scalar_field << endl;

    Calculus::DualForm0 dual_zero_form(calculus);
    dual_zero_form.container(0) = -2;
    dual_zero_form.container(1) = 2;
    dual_zero_form.applyToAccum(accum);
    trace.info() << dual_zero_form << endl;

    accum.writeImage("accum2d.mha");

    Calculus::PrimalForm1 primal_one_form(calculus);
    primal_one_form.container.zero(1);
    Calculus::DualForm1 dual_one_form(calculus);
    dual_one_form.container.zero(2);

    Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);
    trace.info() << primal_one_form << endl;
    trace.info() << primal_vector_field << endl;
    trace.info() << calculus.flat(primal_vector_field) << endl;
    Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);
    trace.info() << dual_one_form << endl;
    trace.info() << (dual_one_form + dual_one_form) << endl;
    trace.info() << dual_vector_field << endl;
    trace.info() << dual_vector_field.extractZeroForm(0) << endl;
    trace.info() << calculus.flat(dual_vector_field) << endl;

    Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
    Calculus::DualDerivative1 d1p = calculus.derivative<1, DUAL>();
    Calculus::PrimalHodge1 hodge1 = calculus.primalHodge<1>();
    Calculus::DualHodge2 hodge2p = calculus.dualHodge<2>();
    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "d0 + d0 = " << (d0 + d0) << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d1p = " << d1p << endl;
    trace.info() << "hodge2p = " << hodge2p << endl;
    trace.info() << "lap = " << (hodge2p * d1p * hodge1 * d0) << endl;
    trace.info() << (d0 * scalar_field) << endl;

    Board2D board_one_hodge;
    (calculus.primalHodge<1>() * primal_one_form).applyToAccum(accum);
    accum.display2D(board_one_hodge, GradientColorMap<double, CMAP_JET>(-2,2));
    primal_vector_field.display2D(board_one_hodge);
    dual_vector_field.display2D(board_one_hodge);
    board_one_hodge.saveSVG("board_one_hodge.svg");

    Board2D board_two_hodge;
    (calculus.dualHodge<1>() * calculus.primalHodge<1>() * primal_one_form).applyToAccum(accum);
    accum.display2D(board_two_hodge, GradientColorMap<double, CMAP_JET>(-2,2));
    calculus.sharp(calculus.primalHodge<1>() * primal_one_form).display2D(board_two_hodge);
    board_two_hodge.saveSVG("board_two_hodge.svg");

    trace.endBlock();
}

template <typename Viewer>
void test3d(const Options& options, Viewer& viewer)
{
    trace.beginBlock("3d discrete exterior calculus test");

    typedef Z3i::Domain Domain;
    typedef Z3i::DigitalSet DigitalSet;
    typedef ImageSelector<Domain, int>::Type Input;

    const Input input = GenericReader<Input>::import(options.input3d);
    const Domain domain = input.domain();
    trace.info() << "domain=" << domain << endl;

    DigitalSet set(domain);
    int input_threshold = 127;
    for (Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Domain::Point& point = *di;
        const Input::Value value = input(point);
        if (value>input_threshold) set.insertNew(point);
    }
    trace.info() << "set.size()=" << set.size() << endl;

    typedef DiscreteExteriorCalculus<Domain, EigenSparseLinearAlgebra> Calculus;
    Calculus calculus(set);

    calculus.cell_size_ratio.writeImage("size3d.mha");
    calculus.cell_indexes.writeImage("index3d.mha");
    trace.info() << calculus;

    typedef AllSCellMap<Calculus, double> AllSCellAccum;
    AllSCellAccum accum(calculus);

    Calculus::PrimalForm0 scalar_field(calculus);
    scalar_field.container(0) = 1;
    scalar_field.container(1) = 2;
    scalar_field.applyToAccum(accum);
    trace.info() << scalar_field << endl;

    Calculus::DualForm0 dual_zero_form(calculus);
    dual_zero_form.container(0) = 3;
    dual_zero_form.container(1) = 4;
    dual_zero_form.applyToAccum(accum);
    trace.info() << dual_zero_form << endl;

    accum.writeImage("accum3d.mha");

    Calculus::PrimalForm1 primal_one_form(calculus);
    primal_one_form.container.zero(1);
    Calculus::PrimalVectorField primal_vector_field = calculus.sharp(primal_one_form);
    trace.info() << primal_one_form << endl;
    trace.info() << primal_vector_field << endl;

    Calculus::DualForm1 dual_one_form(calculus);
    dual_one_form.container.zero(2);
    Calculus::DualVectorField dual_vector_field = calculus.sharp(dual_one_form);
    trace.info() << dual_one_form << endl;
    trace.info() << dual_vector_field << endl;

    (calculus.primalHodge<1>() * primal_one_form).applyToAccum(accum);

    Calculus::PrimalDerivative0 d0 = calculus.derivative<0, PRIMAL>();
    Calculus::DualDerivative2 d2p = calculus.derivative<2, DUAL>();
    Calculus::PrimalHodge1 hodge1 = calculus.primalHodge<1>();
    Calculus::DualHodge3 hodge3p = calculus.dualHodge<3>();
    trace.info() << "d0 = " << d0 << endl;
    trace.info() << "hodge1 = " << hodge1 << endl;
    trace.info() << "d2p = " << d2p << endl;
    trace.info() << "hodge3p = " << hodge3p << endl;
    trace.info() << "lap = " << (hodge3p * d2p * hodge1 * d0) << endl;
    trace.info() << (d0 * scalar_field) << endl;

    accum.display3D(viewer, GradientColorMap<double, CMAP_JET>(0,4));
    viewer << Viewer::updateDisplay;

    trace.endBlock();
}
*/

int main(int argc, char* argv[])
{
    /*
    Options options;
    options.input3d = "../foamutils/debug.vol";
    options.input2d = "../foamutils/debug.png";
    trace.info() << "input3d=" << options.input3d << endl;
    trace.info() << "input2d=" << options.input2d << endl;

    QApplication app(argc, argv);
    Z3i::KSpace kspace3d;
    Viewer3D<Z3i::Space, Z3i::KSpace> viewer(kspace3d);
    viewer.show();
    test3d(options, viewer);

    test2d(options);
    */

    demo2d();

    return 0;
    //return app.exec();
}

