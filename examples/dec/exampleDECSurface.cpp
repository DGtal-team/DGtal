#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/DigitalSetBoundary.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/DiscreteExteriorCalculusFactory.h"

typedef DGtal::Viewer3D<DGtal::Z3i::Space, DGtal::Z3i::KSpace> Viewer;
typedef DGtal::Display3DFactory<DGtal::Z3i::Space, DGtal::Z3i::KSpace> DisplayFactory;

int main(int argc, char* argv[])
{
    using std::endl;
    using DGtal::trace;

    QApplication app(argc, argv);

    trace.beginBlock("digital surface");

    const DGtal::Z3i::Domain input_domain(DGtal::Z3i::Point::diagonal(-1), DGtal::Z3i::Point::diagonal(2));
    trace.info() << "input_domain=" << input_domain << endl;

    DGtal::Z3i::KSpace kspace;
    kspace.init(input_domain.lowerBound(), input_domain.upperBound(), false);

    DGtal::Z3i::DigitalSet input_set(input_domain);
    input_set.insert( DGtal::Z3i::Point(0,0,0) );
    input_set.insert( DGtal::Z3i::Point(1,0,0) );
    input_set.insert( DGtal::Z3i::Point(1,1,0) );
    input_set.insert( DGtal::Z3i::Point(0,1,0) );
    input_set.insert( DGtal::Z3i::Point(0,0,1) );
    trace.info() << "input_set_size=" << input_set.size() << endl;

    Viewer viewer1(kspace);
    viewer1.show();
    viewer1.setWindowTitle("input set");
    viewer1 << input_set;
    viewer1 << Viewer::updateDisplay;

    typedef DGtal::SurfelAdjacency<3> SurfelAdjacency;
    const SurfelAdjacency surfel_adjacency(true);
    trace.info() << "surfel_adjacency=" << endl;
    for (int ii=0; ii<3; ii++)
    {
        std::stringstream ss;
        for (int jj=0; jj<3; jj++)
            ss << surfel_adjacency.getAdjacency(ii, jj) << " ";
        trace.info() << ss.str() << endl;
    }

    typedef DGtal::DigitalSetBoundary<DGtal::Z3i::KSpace, DGtal::Z3i::DigitalSet> DigitalSurfaceContainer;
    const DigitalSurfaceContainer digital_surface_container(kspace, input_set, surfel_adjacency);
    trace.info() << "digital_surface_container=" << digital_surface_container << endl;

    typedef DGtal::DigitalSurface<DigitalSurfaceContainer> DigitalSurface;
    const DigitalSurface digital_surface(digital_surface_container);
    trace.info() << "digital_surface_size=" << digital_surface.size() << endl;

    Viewer viewer2(kspace);
    viewer2.show();
    viewer2.setWindowTitle("digital surface");
    for (DigitalSurface::ConstIterator si=digital_surface.begin(), se=digital_surface.end(); si!=se; si++)
    {
        const DGtal::Z3i::KSpace::SCell cell = *si;
        viewer2 << cell;
    }
    viewer2 << Viewer::updateDisplay;

    trace.endBlock();

    trace.beginBlock("discrete exterior calculus");

    typedef DGtal::DiscreteExteriorCalculusFactory<DGtal::EigenLinearAlgebraBackend> CalculusFactory;
    typedef DGtal::DiscreteExteriorCalculus<2, 3, DGtal::EigenLinearAlgebraBackend> Calculus;
    const Calculus calculus = CalculusFactory::createFromNSCells<2>(digital_surface.begin(), digital_surface.end());
    trace.info() << "calculus=" << calculus << endl;

    Viewer viewer3(kspace);
    viewer3.show();
    viewer3.setWindowTitle("discrete exterior calculus");
    DisplayFactory::draw(viewer3, calculus);
    viewer3 << Viewer::updateDisplay;

    trace.endBlock();

    return app.exec();
}

