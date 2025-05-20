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
/// @file dec/exampleDECSurface.cpp
/**
   Example of solving Poisson equation using the DEC package.
    @see \ref sectDECPoissonProblem
   \image html alcapone_phi.png "Solution dual 0-form."
   \example dec/exampleDECSurface.cpp
**/
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/DiscreteExteriorCalculusFactory.h"

#include "ConfigExamples.h"

typedef DGtal::PolyscopeViewer<DGtal::Z3i::Space, DGtal::Z3i::KSpace> Viewer;

template <typename Predicate, typename Domain>
struct FalseOutsideDomain
{
    BOOST_CONCEPT_ASSERT(( DGtal::concepts::CPointPredicate<Predicate> ));
    BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDomain<Domain> ));

    typedef typename Predicate::Point Point;

    FalseOutsideDomain(DGtal::ConstAlias<Predicate> predicate, DGtal::ConstAlias<Domain> adomain) :
        myPredicate(&predicate), myDomain(&adomain)
    {
    }

    bool
    operator()(const Point& point) const
    {
        if (!myDomain->isInside(point)) return false;
        return (*myPredicate)(point);
    }

    const Predicate* myPredicate;
    const Domain* myDomain;
};

void
alcapone_3d()
{
    using std::endl;
    using DGtal::trace;

    trace.beginBlock("alcapone");

    const std::string filename = examplesPath + "samples/Al.100.vol";

    trace.beginBlock("digital surface");

    //! [alcapone_surface]
    typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain, bool> Image;
    typedef DGtal::functors::Cast<bool> Functor;
    const Image image = DGtal::VolReader<Image, Functor>::importVol(filename, Functor());
    const DGtal::Z3i::Domain domain = image.domain();

    trace.info() << "domain=" << domain << endl;

    typedef FalseOutsideDomain<Image, DGtal::Z3i::Domain> ImageExtended;
    const ImageExtended image_extended(image, domain);

    DGtal::Z3i::KSpace kspace;
    kspace.init(domain.lowerBound(), domain.upperBound()+DGtal::Z3i::Point(0,0,1), true);

    const DGtal::Z3i::KSpace::SCell cell_bel = DGtal::Surfaces<DGtal::Z3i::KSpace>::findABel(kspace, image_extended);

    typedef DGtal::SurfelAdjacency<3> SurfelAdjacency;
    const SurfelAdjacency surfel_adjacency(true);

    typedef DGtal::LightImplicitDigitalSurface<DGtal::Z3i::KSpace, ImageExtended> DigitalSurfaceContainer;
    const DigitalSurfaceContainer digital_surface_container(kspace, image_extended, surfel_adjacency, cell_bel);

    typedef DGtal::DigitalSurface<DigitalSurfaceContainer> DigitalSurface;
    const DigitalSurface digital_surface(digital_surface_container);
    //! [alcapone_surface]

    trace.info() << "digital_surface_size=" << digital_surface.size() << endl;

    {
        Viewer* viewer = new Viewer(kspace);
        viewer->allowReuseList = true;
        for (DigitalSurface::ConstIterator si=digital_surface.begin(), se=digital_surface.end(); si!=se; si++)
        {
            const DGtal::Z3i::KSpace::SCell cell = *si;
            (*viewer) << cell;
        }
        viewer->show();
        delete viewer;
    }

    trace.endBlock();

    trace.beginBlock("discrete exterior calculus");

    //! [alcapone_calculus]
    typedef DGtal::DiscreteExteriorCalculusFactory<DGtal::EigenLinearAlgebraBackend> CalculusFactory;
    typedef DGtal::DiscreteExteriorCalculus<2, 3, DGtal::EigenLinearAlgebraBackend> Calculus;
    const Calculus calculus = CalculusFactory::createFromNSCells<2>(digital_surface.begin(), digital_surface.end(), false);
    //! [alcapone_calculus]
    trace.info() << "calculus=" << calculus << endl;

    {
        Viewer* viewer = new Viewer(kspace);
        (*viewer) << calculus;
        viewer->show();
        delete viewer;
    }

    using DGtal::PRIMAL;
    using DGtal::DUAL;

    trace.endBlock();

    trace.beginBlock("poisson equation");

    //! [alcapone_rho]
    Calculus::DualForm0 rho(calculus);
    for (int index=0; index<rho.length(); index++)
    {
        const Calculus::SCell cell = rho.getSCell(index);
        const Calculus::Point point = kspace.sKCoords(cell);
        if (point[2]>1) continue;
        rho.myContainer(index) = point[1]>100 ? 1 : -1;
    }
    trace.info() << "rho_range=" << rho.myContainer.minCoeff() << "/" << rho.myContainer.maxCoeff() << endl;
    //! [alcapone_rho]

    {
        Viewer* viewer = new Viewer(kspace);
        (*viewer) << rho;
        viewer->show();
        delete viewer;
    }

    //! [alcapone_phi]
    const Calculus::DualIdentity0 laplace = calculus.laplace<DUAL>();

    typedef DGtal::EigenLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
    typedef DGtal::DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> PoissonSolver;
    PoissonSolver solver;
    solver.compute(laplace);
    const Calculus::DualForm0 phi = solver.solve(rho);
    trace.info() << "phi_range=" << phi.myContainer.minCoeff() << "/" << phi.myContainer.maxCoeff() << endl;
    //! [alcapone_phi]

    {
        Viewer* viewer = new Viewer(kspace);
        (*viewer) << phi;
        viewer->show();
        delete viewer;
    }

    trace.endBlock();

    trace.endBlock();
}

void
pyramid_3d()
{
    using std::endl;
    using DGtal::trace;

    trace.beginBlock("pyramid example");

    trace.beginBlock("digital surface");

    const DGtal::Z3i::Domain input_domain(DGtal::Z3i::Point::diagonal(-1), DGtal::Z3i::Point::diagonal(2));
    trace.info() << "input_domain=" << input_domain << endl;

    DGtal::Z3i::KSpace kspace;
    kspace.init(input_domain.lowerBound(), input_domain.upperBound(), true);

    //! [surface_input_set]
    DGtal::Z3i::DigitalSet input_set(input_domain);
    input_set.insert( DGtal::Z3i::Point(0,0,0) );
    input_set.insert( DGtal::Z3i::Point(1,0,0) );
    input_set.insert( DGtal::Z3i::Point(1,1,0) );
    input_set.insert( DGtal::Z3i::Point(0,1,0) );
    input_set.insert( DGtal::Z3i::Point(0,0,1) );
    //! [surface_input_set]
    trace.info() << "input_set_size=" << input_set.size() << endl;

    {
        Viewer* viewer = new Viewer(kspace);
        (*viewer) << input_set;
        viewer->show();
        delete viewer;
    }

    //! [surface_digital_surface]
    const DGtal::Z3i::KSpace::SCell cell_bel = DGtal::Surfaces<DGtal::Z3i::KSpace>::findABel(kspace, input_set);

    typedef DGtal::SurfelAdjacency<3> SurfelAdjacency;
    const SurfelAdjacency surfel_adjacency(true);

    typedef DGtal::LightImplicitDigitalSurface<DGtal::Z3i::KSpace, DGtal::Z3i::DigitalSet> DigitalSurfaceContainer;
    const DigitalSurfaceContainer digital_surface_container(kspace, input_set, surfel_adjacency, cell_bel);

    typedef DGtal::DigitalSurface<DigitalSurfaceContainer> DigitalSurface;
    const DigitalSurface digital_surface(digital_surface_container);
    //! [surface_digital_surface]

    trace.info() << "surfel_adjacency=" << endl;
    for (int ii=0; ii<3; ii++)
    {
        std::stringstream ss;
        for (int jj=0; jj<3; jj++)
            ss << surfel_adjacency.getAdjacency(ii, jj) << " ";
        trace.info() << ss.str() << endl;
    }

    trace.info() << "digital_surface_container=" << digital_surface_container << endl;

    trace.info() << "digital_surface_size=" << digital_surface.size() << endl;

    {
        Viewer* viewer = new Viewer(kspace);
        for (DigitalSurface::ConstIterator si=digital_surface.begin(), se=digital_surface.end(); si!=se; si++)
        {
            const DGtal::Z3i::KSpace::SCell cell = *si;
            (*viewer) << cell;
        }
        viewer->show();
        delete viewer;
    }

    trace.endBlock();

    trace.beginBlock("discrete exterior calculus");

    //! [surface_calculus]
    typedef DGtal::DiscreteExteriorCalculusFactory<DGtal::EigenLinearAlgebraBackend> CalculusFactory;
    typedef DGtal::DiscreteExteriorCalculus<2, 3, DGtal::EigenLinearAlgebraBackend> Calculus;
    const Calculus calculus = CalculusFactory::createFromNSCells<2>(digital_surface.begin(), digital_surface.end());
    //! [surface_calculus]
    trace.info() << "calculus=" << calculus << endl;

    {
        Viewer* viewer = new Viewer(kspace);
        (*viewer) << calculus;
        viewer->show();
        delete viewer;
    }

    using DGtal::PRIMAL;
    using DGtal::DUAL;

    const Calculus::PrimalForm2 primal_surfel_area = calculus.hodge<0, DUAL>() *
        Calculus::DualForm0(calculus, Eigen::VectorXd::Ones(calculus.kFormLength(0, DUAL)));

    const Calculus::DualForm2 dual_surfel_area = calculus.hodge<0, PRIMAL>() *
        Calculus::PrimalForm0(calculus, Eigen::VectorXd::Ones(calculus.kFormLength(0, PRIMAL)));

    const double area_th = calculus.kFormLength(2, PRIMAL);
    const double area_primal = dual_surfel_area.myContainer.sum();
    const double area_dual = primal_surfel_area.myContainer.sum();
    trace.info() << "area_th=" << area_th << endl;
    trace.info() << "area_primal=" << area_primal << endl;
    trace.info() << "area_dual=" << area_dual << endl;
    FATAL_ERROR( area_th == area_primal );
    FATAL_ERROR( area_th == area_dual );

    const Calculus::DualForm1 dual_edge_length = calculus.hodge<1, PRIMAL>() *
        Calculus::PrimalForm1(calculus, Eigen::VectorXd::Ones(calculus.kFormLength(1, PRIMAL)));

    const bool dual_edge_length_match = dual_edge_length.myContainer == Eigen::VectorXd::Ones(calculus.kFormLength(1, DUAL)) ;
    trace.info() << "dual_edge_length_match=" << dual_edge_length_match << endl;
    FATAL_ERROR( dual_edge_length_match );

    trace.beginBlock("dual surfel area");

    for (Calculus::Index index=0; index<dual_surfel_area.length(); index++)
    {
        const Calculus::SCell cell = dual_surfel_area.getSCell(index);
        const Calculus::Scalar area = dual_surfel_area.myContainer(index);
        trace.info() << index << " " << cell << " " << area << endl;
    }

    trace.endBlock();

    trace.endBlock();

    trace.endBlock();
}

int main(int argc, char* argv[])
{
    //pyramid_3d();
    alcapone_3d();

    return 0;
}
