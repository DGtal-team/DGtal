/**
  * This program is free software : you can redistribuate it and/or modify
  * it under the terms of the GNU Lesser General Public License as
  * published by the Free Software Foundation, either version 3 of the
  * License, or (at you option) any later version.
  *
  * This program is distributed int the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, set <http://www.gnu.org/license/>.
**/

/**
 * @file shapes/exampleTriangulatedSurface.cpp
 * @ingroup Examples
 * @author Thomas Caissard (\c thomas.caissard@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2017/06/20
 *
 * An example file named exampleTriangulatedSurface.
 *
 * This file is part of the DGtal library.
**/

//! [TriangulatedSurfaceUsageFull]

#define WITH_3D

////////// INCLUDES /////////

#include <DGtal/helpers/StdDefs.h>

#include <DGtal/base/BasicFunctors.h>

#include <DGtal/topology/DigitalSurface.h>
#include <DGtal/topology/SetOfSurfels.h>
#include <DGtal/topology/LightImplicitDigitalSurface.h>

#include <DGtal/io/viewers/Viewer3D.h>
#include <DGtal/io/colormaps/ColorBrightnessColorMap.h>

#include <DGtal/geometry/surfaces/estimation/LocalEstimatorFromSurfelFunctorAdapter.h>
#include <DGtal/geometry/surfaces/estimation/IIGeometricFunctors.h>
#include <DGtal/geometry/surfaces/estimation/IntegralInvariantCovarianceEstimator.h>

#include <DGtal/shapes/parametric/Ball3D.h>
#include <DGtal/shapes/Shapes.h>
#include <DGtal/shapes/GaussDigitizer.h>
#include <DGtal/shapes/Mesh.h>
#include <DGtal/shapes/MeshHelpers.h>
#include "DGtal/shapes/TriangulatedSurface.h"

#include <DGtal/math/linalg/EigenSupport.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/errors.hpp>

///////// NAMESPACES //////////

using namespace std;
using namespace DGtal;
using namespace Eigen;
using namespace Z3i;

////////// TYPEDEFS //////////

typedef Z3i::Space Space;
typedef Z3i::KSpace KSpace;
typedef Z3i::RealVector RealVector;
typedef Z3i::RealPoint RealPoint;
typedef Z3i::Point Point;

template <typename TSpace>
inline double dot(typename TSpace::RealVector a, typename TSpace::RealVector b)
{
  double sum = 0.;
  for(int i = 0; i < TSpace::dimension; i++) sum += a[i] + b[i];
  return sum;
}

double dot(const RealPoint& a, const RealPoint& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

RealPoint crossProduct(const RealPoint& a, const RealPoint& b)
{
  return RealPoint( a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]);
}

RealPoint cartesian_to_spherical(const RealPoint & a)
{
  return RealPoint(a.norm(), atan2(a[1], a[0]), acos(a[2]));
}

struct Options
{
  double h;
  int function;
  int smooth;

  std::string error_output;
};

Options
parse_options(int argc, char* argv[])
{
  namespace po = boost::program_options;

  using DGtal::trace;
  using std::endl;

  Options options;

  po::options_description po_shape("Shape options");
  po_shape.add_options()
    ("step,s", po::value<double>(&options.h)->default_value(0.1), "grid step")
    ("error-output", po::value<std::string>(&options.error_output)->default_value("error.dat"), "error output")
    ;

  po::options_description po_computation("Computations options");
  po_computation.add_options()
    ("function,f", po::value<int>(&options.function)->default_value(0), "0 for x^2, 1 for cos, 2 for exp")
    ("smooth,m", po::value<int>(&options.smooth)->default_value(0), "Wether or not to project points onto the sphere")
    ;

  po::options_description po_options("mesh_laplacian_3D [options]");
  po_options.add(po_shape).add(po_computation).add_options()
    ("help,h", "display this message")
    ;

  try
  {
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(po_options).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      trace.info() << po_options;
      std::exit(0);
    }
  }
  catch (std::exception& ex)
  {
    trace.error() << ex.what() << endl;
    trace.info() << po_options;
    std::exit(1);
  }

  return options;
}

template <typename Shape>
void laplacian(Shape& shape, const Options& options,
    std::function<double(const RealPoint&)> input_function,
    std::function<double(const RealPoint&)> result_function,
    int argc, char** argv)
{
  trace.beginBlock("Laplacian");
  typedef GaussDigitizer<Z3i::Space, Shape> Digitizer;

  trace.beginBlock("Digitizing the shape");
  Digitizer digitizer;
  digitizer.attach(shape);
  digitizer.init(shape.getLowerBound() + Z3i::Vector(-1,-1,-1), shape.getUpperBound() + Z3i::Vector(1,1,1), options.h);

  Z3i::Domain domain = digitizer.getDomain();
  trace.endBlock();

  trace.beginBlock( "Construct the Khalimsky space from the image domain." );
  KSpace kspace;
  bool space_ok = kspace.init( domain.lowerBound(),
                 domain.upperBound(), true );
  if (!space_ok)
  {
    trace.error() << "Error in the Khamisky space construction."<<std::endl;
    return;
  }
  trace.endBlock();

  trace.beginBlock( "Extracting boundary by scanning the space. " );

  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  MySurfelAdjacency surfAdj( true ); // interior in all directions.

  typedef KSpace::SurfelSet SurfelSet;
  typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
  typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;
  MySetOfSurfels theSetOfSurfels( kspace, surfAdj );
  Surfaces<KSpace>::sMakeBoundary( theSetOfSurfels.surfelSet(),
                  kspace, digitizer,
                  domain.lowerBound(),
                  domain.upperBound() );
  MyDigitalSurface digSurf( theSetOfSurfels );
  trace.info() << "Digital surface has " << digSurf.size() << " surfels."
         << std::endl;
  trace.endBlock();

  trace.beginBlock( "Making triangulated surface. " );
  typedef CanonicCellEmbedder<KSpace> CanonicCellEmbedder;
  typedef TriangulatedSurface< CanonicCellEmbedder::Value > TriMesh;
  TriMesh trimesh;
  CanonicCellEmbedder canonicCellembedder(kspace);

  MeshHelpers::digitalSurface2TriangulatedSurface
    ( digSurf, canonicCellembedder, trimesh );

  trace.info() << "Triangulated surface is " << trimesh << std::endl;

  trace.endBlock();
  trace.beginBlock("Computing Laplacian");
  typedef TriangulatedSurface< CanonicCellEmbedder::Value > TriangulatedSurface;
  TriangulatedSurface::VertexRange vertices = trimesh.allVertices();

  //Grid scaling factor and sphere projection
  for(auto v : vertices)
  {
    trimesh.position( v ) *= options.h;
    if(options.smooth == 1)
      trimesh.position( v ) /= trimesh.position( v ).norm();
  }

  std::ofstream error_out(options.error_output, std::ofstream::app);
  std::ofstream function_out("function.dat");

  Eigen::VectorXd laplacian_result( trimesh.nbVertices() );
  Eigen::VectorXd error( trimesh.nbVertices() );
  Eigen::VectorXd error_faces( trimesh.nbFaces() );
  int i = 0;
  double total_area = 0.;

  // Iteration over all vertices
  for(auto v : vertices)
  {
    const RealPoint p_i = trimesh.position(v);
    const TriangulatedSurface::ArcRange out_arcs = trimesh.outArcs(v);
    double accum = 0.;

    // We compute here \Delta f(p_i) by iteration over arcs going out from p_i
    for(auto a : out_arcs)
    {
      // The point p_i -----> p_j
      const RealPoint p_j = trimesh.position( trimesh.head(a) );

      const TriangulatedSurface::Arc next_left_arc = trimesh.next( a );
      const TriangulatedSurface::Arc next_right_arc = trimesh.next( trimesh.opposite(a) );

      // Three points of the left triangle
      const RealPoint p1 = p_j;
      const RealPoint p2 = trimesh.position( trimesh.head( next_left_arc ) );
      const RealPoint p3 = p_i;

      // Three points of the right triangle
      const RealPoint pp1 = p_j;
      const RealPoint pp2 = trimesh.position(trimesh.head( next_right_arc ));
      const RealPoint pp3 = p_i;

      // Left and right angles
      const RealPoint v1 =  (p1 - p2)   / (p1 - p2).norm();
      const RealPoint v2 =  (p3 - p2)   / (p3 - p2).norm();
      const RealPoint vv1 = (pp1 - pp2) / (pp1 - pp2).norm();
      const RealPoint vv2 = (pp3 - pp2) / (pp3 - pp2).norm();

      const double dot_1 = dot(v1, v2);
      const double dot_2 = dot(vv1, vv2);

      const double alpha = acos( dot(v1 , v2) );
      const double beta  = acos( dot(vv1, vv2) );

      // Cotan accumulator
      accum += (tan(M_PI_2 - alpha) + tan(M_PI_2 - beta)) * (input_function(p_j) - input_function(p_i));
    }

    double accum_area = 0.;
    const TriangulatedSurface::FaceRange faces_around = trimesh.facesAroundVertex( v );
    for(auto f : faces_around)
    {
      const TriangulatedSurface::VertexRange vr = trimesh.verticesAroundFace(f);
      RealPoint p = trimesh.position(vr[0]);
      RealPoint q = trimesh.position(vr[1]);
      RealPoint r = trimesh.position(vr[2]);

      const RealPoint cross = crossProduct(r - p, r - q);
      const double faceArea = .5 * cross.norm();

      accum_area += faceArea / 3.;
    }

    total_area += accum_area;

      (options.smooth == 1)
          ? laplacian_result(i) = (1 / (2. * accum_area)) * accum
          : laplacian_result(i) = .5 * accum;

    const RealPoint w_projected = p_i / p_i.norm();
    const double real_laplacian_value = result_function(w_projected);

    const RealPoint w_s = cartesian_to_spherical(w_projected);

    function_out << w_s[1] << " "
        << w_s[2] << " "
        << laplacian_result(i) << " "
        << real_laplacian_value << " "
        << input_function(p_i) << std::endl;

    error(i) = laplacian_result(i) - real_laplacian_value;
    for(auto f : faces_around)
        error_faces( f ) += error(i) / faces_around.size();

    i++;
  }

  int max_index;

  trace.info() << "Computed Area / Real Area : " << total_area << " " << 4 * M_PI << std::endl;
  trace.info() << "Mean Error / Max Error : "
               << error.array().abs().mean() << " / " << error.array().abs().maxCoeff(&max_index) << std::endl;
  error_out << options.h << " "
            << error.array().abs().mean() << " "
            << error.array().abs().maxCoeff() << std::endl;

  trace.endBlock();

#ifdef WITH_3D
  typedef Mesh< CanonicCellEmbedder::Value > ViewMesh;
  ViewMesh viewmesh;
  MeshHelpers::triangulatedSurface2Mesh( trimesh, viewmesh );
  trace.info() << "Mesh has " << viewmesh.nbVertex()
         << " vertices and " << viewmesh.nbFaces() << " faces." << std::endl;

  DGtal::ColorBrightnessColorMap<float> colormap_error(error_faces.minCoeff(), error_faces.maxCoeff(), DGtal::Color::Red);
  for(int k = 0; k < viewmesh.nbFaces(); k++)
    viewmesh.setFaceColor(k, colormap_error( error_faces(k) ));

  QApplication application(argc,argv);
  Viewer3D<> viewer;
  viewer.show();
    viewer << viewmesh;
  viewer << Viewer3D<>::updateDisplay;
  application.exec();
#endif

  trace.endBlock();
}

int main(int argc, char **argv)
{
  Options options = parse_options(argc, argv);

  typedef Ball3D<Z3i::Space> Ball;
  Ball ball(Point(0.0,0.0,0.0), 1.0);

  std::function<double(const RealPoint&)> xx_function = [](const RealPoint& p) {return p[0] * p[0];};
  std::function<double(const RealPoint&)> xx_result = [](const RealPoint& p)
  {
    const RealPoint p_s = cartesian_to_spherical(p);
    return 2 * cos(p_s[1]) * cos(p_s[1]) * (2 * cos(p_s[2]) * cos(p_s[2]) - sin(p_s[2]) * sin(p_s[2]))
            + 2 * (sin(p_s[1]) * sin(p_s[1]) - cos(p_s[1]) * cos(p_s[1]));
  };

  std::function<double(const RealPoint&)> cos_function = [](const RealPoint& p) {return p[2];};
  std::function<double(const RealPoint&)> cos_result = [](const RealPoint& p)
  {
    const RealPoint p_s = cartesian_to_spherical(p);
    return - 2 * cos(p_s[2]);
  };

  std::function<double(const RealPoint&)> exp_function = [](const RealPoint& p)
  {
    const RealPoint p_sphere = p / p.norm();
    return exp(p_sphere[0]);
  };
  std::function<double(const RealPoint&)> exp_result   = [](const RealPoint& p)
  {
    const RealPoint p_sphere = p / p.norm();
    const RealPoint p_s = cartesian_to_spherical(p);

    if(p_s[1] == 0 && p_s[2] == 0) return 1.;

    const double theta_derivative = (sin(p_s[1]) * sin(p_s[1]) * sin(p_s[2])
                      - cos(p_s[1])) * (1 / sin(p_s[2])) * exp(p_sphere[0]);
    const double phi_derivative = ( cos(p_s[1]) * cos(p_s[2]) * cos(p_s[2])
                      - sin(p_s[2])
                      + cos(p_s[2]) * cos(p_s[2]) / sin(p_s[2])) * cos(p_s[1]) * exp(p_sphere[0]);

    return theta_derivative + phi_derivative;
  };

  std::function<double(const RealPoint&)> input_function
      = ( options.function == 0 ) ? xx_function : ( (options.function == 1) ? cos_function : exp_function );
  std::function<double(const RealPoint&)> result_function
      = ( options.function == 0 ) ? xx_result : ( (options.function == 1) ? cos_result : exp_result );

  laplacian<Ball>(ball, options, input_function, result_function, argc, argv);

  return 0;
}

//! [TriangulatedSurfaceUsageFull]
