
# DGtal 0.9.1

## New Features / Critical Changes

- *Configuration/General*
 - DGtal core now compiles on Microsoft Windows, Visual Studio. Many
   issues have been fixed for compatibility with 'cl' compiler. (David
   Coeurjolly, [XXX](XXXX))

- *Geometry Package*
 - Hull2DHelpers: implementation of the rotating caliper algorithm to compute
   the width (vertical/horizontal or Euclidean) of a convex hull.
   (Bertrand Kerautret, [#1052](https://github.com/DGtal-team/DGtal/pull/1052))

## Changes

- *Base*
 - Deprecated OldAlias, OldClone, OldConstAlias have been removed. (David
   Coeurjolly, [XXX](XXXX))

- *IO*
 - Minor improvements of default settings in Viewer3D. (David
   Coeurjolly, [#1066](https://github.com/DGtal-team/DGtal/pull/1066))

## Bug Fixes

- *Configuration/General*
 - catch unit test framework upgraded to the develop version. (David
 Coeurjolly, [#1055](https://github.com/DGtal-team/DGtal/pull/1055))
 - Fixing boost include path issue when building tools using DGtal and
   its cmake DGtalConfig.cmake. (David Coeurjolly,
   [#1059](https://github.com/DGtal-team/DGtal/pull/1059))
 



# DGtal 0.9

## New Features / Critical Changes

- *Configuration/General*
 - Continuous integration enabled on both linux and macosx
   systems. Furthermore, the nightly build documentation is
   automatically deployed.  (David Coeurjolly,
   [#955](https://github.com/DGtal-team/DGtal/pull/955))
 - New unit test framework based on
   [catch](https://github.com/philsquared/Catch). Catch allows to
   design quick and efficient unit tests with nice trace
   outputs. (David Coeurjolly,
   [#1019](https://github.com/DGtal-team/DGtal/pull/1019))
 - Documentation added for Catch. (David Coeurjolly,
   [#1042](https://github.com/DGtal-team/DGtal/pull/1042))


- *Kernel*
 - New template class DigitalSetlByAssociativeContainer allows to
   define digital sets from any associative container of the STL. For
   instance, using std::unordered_set (c++11) or boost::unordered_set (hash
   function based containers), speed-up up to 40% can be measured when
   processing digital sets. (David Coeurjolly,
   [#1023](https://github.com/DGtal-team/DGtal/pull/1023)
 - By default, Z2i::DigitalSet, Z3i::DigitalSet and digital set from
   DigitalSetSelector use the new hash function based
   container. (David Coeurjolly,
   [#1023](https://github.com/DGtal-team/DGtal/pull/1023)
 - Specializations of std::hash (c++11) and boost::hash to define a hash
   functions on DGtal points. (David Coeurjolly,
   [#1023](https://github.com/DGtal-team/DGtal/pull/1023)
 
## Changes

- *DEC Package*
 - Coherent signed cells support allows lower dimension manifold embedding.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - OppositeDuality struct allows generic hodge and laplace definition.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - Easy k-form and vector field transversal using .length() and .getSCell().
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - Unified operators interface :
   .hodge<order, duality>() replace primalHodge<order>() and dualHodge<order>(),
   .laplace<duality>() replace primalLaplace() and dualLaplace().
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - New antiderivative<order, duality>() operator.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - New flatDirectional<duality, direction>() and sharpDirectional<duality,
   direction>() operators defined as flat(vector_field_along_direction) and
   sharp(1-form).extractZeroForm(direction). (Pierre Gueth
   [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - DiscreteExteriorCalculus<dim_embedded, dim_ambient, Backend>
   takes 2 dimension template parameters for embedding
   manifold in ambient euclidean space.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - Basic openmp support for derivative computation.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - New propagation example and extended embedding tests.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - Improved operator generation using new CSparseMatrix concepts.
   (Pierre Gueth [#1007](https://github.com/DGtal-team/DGtal/pull/1007))
 - DEC constructors are replaced by static factory functions:
   DiscreteExteriorCalculusFactory::createFromDigitalSet and
   DiscreteExteriorCalculusFactory::createFromNSCells.
   (Pierre Gueth [#1008](https://github.com/DGtal-team/DGtal/pull/1008))
 - Mutable iterator on DiscreteExteriorCalculus.
   (Pierre Gueth [#1008](https://github.com/DGtal-team/DGtal/pull/1008))
 - Unary minus operators for k-forms, vector fields and linear operators.
   (Pierre Gueth [#1020](https://github.com/DGtal-team/DGtal/pull/1020))
 - Introduction of .updateIndexes() that needs to be called after any
   call to .insertSCell() or .eraseCell().
   (Pierre Gueth [#1020](https://github.com/DGtal-team/DGtal/pull/1020))
 - Transpose of linear operators.
   (Pierre Gueth [#1020](https://github.com/DGtal-team/DGtal/pull/1020))
 - Intensity operator on vector fields.
   (Pierre Gueth [#1020](https://github.com/DGtal-team/DGtal/pull/1020))
 - Reorder operators to remap indexes.
   (Pierre Gueth [#1020](https://github.com/DGtal-team/DGtal/pull/1020))

- *Geometry Package*
 - New EstimatorCache class to cache quantities estimated by a
   surfel local estimator. (David Coeurjolly,
   [#927](https://github.com/DGtal-team/DGtal/pull/927))
 - New digital surface local estimator that computes a sphere
  fitting. It requires to have the Patate library installed (and
  WITH_PATATE=true): http://patate.gforge.inria.fr/html/. See
  SphereFittingEstimator (David Coeurjolly,
  [#929](https://github.com/DGtal-team/DGtal/pull/929))
 - Algorithm to compute the union of two DSSs in logarithmic time
	(Isabelle Sivignon,
	[#949](https://github.com/DGtal-team/DGtal/pull/949))
 - InexactPredicateLpSeparableMetric class is now templated by an
   EuclideanRing type. (David Coeurjolly,
   [#1017](https://github.com/DGtal-team/DGtal/pull/1017))
 - Main example files of geometry/curves are introduced in the list of examples
   and briefly described.
   (Tristan Roussillon, [#1026](https://github.com/DGtal-team/DGtal/pull/1026))
 - New algorithms to compute the convex hull of planar point sets.
   (Tristan Roussillon, [#1028](https://github.com/DGtal-team/DGtal/pull/1028))
 - Lambda maximal segment tangent direction estimator 2D/3D: LambdaMST2D, LambdaMST3D.
   A fast tangent direction estimator which uses maximal digital straight segments. 
   (Kacper Pluta, [#1021](https://github.com/DGtal-team/DGtal/pull/1021))
 - Segmentation of 3D digital curves by a combination of the segmentations of its 2D 
   projections onto 2D base planes: XY, XZ, YZ. Notice that, only valid projections
   are used. By valid one understands that there are no two 3D points which are projected
   onto the same 2D point. A segment is computed as long as is extendable and at least 
   two projections are valid.
 : NaiveDSS3DComputer.
   (Kacper Pluta, [#1021](https://github.com/DGtal-team/DGtal/pull/1021))

- *Math Package*
 - Utilities added (OrderedLinearRegression) to perform sequential
   linear model estimation of scalar data. (David Coeurjolly, Jérémy
   Levallois [#935](https://github.com/DGtal-team/DGtal/pull/935),
   backport from imagene)
 - New linear algebra concepts: CDenseVector, CDenseMatrix, CSparseMatrix.
   (Pierre Gueth [#1007](https://github.com/DGtal-team/DGtal/pull/1007))

- *Image Package*
 - Adding copy between images of different types. (Roland Denis [#1001]
   (https://github.com/DGtal-team/DGtal/pull/1001))

- *IO Package*
 - Fix RawWriter and RawReader. Added templated generic RawReader::importRaw
   and RawWriter::exportRaw.
   (Pierre Gueth [#1010](https://github.com/DGtal-team/DGtal/pull/1010))
 - New 2D DEC board style with orientated cells.
   (Pierre Gueth [#977](https://github.com/DGtal-team/DGtal/pull/977))
 - Limited interaction added to QGLViewer Viewer3D class. The user
   may assign integer identifiers (OpenGL names) to surfels and
   callback functions, which are called when surfels are
   selected. (Jacques-Olivier Lachaud
   [#942](https://github.com/DGtal-team/DGtal/pull/942))
 - Balls can be exported to OBJ in Board3D and ball resolution can now
   be specified in Viewer3D and Board3D (David Coeurjolly,
   [#945](https://github.com/DGtal-team/DGtal/pull/945))
 - Viewer3d cleanings with better organisation through the
   separation of all code generating the GL lists. (Bertrand Kerautret)
  ([#945](https://github.com/DGtal-team/DGtal/pull/945))
 - Operators added to perform computations on Color objects (addition,
   substraction scaling...). Color is now CopyConstructible and
   Assignable (David Coeurjolly
   [#940](https://github.com/DGtal-team/DGtal/pull/940))
 - Improvement of memory footprint of DGtal::Color (David Coeurjolly,
   [#961](https://github.com/DGtal-team/DGtal/pull/961))
 - New colormap adapter to add ticks/iso-contours (regularly spaced or
   specified by the user) to a given colormap. (David Coeurjolly,
   [#987](https://github.com/DGtal-team/DGtal/pull/987))
 - New flag (-DWITH_QT5) enables QT5 support in libqglviewer. (Nicolas
   Aubry, [#983](https://github.com/DGtal-team/DGtal/pull/983))
 - Board2D now supports quadratic Bezier curve drawing. (Tristan Roussillon,
   [#1002](https://github.com/DGtal-team/DGtal/pull/1002))
 - MeshWriter class can now export OBJ file including colors.
   (Bertrand Kerautret, [#1016](https://github.com/DGtal-team/DGtal/pull/1016))
 - Viewer3D: Shift-L / L key binding added to save and restore camera settings.
   (Bertrand Kerautret, [#1024](https://github.com/DGtal-team/DGtal/pull/1024))
 - Viewer3D:  change the chronological order to diplay primitives (in the draw
   function) in order to see see textured image primitives through the
   transparency of other 3D primitives. (Bertrand Kerautret,
   [#1041](https://github.com/DGtal-team/DGtal/pull/1041))


- *Kernel Package*
- HyperRectDomain can now be empty (lowerBound == upperBound + diagonal(1)).
    Warning about the use of lexicographical order in comparison operators of
    PointVector. (Roland Denis,
    [#996](https://github.com/DGtal-team/DGtal/pull/996))
  - Adds generic linearization (point to index) and reverse process (index to
    point), specialized for HyperRectDomain. (Roland Denis,
    [#1039](https://github.com/DGtal-team/DGtal/pull/1039))
 - HyperRectDomain can now be empty (lowerBound == upperBound +
    diagonal(1)). Warning about the use of lexicographical order in
    comparison operators of PointVector. (Roland Denis,
    [#996](https://github.com/DGtal-team/DGtal/pull/

- *Shapes Package*
 - Adds a vertex Iterator in the Mesh class in addition to the
   ConstIterator and adds a new method to change the color of a
   specific face. (Bertrand Kerautret,
   [#937](https://github.com/DGtal-team/DGtal/pull/937))
 - New methods to generate basic 3D tubular meshes and height
   fields. New mesh module documentation added. (Bertrand Kerautret,
   [#969](https://github.com/DGtal-team/DGtal/pull/969))
 - Refactoring of CSG operations on Euclidean / Digital shapes to easily
   combine several operations.
   EuclideanShapesUnion, EuclideanShapesIntersection and
   EuclideanShapesMinus are now deprecated. Use EuclideanShapesCSG
   instead.
   DigitalShapesUnion, DigitalShapesIntersection and
   DigitalShapesMinus are now deprecated. Use DigitalShapesCSG
   instead. (Jérémy Levallois
   [#962](https://github.com/DGtal-team/DGtal/pull/962))
 - Add various methods in the Mesh class to get the bounding box, to
   change the mesh scale or to subdivide triangular faces. (Bertrand
   Kerautret, [#990](https://github.com/DGtal-team/DGtal/pull/990) and
   [#992](https://github.com/DGtal-team/DGtal/pull/992))
 - New copy constructor and copy operator on Mesh object (and
   documentation added about vertex ordering for obj format).
   (Bertrand Kerautret,
   [#976](https://github.com/DGtal-team/DGtal/pull/976))

- *Arithmetic Package*
 - Algorithm to compute the fraction of smallest denominator in
	between two irreducible fractions (Isabelle Sivignon
	[#949](https://github.com/DGtal-team/DGtal/pull/949))

## Bug Fixes

- *Configuration* 
  - Removing code coverage with coverall.io (David Coeurjolly,
  [1040](https://github.com/DGtal-team/DGtal/pull/1032)).
  - Forces Eigen 3.2.1 minimum (for a bug fix).  (Jacques-Olivier
    Lachaud, [1032](https://github.com/DGtal-team/DGtal/pull/1032)).
  - Fix issue #925, detection of Eigen3 (3.1 minimum) and also issue
    #924, DGtal configuration file when using Eigen3.  (Jacques-Olivier
    Lachaud, [#926](https://github.com/DGtal-team/DGtal/pull/926))
 - Backport of changes in google/benchmarck API for micro-benchmarking
   (David Coeurjolly, [#1014](https://github.com/DGtal-team/DGtal/pull/1014))
 - New travis configuration file to enable new travis Docker based
   container system (David Coeurjolly,
   [#1030](https://github.com/DGtal-team/DGtal/pull/1030))
 - Various fixes of compiler warnings due to unused paramters (David
   Coeurjolly, Roland Denis,
   [#1034](https://github.com/DGtal-team/DGtal/pull/1030))


- *Base Package*
 - Fix bug with LabelledMap copy constructor and copy iterator. (Roland
   Denis, [#973](https://github.com/DGtal-team/DGtal/pull/973))
 - Fix bug with Labels iterator when first index is set (Roland Denis,
 [#972](https://github.com/DGtal-team/DGtal/pull/972))
 - Iterator category fix for boost > 1.57 (David Coeurjolly,
 [#938](https://github.com/DGtal-team/DGtal/pull/938))
 - Cleanup of DGtal namespaces (David Coeurjolly,
 [#993](https://github.com/DGtal-team/DGtal/pull/993))


- *Geometry Package*
 - Fix bug occuring in the computation of the Faithful Polygon (class FP)
   in the closed case, ie. with circulators.
   (Tristan Roussillon, [#939](https://github.com/DGtal-team/DGtal/pull/939))
 - Fixing DSS based length estimator on open curves. (David
   Coeurjolly, [#941](https://github.com/DGtal-team/DGtal/pull/941))
 - Fix bug of method ArithmeticalDSL::getPoint with negative values
   of positions as input arguments.
   (Tristan Roussillon, [#944](https://github.com/DGtal-team/DGtal/pull/944))
 - Fix too restrictive asserts of methods
	ArithmeticalDSSConvexHull::smartCH and
	ArithmeticalDSSConvexHull::smartCHNextVertex to enable negative
	positions as input arguments. (Isabelle Sivignon,
	[#950](https://github.com/DGtal-team/DGtal/pull/950))
 - Fix Bezout Vector computation (Isabelle Sivignon,
 [#948](https://github.com/DGtal-team/DGtal/pull/948))
 - Fix issues with SphereFitting and TensorVoting local estimators on
   digital surfaces (Jérémy Levallois, David Coeurjolly
   [#970](https://github.com/DGtal-team/DGtal/pull/970))

- *IO Package*
 - Performance improvement of color managment in Display3D, Board3D
   and Viewer3D: no more "createNew...List" when setting a new
   color. (David Coeurjolly,
   [#958](https://github.com/DGtal-team/DGtal/pull/958))
 - Radius and resolution of balls have been fixed when used to
   represent a 3D point in grid mode (David Coeurjolly,
   [#978](https://github.com/DGtal-team/DGtal/pull/978))
 - Change in the mesh export in OFF format: now it tries by default to export
   colors (if stored). (Bertrand Kerautret,
   [#985](https://github.com/DGtal-team/DGtal/pull/985))
 - Bugfix in quad visualization in BoardD3D and Viewer3D (David
   Coeurjolly, [#980](https://github.com/DGtal-team/DGtal/pull/980))
 - Fix warnings message of std::abs in Display3D.    (Bertrand Kerautret,
   [#991](https://github.com/DGtal-team/DGtal/pull/991))
 - Fix memory leaks present in the Viewer3d.  (Bertrand Kerautret,
   [#995](https://github.com/DGtal-team/DGtal/pull/995))
 - Fix issues in OBJ color export when exporting voxels. (David
   Coeurjolly, [#1022](https://github.com/DGtal-team/DGtal/pull/1022))
 - Fix compilation issue on gentoo system related to MeshWriter
   (gcc version 4.9.2-r2). (Van Tho Nguyen,
   [#1035](https://github.com/DGtal-team/DGtal/pull/1035))
 - Fix deprecated usage of setMouseBindingDescription with QGLViewer >= 2.5.0.
   (Roland Denis, [#1036](https://github.com/DGtal-team/DGtal/pull/1036))

- *Kernel Package*
  - BasicDomainSubSampler can now handle non 0 origin point. This update also
    correct the search of point which are outside the source domain (it is now
    checked in testBasicPointFunctors). (Bertrand Kerautret,
    [989](https://github.com/DGtal-team/DGtal/pull/989)).

- *Topology  Package*
  - Fix loop bug in extractAllConnectedSCell of Surfaces from helpers.
    (Bertrand Kerautret, [994](https://github.com/DGtal-team/DGtal/pull/994)).

- *DEC  Package*
  - Fix missing include in testEigenSolver.
    (Jacques-Olivier Lachaud,
    [1032](https://github.com/DGtal-team/DGtal/pull/1032)).


# DGtal 0.8


## New Features / Critical Changes

- *General*
 - This Changelog has been ported to MarkDown (David Coeurjolly,
   [#846](https://github.com/DGtal-team/DGtal/pull/846))
 - The DGtal main website is now http://dgtal.org

 - Global refactoring of base functors (David Coeurjolly,
   [#861](https://github.com/DGtal-team/DGtal/pull/861))
    - BasicFunctor functors have been moved to functors:: namespace.
    - DefaultFunctor has been renamed functors::Identity.
    - xxxFunctor have been renamed to xxx.

 - Moving graph, topology, geometry/estimation concepts into
   namespace concepts::, also moving some functors into namespace
   functors:: (Jacques-Olivier Lachaud,
   [#912](https://github.com/DGtal-team/DGtal/pull/912)).

- *DEC Package*
 - DGtal 0.8 contains the first release of the Discrete Exterior
   Calculus Package. DEC provides an easy and efficient way to
   describe linear operator over various structure. Basic operators,
   such as Hodge duality operator or exterior derivative, can be
   combined to create classical vector analysis operator such as
   gradient, curl and divergence. (Pierre Gueth,
   [#877](https://github.com/DGtal-team/DGtal/pull/877))


- *Geometry Package*
 - Add digital nD Voronoi Covariance Measure support, as well as
  digital geometric estimators based on it. Add tests and examples of
  feature detection with VCM. (Jacques-Olivier Lachaud,
  [#803](https://github.com/DGtal-team/DGtal/pull/803))

 - Add Integral Invariant estimators so that they meet the concept of
  surface local estimator. Add geometric functors to define easily all
  the geometric estimators that can be built from the volume and
  coariance matrix. Previous estimators (IntegralInvariantMeanCurvatureEstimator
  and IntegralInvariantGaussianCurvatureEstimator) are removed. Please use
  the new ones instead. (Jeremy Levallois, Jacques-Olivier Lachaud,
  [#803](https://github.com/DGtal-team/DGtal/pull/803)
  [#856](https://github.com/DGtal-team/DGtal/pull/856)
  [#893](https://github.com/DGtal-team/DGtal/pull/893))

 - Various geometric predicates are now available in order to test the
  orientation of three points in the planes. Most classes are template
  classes parametrized by a type for the points (or its coordinates)
  and an integral type for the computations. They always return an
  exact value (or sign), provided that the integral type used for the
  computations is well chosen with respect to the coordinates of the
  points. Some implementations do not increase the size of the input
  integers during the computations. (Tristan Roussillon,
  [#755](https://github.com/DGtal-team/DGtal/pull/755))

 - Logarithmic construction of an arithmetical DSS of minimal
   parameters from a bounding DSL and two end points (ctor of
   ArithmeticalDSS) (Tristan Roussillon,
   [#819](https://github.com/DGtal-team/DGtal/pull/819))

 - Proof-of-concept that path-based norms can be implemented in a
   separable approach using logarithmic cost predicates
   (experimental::ChamferNorm2D). (David Coeurjolly,
   [#898](https://github.com/DGtal-team/DGtal/pull/898))

 - Logarithmic construction of an arithmetical DSS of minimal
   parameters from a bounding DSS (of known leaning points)
   and two end points (ctor of
    ArithmeticalDSS) (Tristan Roussillon,
    [#914](https://github.com/DGtal-team/DGtal/pull/914))

 - Feature extraction algorithm from Tensor Voting.(Jérémy Levallois,
   David Coeurjolly,
   [#895](https://github.com/DGtal-team/DGtal/pull/895))

 - Ray shooting intersection predicates (ray-triangle, ray-quad,
   ray-surfel) added in geometry/tools (David Coeurjolly,
   [#904](https://github.com/DGtal-team/DGtal/pull/904))


- *IO Package*
  - Now VolReader/VolWriter and LongvolReader/LongvolWriter support the
   usage of Center-(X,Y,Z) parameters, as described in Vol file
   specification. (Jérémy Levallois,
   [#879](https://github.com/DGtal-team/DGtal/pull/879))

- *Math Package*

    - New classes to compute nD eigen decomposition of symmetric
      matrix (class EigenDecomposition).  Add tests. (Jacques-Olivier
      Lachaud, #803)
    - Simple Linear Regression tool added (backport from
      imagene). (David
      Coeurjolly, [#794](https://github.com/DGtal-team/DGtal/pull/794))

- *Kernel package*
  - BasicPointFunctors functors have been moved in the functors::
    namespace (David Coeurjolly,
    [#863](https://github.com/DGtal-team/DGtal/pull/863))

- *For developpers*
     - Google Benchmark can be enabled to allow micro-benchmarking in
         some DGtal unit tests (https://github.com/google/benchmark)
         (David Coeurjolly,
         [#790](https://github.com/DGtal-team/DGtal/pull/790))

- *Images*
   - Classes to perform rigid transformations of 2D and 3D images
     (Kacper Pluta,
     [#869](https://github.com/DGtal-team/DGtal/pull/869))

## Changes

- *Base Package*
 - Add comparison operators in variants of CountedPtr. Improve
   coverage of these classes and fix compilation problem
   (Jacques-Olivier Lachaud)
 - Update doc of CountedPtr, CountedPtrOrPtr and
   CountedConstPtrOrConstPtr. Add asserts. Add tests. Fix issue 773
   (https://github.com/DGtal-team/DGtal/issues/773). (Jacques-Olivier
   Lachaud, [#894](https://github.com/DGtal-team/DGtal/pull/894)).
 - XXXOutputRangeYYY classes are now called
   XXXRangeWithWritableIteratorYYY (Tristan Roussillon,
   [#850](https://github.com/DGtal-team/DGtal/pull/850)).

- *Geometry Package*
 - Fix and add concept of CSurfelLocalEstimator and related ground
  truth estimators for implicit polynomial shapes
  (TrueDigitalSurfaceLocalEstimator). (Jacques-Olivier Lachaud,
  [#803](https://github.com/DGtal-team/DGtal/pull/803))
 - Random-access iterators added in ArithmeticalDSL. (Tristan
   Roussillon, [#801](https://github.com/DGtal-team/DGtal/pull/801))
 - Updates in Metric concepts: better and simpler concept structure
   and a new adapter to adapt any euclidean metric to a digital one
   (with values on Z) (David Coeurjolly,
   [#870](https://github.com/DGtal-team/DGtal/pull/870)
 - CubicalSudivision has been renamed SpatialCubicalSubdivision and
   moved to "geometry/tools" (David Coeurjolly,
   [#862](https://github.com/DGtal-team/DGtal/pull/862))

- *IO Package*
  - Better handling of materials in Board3D and OBJ exports. (David
    Coeurjolly,
    [#784](https://github.com/DGtal-team/DGtal/pull/784))
  - New 'basic' display mode for surfels (oriented or not), useful for
    large digital surface displays (quads instead of 3D prism)
    (Bertrand Kerautret,
    [#783](https://github.com/DGtal-team/DGtal/pull/783))
  - New clear() method to subclasses of Display3D (Viewer3D and
    Board3D) to clear the current drawning buffer. (Kacper Pluta,
    [#807](https://github.com/DGtal-team/DGtal/pull/807))
  - New draw() method for 3D display models (Viewer3D and Board3D) to
    display surfels with prescribed normal vectors (David Coeurjolly,
    [#802](https://github.com/DGtal-team/DGtal/pull/802)).
  - When exporting an 3D visualization to OBJ, a new option will
    rescale the geometry to fit in [-1/2,1/2]^3. (David Coeurjolly,
    [#820](https://github.com/DGtal-team/DGtal/pull/820))
  - New raw import/export for 32 bits images (Bertrand Kerautret,
	[#877](https://github.com/DGtal-team/DGtal/pull/876))

- *Kernel Package*

  - New functor DomainSubSampler allowing to apply different
    samplings with larger or smaller size on N dimensional domain. New tests
    and examples are given for 2D and 3D images (Bertrand Kerautret,
    [825](https://github.com/DGtal-team/DGtal/pull/825) and
    [882](https://github.com/DGtal-team/DGtal/pull/882)).

- *Shapes Package*
  - Shape concepts have been moved to concepts:: namespace (David
  Coeurjolly, [#871](https://github.com/DGtal-team/DGtal/pull/871))

- *Topology Package*
  - Surfaces::findABell accepts now arbitrary pair of points (Jacques-Olivier
    Lachaud, David Coeurjolly,
    [#851](https://github.com/DGtal-team/DGtal/pull/851))



## Bug Fixes


- *Base Package*

  - Fixing issue on Circulator/IteratorFunctions (related to #770 on
    MacOS).

- *Kernel Package*
  - BinaryPointPredicate is now specialized for DGtal::AndBoolFct2 and
    DGtal::OrBoolFct2 in order to guarantee that the second computation
    is not performed when the first point predicate return false (resp. true)
    with DGtal::AndBoolFct2 (resp. DGtal::OrBoolFct2) (Tristan Roussillon
    [#852](https://github.com/DGtal-team/DGtal/pull/852)).

- *Geometry Package*
  - Bug fix in PowerMap construction. (David Coeurjolly,
    [#814](https://github.com/DGtal-team/DGtal/pull/814))
  - Bug fix in 3d display of StandardDSS6Computer (Tristan Roussillon
    [#854](https://github.com/DGtal-team/DGtal/pull/854))

- *Topology Package*
  - small fix in ImplicitDigitalSurface. (Jacques-Olivier Lachaud,
    [#803](https://github.com/DGtal-team/DGtal/pull/803))
  - fix examples volTrackBoundary and volScanBoundary for DEBUG mode
    (Jacques-Olivier Lachaud, David Coeurjolly,
    [#851](https://github.com/DGtal-team/DGtal/pull/851))
  - New methods to fill the interior/exterior of digital contours
    (in the Surface class of topology/helpers).  (Bertrand Kerautret
    [#827](https://github.com/DGtal-team/DGtal/pull/827))


- *Graph Package*
  - fix examples volDistanceTraversal for DEBUG mode (Jacques-Olivier Lachaud,
    David Coeurjolly, [#851](https://github.com/DGtal-team/DGtal/pull/851))

- *Image Package*
  - Fixing template types in ImageAdapter (David Coeurjolly,
    [#835](https://github.com/DGtal-team/DGtal/pull/835))
  - Fixing image thresholders (SimpleThresholdForegroundPredicate and
    IntervalForegroundPredicate) which require CConstImage instead of
    CImage (David Coeurjolly,
    [#843](https://github.com/DGtal-team/DGtal/pull/843))

- *IO*
  - Bug fix for reading PGM(P2) 3D. (Kacper Pluta,
   [#853](https://github.com/DGtal-team/DGtal/pull/853))
  - Renaming BasicColorToScalarFunctors namespace to functors:: (David
    Coeurjolly,  [#857](https://github.com/DGtal-team/DGtal/pull/857))
  - Fix OpenGL warnings by redefining openGL primitive (glSphere) (Bertrand
    Kerautret [#981](https://github.com/DGtal-team/DGtal/pull/891))

=== DGtal 0.7 ===

*General*

    - Unit tests build is now disabled by default (to turn it on, run cmake with "-DBUILD_TESTING=on")

    - The "boost program option library" dependency was removed.

    - DGtal needs boost >= 1.46.

    - Thanks to new compiler warning option (-Wdocumentation), the doxygen documentation has been considerably improved.

*Base Package*

    - Complete rewriting of Clone, Alias and ConstAlias
      classes. Parameter passing is now documented with a standardized
      method to determine parameters unambiguously. Associated classed
      CowPtr, CountedPtrOrPtr and CountedConstPtrOrConstPtr are now used
      in conjunction with the previous classes.

    - Few improvments in Clock and Trace base classes.

*Kernel Package*

    - Two initialisation methods (initRemoveOneDim and initAddOneDim)
      for the Projector Functor from the BasicPointFunctors class in
      order to simplify the slice images (with example and test in 2D
      slice image extraction from 3D volume file).

    - New basic functors:
	- SliceRotator2D: to rotate 2D Slice images from 3D volume.
	- Point2DEmbedderIn3D: a simple functor to embed in 3d a 2d points
	  (useful to extract 2D image from 3D volume).

    - Sets have been updated to own their domain with a copy-on-write pointer,
      in order to avoid some inconsistencies.

*Topology Package*

    - Fixing bugs in Object::isSimple for some digital
      topologies. Speed of Object::isSimple has been improved. Homotopic
      thinning is much faster (even without a precomputed simplicity
      table).

    - Objects have been updated to use Clone services.

*Geometry Package*

    - New classes to deal with arithmetical digital straight segments.
      Now the representation of the primitives and their recognition
      along a discrete structure are separated. The unique class
      ArithmeticalDSS,  which was a segment computer, has been replaced by
      mainly three classes: ArithmeticalDSL, ArithmeticalDSS and
      ArithmeticalDSSComputer. This is described in a doc page of the geometry
      package. Note that Backward/Forward suffixes have been renamed into
      Back/Front. Moreover, get prefixes for data members accessors have been
      removed.

    - Generic adapter to transform a metric (model of CMetric) with
      monotonic (see doc) properties to a separable metric (model of
      CSeparableMetric) which can be used in
      VoronoiMap/DistanceTransformation algorithms.

    - New possibility to access the 3 2D ArithmeticDSS object within an
      ArithmeticDSS3d.

    - New local estimator adapter to make easy implementation of locally defined
      differential estimator on digital surfaces.

    - New documentation on local estimators from digital surface
      patches and surfel functors. New normal vector estimator from
      weighted sum of elementary surfel normal vectors added.

    - With an optional binding with CGAL and Eigen3, new curvature and
      normal vector estimators have been added. For instance, you can
      now estimate curvature from polynomial surface fitting (Jet
      Fitting) and Monge forms.

    - Minor improvements in the spherical accumulator.

    - Improvement of integral invariant estimators (better memory footprint,
	...).
      They also allow to estimate principal curvatures using Covariance matrix.
      Covariance matrix is also "masks" based, so the computation is efficient.

    - New algorithms to compute the minimal characteristics of a
      Digital Straight Line subsegment in logarithmic time using local
      convex hulls or Farey Fan. Also works when the DSL
      characteristics are not integers.

    - Chord algorithm for (naive) plane recognition and width computation.

    - New organization for computing primitives. Introduction of the concept
      of PrimitiveComputer and specialization. COBA algorithm and Chord
      algorithm are now models of AdditivePrimitiveComputer.

    - Introduction of the primitive ParallelStrip, computed by COBA and Chord
      algorithms

    - New documentation for planarity decision, plane recognition and width
      computation.
      Quantitative and qualitative evaluation of COBA and Chord algorithm.

    - Bug fix in COBA algorithm when extending an empty computer with a group of
      points.

    - add standard plane recognition with adapter classes both for COBA and
      Chord algorithm.

*Shape Package*

    - The class MeshFromPoints was transformed into Mesh (more from
      shapes/fromPoints to shapes/ directory), iterators on mesh
      points and mesh face.

*Topology Package*

    - The class SCellToMidPoint is now deprecated. Use CanonicSCellEmbedder
      instead to map a signed cell to its corresponding point in the Euclidean
      space

*IO Package*

    - Complete refactoring of 3D viewers and boards (Viewer3D, Board3DTo2D).
    - New Board3D to export 3D displays to OBJ 3D vector format.
    - A new display of 2D and 3D image in Viewer3D.
    - New reader: HDF5 file with 2D image dataset(s) (8-bit with palette and
      24-bit truecolor with INTERLACE_PIXEL).
    - New GenericReader and Generic Writer for both 2D, 3D and ND images.
    - Adding a Table Reader to extract objets given in a specific column from a
      text file.
    - Adding missing PPM Reader.
    - Adding missing DICOM reader (with ITK library)
    - Adding ITK reader and ITK writer
    - OpenInventor (SOQT/Coin3D) based viewer has been removed (please consider
      release <=0.6 if interested).

*Image Package*

    - New concepts : CImageFactory to define the concept describing an
      image factory and CImageCacheReadPolicy/CImageCacheWritePolicy
      to define the concept describing cache read/write policies.

    - New classes : ImageFactoryFromImage to implement a factory to
      produce images from a "bigger/original" one according to a given
      domain, ImageCache to implement an images cache with 'read and
      write' policies, TiledImageFromImage to implement a tiled image
      from a "bigger/original" one.

    - ImageContainerByITKImage complies with CImage.
      The container has been moved from the DGtal::experimental namespace to
      the main DGtal namespace.

*Graph Package*

    - New graph visitor, which allows to visit a graph according to
      any distance object (like the Euclidean distance to some point).

*Math Package*

    - add Histogram class and CBinner concept.
    - add math concepts diagram.


=== DGtal 0.6 ===

 *General*
    - Multithread capabilities via OpenMP are now detected during DGtal
      build. Example of usage can be found in the Volumetric module.

 *Documentation*
    - update documentation for boost concepts, so that subconcepts are
      displayed and html reference pages are pointed.
    - package/module documentation files are now in their associated
      package folder (e.g. kernel/doc/ for kernel package related
      documentation pages). The "make doc" command (or "make dox", see
      below) generates the documentation in the "html/" sub-folder of your
      current build folder.
    - latex citations within doxygen documents are now working

 *Base Package*
    - correct concept checks for some range concepts.
    - Statistic class moved to math package

 *Kernel Package*
    - digital sets are now also point predicates, update of
      DigitalSetDomain accordingly. As a consequence, SetPredicate is
      now deprecated.
    - exposed Compare template parameter of underlying std::set in
      DigitalSetBySTLSet class.

    - new documentation for module digital sets.

 *Arithmetic Package*
    - new class for representing lattice polytopes in 2D (with cut
      operations)
    - bugfix in LighterSternBrocot::Fraction
    - bugfix in ArithmeticalDSS (thanks, Kacper)

 *Image Package*
    - Update on image writers (no colormap required for scalar only writers).
      Documentation updated.
    - New image adapters to adapt both domains and values of an image
      (ImageAdapter and ConstImageAdapter).
    - several enhancements of the main image concept and its image
      container models

 *Geometry Package*
    - New primitives for digital plane recognition. Naive planes, and
      more generally planes with arbitrary axis-width can be detected
      and recognized incrementally. Based on a COBA algorithm
      implementation, which uses 2D lattice polytopes.
    - Fréchet segment computer added to compute bounded simplifications of
      digital curves for instance.
    - Complete rewritting of volumetric tools by separable processes:
      new generic algorithms (VoronoiMap, PowerMap) and metric
      concepts hierarchy (l_2, l_p, ...p) to efficiently compute
      DistanceTransformation, ReverseDistanceTransformation and
      preliminary medial axis extraction.
    - Separable volumetric tools are now multithread using OpenMP.
    - New curvature estimator in 2D/3D based on integral invariants
      (both mean and gaussian curvatures in 3D).

 *Shape Package*
    - New operators available on digital and Euclidean shapes (Union,
      Intersection, Minus)

 *Topology Package*
    - update documentation for digital surfaces and digital surface
      containers so as to emphasize the fact that the ranges are only
      single-pass.

 *Graph Package*
    - New package gathering graph related structures and algorithms
      (visitors, graph concepts, ...)
    - Add concepts for graph visitors
    - Add boost::graph support for DigitalSurface
    - Add documentation for graph package.

 *Math Package*
    - Exact exponentiation x^p by squaring on O(log p) added
      (BasicMathFunctions::power).

 *For developers*
    - new "make dox" target to only build dox file documentation
      ("make doc" for complete documentation build)


=== DGtal 0.5.1 ===
Posted on June, 6th, 2012 by David Coeurjolly

    - New way to cite package/module authors in the documentation
    - Improvement of DGtal::GridCurve ranges
    - Improvement of package concepts  in the  documentation
    - new documentation for DGTal build on MSWindows
    - arithmetic is now a main package (previously in math)
    - Specialized classes for classical metric adjacencies


=== DGtal 0.5 ===
Posted on May, 9th, 2012 by David Coeurjolly

Many changes have been pushed to this release with a lot of nice
tools.  Before going into details component by component, we would
like to focus on a couple of new cool features:

  - new arithmetic package (fractions, models of fraction,
    Stern-Brocot, continued fraction,...)
  - new nD DigitalSurface model (collections of (n-1) topological cells
    with many tools/utilities to track surface elements)
  - update of the build system to make easier the use of DGtal in your
    projects.
  - DGtal and DGtalTools
  - many bugfixes..

* Overall  Project

  - In previous DGtal releases, tools were given in the source
    "tools/" folder. In this release, we have chosen to move the
    tools to another GitHub project
    (http://github.com/DGtal-team/DGtalTools) with a specific
    development process. Please have a look to this project to get
    nice tools built upon the DGtal library.

  - cmake scripts and DGtalConfig have been widely updated to make
    easier the use of the library in your own code

  - We are debugging both the code and the scripts to make it compile
    on windows. We still have couple of issues but most of DGtal
    compiles.

  - Again, efforts have been done on the documentation.


* Package Topology:

 - Creation of the graph concept (see Doxygen documentation)

 - Graph tools have been added: breadth first visitor for any model of
   graph

 - Creation of high-level classes to represent several kinds of
   digital surfaces. Surfaces are n-1 dimensional objetcs and may be
   open or closed. There are several models of digital surface
   containers: boundary of a set of points, explicit set of surfels,
   boundary of a digital object defined by a predicate, frontier
   between two regions, light containers that are discovered on
   traversal but not stored explicitly, etc.

 - All these digital surfaces can be manipulated through the same
   object (DigitalSurface), whichever the container.

 - DigitalSurface is a model of a graph whose vertices are the surfels
   and whose arcs are the connections between surfels.

 - Definition of umbrellas over digital surfaces, that forms faces on
   the surface graph.

 - In 3D, digital surface form combinatorial 2-manifolds with boundary

 - Digital surface can be exported in OFF format

 - Several examples using digital surfaces are provided, like
   extracting isosurfaces from images or volume files defining
   surfaces in labelled images.

* Package Algebraic (new package)

 - Definition of n-variate polynomial as a one-dimensional polynomial
   whose coefficients are n-1-variate polynomials. Coefficient ring
   and dimension are templated.

 - Creation of a reader that can transform a string representation of
   multivariate polynomial into such polynomial object. Use
   boost::spirit.

 - Example using package Topology to extract and display implicit
   polynomial surfaces in 3D.

* Package Arithmetic (new package)

 - Standard arithmetic computations are provided: greatest common
   divisor, Bézout vectors, continued fractions,  convergent.

 - Several representations of irreducible fractions are provided. They
   are based on the Stern-Brocot tree structure. With these fractions,
   amortized constant time operations are provided for computing
   reduced fractions.

 - An implementation of patterns and subpatterns is provided, based on
   the irreducible fractions.
 - A representation of digital standard line in the first quadrant is
   provided, as well as fast algorithms to recognize digital straight
   subsegments.


* Package Image

  - Complete refactoring of Images and ImageContainers (more
    consistent design)

  - Documentation added

  - Graph of concepts added in the documentation


* Package Geometry

  - New SegmentComputer (a.k.a. geometrical primitives to use for
    recognition, curve decomposition,...) : ArithDSS3D (3D DSS), DCA
    (Digital Circular Arcs), CombinatorialDSSS, ...

  - New normal vector field estimation based on elementary normal
    vector convolution in n-D

  - Distance Transformation by Fast Marching Method added.

* Package IO

  - Complete refactoring of the way a DGtal object is displayed in
    boards/viewers.

  - New 2D board  backend: you can export your drawning in TikZ for
    latex includes.


=== DGtal 0.4 ===
Posted on September 26, 2011 by David Coeurjolly

	* Global changes:
	   - A better decomposition of DGtal algorithms and
	data structures into packages.
	   - By default, DGtal is built with minimal dependencies.
	   - Concepts and concept checking mechanism have been
	considerably improved.

	* Kernel Package: refactoring of Integer types considered in
	DGtal.

	* Topology Package: Interpixel/cellular topological model,
	boundary tracking tools, ...

	* Geometry Package:
	  - many things have been added in the 1D contour analysis module:
	multi-modal representation of 1D contours and curves (GridCurve facade),
	decomposition/segmentation into primitives, many differential
	estimators added, helpers for multigrid comparison of estimators
	  - multigrid digital set generators from implicit and parametric
	shapes in dimension 2.

	* I/O Package: refactoring/enhancements of DGtal boards and
	viewers,  enhancement of 2D boards with libcairo and a new
	Board3Dto2D board has been added.


	* Tools: multigrid shapeGenerator/contourGenerator added,
	lengthEstimator/estimatorComparator  added for differential
	estimator multigrid comparison, connected components extraction in
	3D, ...

	* Documentation: User guide has been improved thanks to a
	decomposition of the library into packages.

=== DGtal 0.3.1 ===
Posted on April 4, 2011 by David Coeurjolly

	* Quick release due to a build problem on linux. No more feature
	added.
	* Preliminary cellular grid documentation added.
	* Documentation cleanup.




=== DGtal 0.3.0 ===
Posted on April 1, 2011 by David Coeurjolly

Beside the DGtal presentation at DGCI 2011, we are pleased to announce a new
DGtal release 0.3.0.

New features:

    User-guide added (based on doxygen system)
    Kernel: new concepts and controls to enhance the Interger type management,
            new iterators (Range/SubRange) on HyperRectDomains.
    Topology: interpixel model added (cells, boundary tracking mechanisms,…)
    Geometry 2D: 2D curve primitive decomposition, tangential cover,
                 convexity/concavity decomposition.
    Geometry nD: reverse Euclidean distance transformation
    Visualisation: stream mechanism to visualize 3D DGtal objects with
	           libQGLViewer (optional) Shape generator factory added in nD

BugFixes, enhancements:

    Many bugs have been fixed for this release.
    cmake DGtal dependency checking process is more stable now

Known problems:

    For technical reasons, we haven’t be able to verify that this release also
    compile on Windows Visual Studio systems (see ticket #87). A new release
    will fix this problem as soon as possible. 





=== Older Releases ===





2011-04-01 dcoeurjo
	* Release 0.3.0
	* Kernel: global enhancement of different Integer types and
	associated concepts.
	* Topology: interpixel topology, cells, surface tracking
	* Geometry2D: contour primitive decomposition, tangential cover,
	convexity/concavity decomposition.
	* GeometrynD: Reverse DT transformation (Euclidean)
	* Infrastructure: 3D visualisation of DGtal objects with
	libQGLViewer, shape factory
	* IO: PointListReader added
	* Documentation: first DGtal user-guide


2010-01-12 dcoeurjo
	* Release 0.2
	* Kernel: DGtalBoard mechanism for 2D drawing of DGtal objects, ..
	* Geometry package
	   - Volumetric: distance transformation with separable	metric
	(l2, l1 and linfinity) in arbitrary dimension
	   - 2D: Arithmetical DSS, Greedy decomposition of a contour into
	primitives, FreemanChain code iterators
	* Topolopy package: Set, Adjacencies, Object, border extraction,
	connected components computation, ...
	* IO: 2D file formats with Magick++, Vol/Raw format in 3D, Raw
	format in n-D (non-portable)
	* Misc: Compiles on linux, MacOS and VisualStudio 2008


2010-21-05 dcoeurjo
	* Iterators added to PointVector
  * Debug methods removed in Trace class
  * Many bug fixes for VS compatibility

2010-05-15 dcoeurjo
  * Assert.h: added macro ASSERT() added based on the boost/assert.hpp (TODO:
	      implement a nice callback)
  * Point and Vector templated classes added
  * Space.*: skeleton of a DGtal::Space added

2010-03-03 dcoeurjo
	* math/MeasureOfStraightLines: new class to compute the measure of a set
	of Straight lines defined as a polygon in the (a,b) parameter space.
	* test_measure: added

2010-02-17 dcoeurjo
  * Trace: new class models for output streams in Trace class.
  * TraceWriter/TraceWriterTerm/TraceWriterFile: added

2010-02-12 dcoeurjo
  * models: bug fix  in INLINE commands
  * Trace/Clock: minor edit and bug report

2010-01-05 dcoeurjo
  * Trace can be initialized on diffrent output stream (e.g. std::cerr or a file
	  stream)
  * test_trace: update to test the new API

2010-01-04 dcoeurjo
  * Clock: no more static variables and methods (in order to have several
	   running clocks)
  * Trace: new interface and the endBlock displays and returns the
	   ellapsed time within the block

2009-12-28 dcoeurjo
  * Trace: a new class to trace out messages to the standard output. Four type
	   of messages are possible: info, debug, error and "emphased". On
	   color linux terminal, messages appears with an appropriate color
	   foreground.
  * test_trace: an illustration of the Trace interface

2009-12-14 dcoeurjo
  * CMakeLists, test_clock updates to ensure compatibility with VisualStudio
  * New cmake options
    - OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)
    - OPTION(BUILD_TESTS "Build tests." ON)

2009-12-11 dcoeurjo
	* CMakeLists scripts and first backport from imagene (Clock class)

2009-12-11 dcoeurjo
	* Repository cleanup:
		  - Modeles and genereateClass.sh removed
      - JOL scripts & templates added


2009-12-03 dcoeurjo
	* Modeles: class templates added with generateClass.sh script
