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

#pragma once

/**
 * @file
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2019/10/25
 *
 * Header file for module DigitalSurfaceRegularization.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurfaceRegularization_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceRegularization.h
#else // defined(DigitalSurfaceRegularization_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceRegularization_RECURSES

#if !defined DigitalSurfaceRegularization_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceRegularization_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/topology/DigitalSurface.h"

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
  {

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSurfaceRegularization
  /**
   * Description of template class 'DigitalSurfaceRegularization' <p>
   * \brief Aim: Implements Digital Surface Regularization as described in @cite coeurjolly17regDGCI.
   *
   *  Given a digital surface and a normal vector field attached to the surfels, this class
   *  performs a regularization of the digital surface vertices to obtain a piecewise smooth
   *  surface such that each quad of the output surface is as perpendicular as possible to the given
   *  input normal vector field.
   *
   * @note The digital surface must be a closed.
   *
   * If @f$P@f$ denotes the vertices of the input digital surface, @f$F@f$ the set of
   * (quadrilateral) faces and @f$n_f@f$ an estimated normal vector on the
   * face @f$f@f$, we want the quad surface vertex positions @f$P^*@f$ that
   * minimizes the following energy function:
   * @f[\mathcal{E}(P) := \alpha \sum_{i=1}^{n} \|p_i - \hat{p}_i\|^2  +
   *      \beta \sum_{f\in F} \sum_{{e_j} \in \partial{f} } ( e_j \cdot n_{f} )^2 + \gamma \sum_{i=1}^{n} \|\hat{p}_i - \hat{b}_i\|^2\,.@f]
   * where @f$"\cdot"@f$ is the standard @f$\mathbb{R}^3@f$ scalar product, @f$e_j\in
   * \partial{f}@f$ is an edge of the face @f$f@f$ (and is equal to some @f$p_k -
   * p_l@f$) and @f$ \hat{b}_i@f$ is the barycenter of the vertices adjacent to
   * @f$\hat{p}_i@f$.
   *
   * To minimize this energy, instead of solving the associated sparse linear system as described in @cite coeurjolly17regDGCI,
   * we perform a gradient descent strategy which allows us a finer control over the vertices displacement (see advection methods).
   *
   * @see testDigitalSurfaceRegularization.cpp
   *
   * @tparam TDigitalSurface a Digital Surface type (see DigitalSurface).
   */
  template <typename TDigitalSurface>
  class DigitalSurfaceRegularization
  {
    // ----------------------- Standard services ------------------------------
  public:

    ///DigitalSurface type
    typedef TDigitalSurface DigSurface;
    ///Digital Surface Container type
    typedef typename TDigitalSurface::DigitalSurfaceContainer DigitalSurfaceContainer;
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSurfaceContainer< DigitalSurfaceContainer > ));

    ///We rely on the Shortcuts 3D types
    typedef Shortcuts<Z3i::KSpace> SH3;

    ///We rely on the ShortcutsGeometry 3D types
    typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

    ///Pointels position container
    typedef std::vector<Z3i::RealPoint> Positions;

    ///Pointels position container
    typedef std::vector<Z3i::RealVector> Normals;

    /**
     * Default constructor.
     */
    DigitalSurfaceRegularization(CountedPtr<DigSurface> aDigSurf)
    :  myInit(false), myVerbose(false), myDigitalSurface(aDigSurf)
    {
      myK = SH3::getKSpace( myDigitalSurface );
    }

    /**
     * Destructor.
     */
    ~DigitalSurfaceRegularization() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalSurfaceRegularization ( const DigitalSurfaceRegularization & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    DigitalSurfaceRegularization ( DigitalSurfaceRegularization && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DigitalSurfaceRegularization & operator= ( const DigitalSurfaceRegularization & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    DigitalSurfaceRegularization & operator= ( DigitalSurfaceRegularization && other ) = delete;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Attach normal vectors from a generic function that associates
     * a normal vector to each surfel.
     *
     * @param normalFunc a function (or a lambda, or a functor) that maps
     * surfels (SH3::SCell) to normal vectors (SH3::RealVector).
     */
    void attachNormalVectors(const std::function<SHG3::RealVector(SH3::SCell&)> &normalFunc);

    /**
     * Attach convolved trivial normal vectors to the digital surface
     * (cf ShortCutsGeometry::getCTrivialNormalVectors).
     *
     * An important parameter is the radius used to estimate the normal vectors (@a t-ring, default=3.0).
     *
     * @param someParams the parameters (@a t-ring, default=3.0 used in the local convolution).
     */
    void attachConvolvedTrivialNormalVectors(const Parameters someParams
                                    = SH3::defaultParameters() | SHG3::defaultParameters() );


    /**
     * @brief Initialize the parameters of the energy function.
     *
     * This init() method attaches the same weights to all vertices.
     *
     * @param [in] alpha the data attachment term coeff. (default=0.001)
     * @param [in] beta  the alignment term coeff. (default=1.0)
     * @param [in] gamma the fairness term coeef. (default=0.05)
     */
    void init(const double alpha = 0.001,
              const double beta  = 1.0,
              const double gamma = 0.05);


    /**
     * @brief Initialize the parameters of the energy function.
     *
     * This init() method considers local weights per vertex (all vectors
     * must have the same size: the number of pointels of the original surface).
     *
     * @param [in] alphas the data attachment term coeff.
     * @param [in] betas  the alignment term coeff.
     * @param [in] gammas the fairness term coeef.
     */
    void init(ConstAlias< std::vector<double> > alphas,
              ConstAlias< std::vector<double> > betas,
              ConstAlias< std::vector<double> > gammas);


    /**
     * Compute the energy gradient vector and return the energy value.
     *
     * @note init() method must have been called and normal vectors
     * must be have been attached to the surfels.
     *
     * @return the energy value.
     **/
    double computeGradient();


    /**
     * Compute the energy gradient vector and return the energy value.
     *
     * This method assumes that you have local alpha, beta and gamma weights.
     *
     * @note init(alphas,betas,gammas) method must have been called and normal vectors
     * must have been attached to the surfels.
     *
     * @return the energy value.
     **/
    double computeGradientLocalWeights();


    /**
     * @brief Main regularization loop.
     *
     * This method performs the main minimization loop of the energy
     * using a gradient descent scheme (with automatic update of the learning step).
     * The iterative process stops either when the number of steps reaches @a nbIters,
     * or when the @f$l_\infty@f$ norm of the energy gradient is below @a epsilon.
     *
     * The last parameter is a function that describes how regularized points are advected
     * during the gradient descent. By default, points are shifted by a fraction of the energy
     * gradient vector (and the default function is thus @f$ p \leftarrow p + v@f$ with
     * @f$ v = -dt  \nabla E_p@f$). See @see clampedAdvection for another advection strategy.
     *
     * The energy at the final step is returned.
     *
     * @param [in] nbIters maximum number of steps
     * @param [in] dt initial learning rate
     * @param [in] epsilon minimum l_infity norm of the gradient vector
     * @param [in] advectionFunc advection function/functor/lambda to move a regularized point &a p associated with
     * the original point @a o w.r.t to a displacement vector @a v (default = p+v)
     * @tparam AdvectionFunction type of advection function, functor or lambda (RealPoint, RealPoint, RealVector)->RealPoint.
     * @return the energy at the final step.
     */
    template <typename AdvectionFunction>
    double regularize(const unsigned int nbIters,
                      const double dt,
                      const double epsilon,
                      const AdvectionFunction & advectionFunc);
    /**
        * @brief Main regularization loop.
        *
        * This method performs the main minimization loop of the energy
        * using a gradient descent scheme (with automatic update of the learning step).
        * The iterative process stops either when the number of steps reaches @a nbIters,
        * or when the @f$l_\infty@f$ norm of the energy gradient is below @a epsilon.
        *
        * This methods uses the default advection function @f$ p = p + v@f$.
        *
        * The energy at the final step is returned.
        *
        * @param [in] nbIters maximum number of steps (default=200)
        * @param [in] dt initial learning rate (default = 1.0)
        * @param [in] epsilon minimum l_infity norm of the gradient vector (default = 0.0001)
        * @return the energy at the final step.
        */
       double regularize(const unsigned int nbIters = 200,
                         const double dt = 1.0,
                         const double epsilon = 0.0001)
       {
         return regularize(nbIters,dt,epsilon,
                           [](SHG3::RealPoint& p,SHG3::RealPoint& o,SHG3::RealVector& v){ (void)o; p += v; });
       }



    /**
     * Static method to be used in @e regularize() that
     * clamps to regularized point @a p when shifted by @a v
     * in the unit cube centered at @a orig.
     *
     * @param [in,out] p the point to advect.
     * @param [in] orig the associated point in the original surface.
     * @param [in] v the advection vector.
     */
    static void clampedAdvection(SHG3::RealPoint &p,
                                 const SHG3::RealPoint &orig,
                                 const SHG3::RealVector &v)
    {
      p += v;
      for(auto i=0; i < 3; ++i)
        if ((p[i]-orig[i])> 0.5) p[i]=orig[i]+0.5;
        else
          if ((p[i]-orig[i])< -0.5) p[i]=orig[i]-0.5;
    }


    /**
     * @returns the regularized position of a given pointel @a aPointel.
     * @param[in] aPointel the digital surface pointel.
     *
     * @note the init() method must have been called. For relevant results, the regularize()
     * methods should have been also called before accessing the new positions.
     */
    SHG3::RealPoint getRegularizedPosition(const SH3::Cell& aPointel)
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      ASSERT_MSG(myK.uDim(aPointel) == 0, "The cell must be a pointel (0-cell)");
      return myRegularizedPositions[ myPointelIndex[ aPointel] ];
    }

    /**
     * @return the regularized vertices positions
     * (see getCellIndex for the Cell->Index map).
     * @note the init() method must have been called.
     */
    const Positions & getRegularizedPositions() const
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      return myRegularizedPositions;
    }
    /**
     * @return the input vertices positions
     * (see getCellIndex for the Cell->Index map).
     * @note the init() method must have been called.
     */
    const Positions & getOriginalPositions() const
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      return myOriginalPositions;
    }
    /**
     * @return the CellIndex (Cell->Index map) for
     * the positions and normal vectors containers.
     * @note the init() method must have been called.
     */
    const SH3::Cell2Index & getCellIndex() const
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      return myPointelIndex;
    }
    /**
     * @return the input normal vectors
     * (see getSurfelIndex for the Cell->Index map).
     * @note the init() method must have been called.
     */
    const Normals & getNormalVectors() const
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      return myNormals;
    }

    /**
     * @return the CellIndex (Cell->Index map) for
     the positions and normal vectors containers.
     * @note the init() method must have been called.
     */
    const SH3::Surfel2Index & getSurfelIndex() const
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      return mySurfelIndex;
    }

    /**
     * Reset the regularized vertices positions to the original one.
     * @note the init() method must have been called.
     */
    void reset()
    {
      ASSERT_MSG(myInit, "The init() method must be called first.");
      std::copy(myOriginalPositions.begin(), myOriginalPositions.end(), myRegularizedPositions.begin());
    }


    // ----------------------- Services --------------------------------------


    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * Enable verbose messages.
     */
    void enableVerbose() {myVerbose=true;}

    /**
     * Disable verbose messages.
     */
    void disasbleVerbose() {myVerbose=false;}

    // ------------------------- Private methods --------------------------------
  private:

    /**
     * Internal init method to set up topological caches.
     */
    void cacheInit();


    // ------------------------- Private Data --------------------------------
  private:

    ///Data attachment term coefficient
    double myAlpha;
    ///Alignment term coefficient
    double myBeta;
    ///Fairness term coefficient
    double myGamma;

    ///Data attachment term local coefficients
    const std::vector<double> *myAlphas;
    ///Alignment attachment term local coefficients
    const std::vector<double> *myBetas;
    ///Fairness attachment term local coefficients
    const std::vector<double> *myGammas;

    ///Flag if the gradient has constant weights for the init method and gradient.
    bool myConstantCoeffs;

    ///Flag if the object has been set up properly
    bool myInit;

    ///Flag for verbose messages
    bool myVerbose;

    ///Input DigitalSurface to regularize
    CountedPtr<DigSurface> myDigitalSurface;

    ///Copy of the input pointels positions.
    Positions myOriginalPositions;

    ///Regularized vertices
    Positions myRegularizedPositions;

    ///Normals
    Normals myNormals;

    ///Gradient of the energy w.r.t. vertex positons
    Positions myGradient;



    // ---------------------------------------------------------------
    ///Internal members to store precomputed topological information

    ///Gradient of the quad alignment w.r.t. vertex positons
    Positions myGradientAlign;

    ///Instance of the KSpace
    SH3::KSpace myK;

    ///Indexed surface elements
    SH3::SurfelRange mySurfels;

    ///Surfel Index
    SH3::Surfel2Index mySurfelIndex;

    ///Indices for pointels
    SH3::Cell2Index myPointelIndex;

    ///Indices of adjacent pointels for the Alignment energy term
    std::vector< SH3::Idx > myAlignPointelsIdx;
    ///Adjacent pointels for the Alignment energy term
    std::vector< SH3::Cell > myAlignPointels;
    ///Number of adjacent edges to pointels
    std::vector<unsigned char> myNumberAdjEdgesToPointel;
    ///Indices of cells foor the Fairness term
    std::vector< SH3::Idx > myFairnessPointelsIdx;
    ///Number of adjacent faces to given vertex
    std::vector< unsigned char > myNbAdjacent;
    ///All faces of the dual digital surfacce
    SH3::PolygonalSurface::FaceRange myFaces;

  }; // end of class DigitalSurfaceRegularization


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfaceRegularization'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfaceRegularization' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSurfaceRegularization<T> & object );

  } // namespace surfaces


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#include "DGtal/geometry/surfaces/DigitalSurfaceRegularization.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceRegularization_h

#undef DigitalSurfaceRegularization_RECURSES
#endif // else defined(DigitalSurfaceRegularization_RECURSES)
