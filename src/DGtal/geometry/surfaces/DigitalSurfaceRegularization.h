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
   *  MapSurfel -> Normal
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
     * surfels (SH3::Cell) to normal vectors (SH3::RealVector).
     */
    void attachNormalVectors(const std::function<SHG3::RealVector(SH3::Cell)> &normalFunc);
    
    /**
     * Attach trivial normal vectors to the digital surface
     * (@see getCTrivialNormalVectors).
     *
     * An important parameter is the radius used to estimate the normal vectors (@a t-ring, default=3.0).
     *
     */
    void attachTrivialNormalVectors(const Parameters someParams
                                    = SH3::defaultParameters() | SHG3::defaultParameters() );
    
    
    /**
     * @brief Initialize the parameters of the energy function.
     *
     * @param [in] alpha the data attachment term coeff. (default=0.001)
     * @param [in] beta  the alignemnt term coeff. (default=1.0)
     * @param [in] gamma the fairness term coeef. (default=0.05)
     */
    void init(const double alpha = 0.001,
              const double beta  = 1.0,
              const double gamma = 0.05);
    
    
    /**
     * Compute the enegery gradient vector and return the energy value.
     *
     * @note init() method must have been called and normal vectors must be attached
     * to the surfels.
     *
     * @return the energy value.
     **/
    double computeGradient();
    
    
    /**
     * @brief Main regularization loop.
     *
     * This method performs the main minimization loop of the energy
     * using a gradient descent scheme. The iterative process stops either when
     * the number of steps reaches @a nbIters, or when the @f$l_\infy@f$ norm of
     * the energy gradient is below @a epsilon.
     *
     * The energy at the final step is returned.
     *
     * @param [in] nbIters maxium number of steps (default=500)
     * @param [in] dt initial learning rate (default = 1.0)
     * @param [in] epsilon minimum l_infity norm of the gradient vector (default = 0.0001)
     * @return the energy at the final step.
     */
    double regularize(const unsigned int nbIters = 200,
                      const double dt = 1.0,
                      const double epsilon = 0.0001);
    
    
    
    /**
     * @return the regulariezed vertices positions
     * (see getCellIndex for the Cell->Index map).
     */
    const Positions & getRegularizedPositions() const
    {
      return myRegularizedPositions;
    }
    /**
     * @return the input vertices positions
     * (see getCellIndex for the Cell->Index map).
     */
    const Positions & getOriginalPositions() const
    {
      return myOriginalPositions;
    }
    /**
     * @return the input normal vectors
     * (see getCellIndex for the Cell->Index map).
     */
    const Normals & getNormalVectors() const
    {
      return myNormals;
    }
    /**
     * @return the CellIndex (Cell->Index map) for
     the positions and normal vectors containers.
     */
    const SH3::Cell2Index & getCellIndex() const
    {
      return myCellIndex;
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
    
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    ///Data attachment term coefficient
    double myAlpha;
    ///Alignment term coefficient
    double myBeta;
    ///Fairness term coefficient
    double myGamma;
    
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
    ///Gradient of the energy w.r.t. vertex positons
    ///TODO: remove this structure?
    Positions myGradientAlign;
    
    
    ///Instance of the KSpace
    SH3::KSpace myK;
    
    ///Indexed surface elements
    SH3::SurfelRange mySurfels;
    
    ///Indices for pointels
    SH3::Cell2Index myCellIndex;
    
    ///Indices of adjacent pointels for the Alignement energy term
    std::vector< SH3::Idx > myAlignPointelsIdx;
    ///Adjacent pointels for the Alignement energy term
    std::vector< SH3::Cell > myAlignPointels;
    ///Number of adjacent edges to pointels
    std::vector<unsigned char> myNumberAdjEdgesToPointel;
    ///Indices of cells foor the Fairness term
    std::vector< SH3::Idx > myFairnessPointelsIdx;
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
