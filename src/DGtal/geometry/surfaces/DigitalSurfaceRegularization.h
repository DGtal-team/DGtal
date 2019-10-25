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
#include "DGtal/topology/CDigitalSurfaceContainer.h"

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
   * @tparam TDigitalSurface a model of concepts::CDigitalSurfaceContainer.
   */
  template <typename TDigitalSurface>
  class DigitalSurfaceRegularization
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    ///DigitalSurface type
    typedef TDigitalSurface DigitalSurface;
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSurfaceContainer< TDigitalSurface > ));
    
    ///We rely on the Shortcuts 3D types
    typedef Shortcuts<Z3i::KSpace> SH3;
    
    ///We rely on the ShortcutsGeometry 3D types
    typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
    
    ///Pointels position container
    typedef std::vector<Z3i::RealPoint> Positions;
    
    ///Normal vector per surfel container
    typedef std::vector<Z3i::RealPoint> Normals;
    
    
    
    /**
     * Default constructor.
     */
    DigitalSurfaceRegularization()
    {
      myInit=false;
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
    
    void attachNormalVectorField();
    
    
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
     * @param [in] epsilon minimum l_infity norm of the gradient vector (default = 0.0001)
     * @return the energy at the final step.
     */
    double regularize(const unsigned int nbIters = 500,
                      const double epsilon = 0.0001);
    
    
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
