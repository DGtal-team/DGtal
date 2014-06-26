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
 * @file RigidTransformation2D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/06/26
 *
 * Header file for module RigidTransformation2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RigidTransformation2D_RECURSES)
#error Recursive header files inclusion detected in RigidTransformation2D.h
#else // defined(RigidTransformation2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RigidTransformation2D_RECURSES

#if !defined RigidTransformation2D_h
/** Prevents repeated inclusion of headers. */
#define RigidTransformation2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include <DGtal/helpers/StdDefs.h>
#include "DGtal/images/CImage.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class RigidTransformation2D
  /**
   * Description of template class 'RigidTransformation2D' <p>
   * \brief Aim:
   */
  
  // \todo discretization functor
  template <typename Image>
  class RigidTransformation2D
  {
    BOOST_CONCEPT_ASSERT(( CImage<Image> ));
    
    // ----------------------- Types ------------------------------
  public:
    enum Model { EULERIAN, LAGRANGIAN };
    
    // ----------------------- Standard services ------------------------------
  public:
    RigidTransformation2D()
    {
      t_sin = 0.;
      t_cos = 1.;
    }
    
    // setters
    //! \param center of rotation.
    void setCenter ( Z2i::Point &center )
    {
      t_center = center;
    }
    //! \param angle in radians
    void setAngle ( double angle )
    {
      t_cos = std::cos ( angle );
      t_sin = std::sin ( angle );
    }
    void setTranslation ( Z2i::RealVector & vector )
    {
      translation = vector;
    }
    Image transform ( Image & image, Model model );
    // getters
    
    /**
     * Destructor.
     */
    ~RigidTransformation2D();

    // ----------------------- Interface --------------------------------------
  public:

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
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    double t_sin, t_cos;
    Z2i::Point t_center;
    Z2i::RealVector translation;

    // ------------------------- Hidden services ------------------------------

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    RigidTransformation2D ( const RigidTransformation2D & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    RigidTransformation2D & operator= ( const RigidTransformation2D & other );

    // ------------------------- Internals ------------------------------------
  private:
   // Allocation of output image
    Image allocate ( typename Image::Domain & domain );
    typename Image::Point transform ( typename Image::Point &point );
    typename Image::Point transformInverted ( typename Image::Point &point );
    void lagrangianTransformation ( Image & image ){}
    void eulerianTransformation ( Image & image ){}
  }; // end of class RigidTransformation2D
  

  /**
   * Overloads 'operator<<' for displaying objects of class 'RigidTransformation2D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'RigidTransformation2D' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const RigidTransformation2D<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/RigidTransformation2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RigidTransformation2D_h

#undef RigidTransformation2D_RECURSES
#endif // else defined(RigidTransformation2D_RECURSES)

