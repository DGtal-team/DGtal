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
 * @file SphericalAccumulator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/09/17
 *
 * Header file for module SphericalAccumulator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SphericalAccumulator_RECURSES)
#error Recursive header files inclusion detected in SphericalAccumulator.h
#else // defined(SphericalAccumulator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SphericalAccumulator_RECURSES

#if !defined SphericalAccumulator_h
/** Prevents repeated inclusion of headers. */
#define SphericalAccumulator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SphericalAccumulator
  /**
   * Description of template class 'SphericalAccumulator' <p> 
   *
   * \brief Aim: implements an accumulator (as histograms for 1D scalars)
   * adapted to spherical points.
   *
   * Spherical accumulator can be used to count  the number of directions given
   * by a set of vectors which fall into a given bin (i,j). 
   *
   * The accumulator is parametrized by two angular resolutions and
   * has got the property that each bin covers a region on the unit
   * sphere with similar area. We obtain thus a fast approximation of
   * an exact geodesic covering of the sphere with patches.
   *   
   * @tparam TVector type used to represent directions.
   *
   */
  template <typename TVector>
  class SphericalAccumulator
  {
    // ----------------------- Standard services ------------------------------
  public:

    ///Vector direction types
    typedef TVector Vector;

    ///Type to store the bin counts
    typedef DGtal::uint32_t Quantity;

    ///Type to represent bin indexes
    typedef DGtal::uint32_t Size;


    /** 
     * Constructs a spherical accumulator with @a aNphi times @a
     * Ntheta bins.
     * 
     * @param aNphi the number of slices in the longitude polar coordinates.  
     */
    SphericalAccumulator(const Size aNphi);

    /**
     * Destructor.
     */
    ~SphericalAccumulator();

    // ----------------------- Interface --------------------------------------
  public:

    /** 
     * Add a new direction into the accumulator;
     * 
     * @param aDir a direction
     */
    void addDirection(const Vector &aDir);

    /** 
     * Given a direction, this method computes the bin coordinates.
     * 
     * @param aDir a direction
     * @param posPhi position according to the first direction
     * @param posTheta position according to the second direction
     */
    void binCoordinates(const Vector &aDir, 
			   Size &posPhi, 
			   Size &posTheta);


    /** 
     * Returns the current number of samples in the bin
     * (posPhi,posTheta).
     * 
     * @param posPhi position according to the first direction
     * @param posTheta position according to the second direction
     *
     * @return the number of accumulated samples
     */
    Quantity count( Size &posPhi, 
		    Size &posTheta);
   
    
    /** 
     * Returns the representative direction associated with the bin
     * (posPhi,posTheta).
     * If the bin does not contain any sample, a null vector is
     * returned ( @a Vector::ZERO ).
     * 
     * @param posPhi position according to the first direction
     * @param posTheta position according to the second direction
     *
     * @return the representative direction.
     */
    Vector representativeDirection(Size &posPhi, 
				   Size &posTheta);
    
    /** 
     * @return returns the number of directions in the current
     * accumulator.
     *
     */
    Quantity samples();
        

    /** 
     * Clear the current accumulator.
     * 
     */    
    void clear();

    
    
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

    ///Number of slices in the phi direction
    Size myNphi;

    ///Number of bins in the theta direction
    Size myNtheta;

    ///Accumulator container
    std::vector<Quantity> myAccumulator; 
    
    ///Number of samples
    Quantity myTotal;
    
    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    SphericalAccumulator();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    SphericalAccumulator ( const SphericalAccumulator & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    SphericalAccumulator & operator= ( const SphericalAccumulator & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class SphericalAccumulator


  /**
   * Overloads 'operator<<' for displaying objects of class 'SphericalAccumulator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SphericalAccumulator' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const SphericalAccumulator<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/tools//SphericalAccumulator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SphericalAccumulator_h

#undef SphericalAccumulator_RECURSES
#endif // else defined(SphericalAccumulator_RECURSES)
