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
 * @file Lattice.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/06
 *
 * Header file for module Lattice.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Lattice_RECURSES)
#error Recursive header files inclusion detected in Lattice.h
#else // defined(Lattice_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Lattice_RECURSES

#if !defined Lattice_h
/** Prevents repeated inclusion of headers. */
#define Lattice_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Lattice
  /**
   * Description of template class 'Lattice' <p> \brief Aim:
   * Represents an n-dimensional integer lattice in an m-dimensional real vector
   * space.
   *
   * @param TSpace the source digital space for the lattice.
   */
  template <typename TSpace>
  class Lattice
  {
    // ----------------------- Concept checks ------------------------------
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));

   // ----------------------- Associated types ------------------------------
  public:
    typedef TSpace Space;
    typedef typename TSpace::Integer Integer;
    typedef typename TSpace::Point Point;
    typedef typename TSpace::Dimension Dimension;


   // ----------------------- Static services ------------------------------
  public:

    /**
     * Constructs a default 2D lattice centered at [x0], with vectors
     * (dh,0,0) and (0,dh,0).
     *
     * @param l (returns) the lattice
     * @param x0 the origin of the lattice embedding (a 3D point)
     * @param dh the grid step or length of each vector.
     */
    static void defaultZ2toZ3( Lattice & l, const double* x0, double dh = 1.0 ); 

    /**
     * Constructs a default 3D lattice centered at [x0], with vectors
     * (dh,0,0) and (0,dh,0) and (0,0,dh).
     *
     * @param l (returns) the lattice
     * @param x0 the origin of the lattice embedding (a 3D point)
     * @param dh the grid step or length of each vector.
     */
    static void defaultZ3toZ3( Lattice & l, const double* x0, double dh = 1.0 );

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~Lattice();

    /**
     * Constructor. Invalid.
     */
    Lattice();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Lattice ( const Lattice<Space> & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Lattice<Space> & operator= ( const Lattice<Space> & other );

    /**
     * Defines the standard lattice when aN=aM, the standard injection
     * when aN < aM, and when aM > aN, vectors are overlapping.
     *
     * @param aN the dimension of the lattice space
     *
     * @param aM the dimension of the target space where the lattice is
     * represented.
     *
     * @param dh the length of the elementary displacement.
     */
    void init( Dimension aN, Dimension aM, double dh = 1.0 );

    /**
     * Resets the lattice.
     */
    void clear();

    /**
     * @return the dimension of the lattice space.
     */
    Dimension n() const;

    /**
     * @return the dimension of the target space.
     */
    Dimension m() const;


    /**
     * Sets the origin of the space.
     * @param x is a vector of size m.
     */
    void setOrigin( const double* x );  

    /**
     * Sets the elementary vector along direction i
     * @param i is a coordinate between 0 and n-1.
     * @param v is a vector of size m.
     */
    void setVector( Dimension i, const double* v );  

    /**
     * Immerse a point of the lattice into the target space of dimension m.
     *
     * @param p the n coordinates of a point in the lattice.
     * @param x (returns) its m coordinates in the target space of dimension m.
     */
    void immerse( const Integer* p, double* x ) const;

    /**
     * Immerse a point of the lattice into the target space of dimension m.
     *
     * @param p the n coordinates of a point in the lattice.
     * @param x (returns) its m coordinates in the target space of dimension m.
     */
    void immerse( const Point & p, double* x ) const;

    /**
     * Immerse a point of the lattice into the target space of dimension m.
     *
     * @param p the n coordinates of a point in the lattice.
     * @param x (returns) its m coordinates in the target space of dimension m.
     */
    void immerse( const Integer* p, float* x ) const;

    /**
     * Immerse a point of the lattice into the target space of dimension m.
     *
     * @param p the n coordinates of a point in the lattice.
     * @param x (returns) its m coordinates in the target space of dimension m.
     */
    void immerse( const Point & p, float* x ) const;


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

    /**
     * Dimension of lattice space;
     */
    unsigned int myN;

    /**
     * Dimension of target space;
     */
    unsigned int myM;

    /**
     * Origin. Array of size m_m;
     */
    double* myX0;

    /**
     * Vectors for each dimension. Array of size m_n times m_m;
     */
    double* myV;


    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Lattice


  /**
   * Overloads 'operator<<' for displaying objects of class 'Lattice'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Lattice' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out, const Lattice<TSpace> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/Lattice.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Lattice_h

#undef Lattice_RECURSES
#endif // else defined(Lattice_RECURSES)
