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
 * @file Knot_4_3.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/01
 *
 * Header file for module Knot_4_3
 *
 * This file is part of the DGtal library.
 */

#if defined(Knot_4_3_RECURSES)
#error Recursive header files inclusion detected in Knot_4_3.h
#else // defined(Knot_4_3_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Knot_4_3_RECURSES

#if !defined Knot_4_3_h
/** Prevents repeated inclusion of headers. */
#define Knot_4_3_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Knot_4_3
/**
 * Description of class 'Knot_4_3' <p>
 * \brief Aim: Implement a parametrized knot 4, 3.
 * @tparam TSpace model of CSpace
 */
template <typename TSpace>
class Knot_4_3
{
    // ----------------------- Standard services ------------------------------
public:
    typedef TSpace Space;
    typedef typename TSpace::RealPoint RealPoint;
    typedef typename TSpace::Point Point;

    /**
     * Destructor.
     */
    ~Knot_4_3() {}

    /**
     * Constructor.
     * @param scale_1 x-axis scaling factor
     * @param scale_2 y-axis scaling factor
     * @param scale_3 z-axis scaling factor
     */
    Knot_4_3 ( long double scale_1, long double scale_2,  long double scale_3 );


    // ----------------------- Interface --------------------------------------
public:

    /**
     * @param t any value.
     *
     * @return the vector (x(t), y(t), z(t))
     */
    RealPoint x ( long double t ) const;

    /**
     * @param t any value.
     *
     * @return the vector (x(t)', y(t)', z(t)')
     */
    RealPoint xp ( long double t ) const;

    static double getPeriod();


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
    long double scale[3];
    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Knot_4_3();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Knot_4_3 ( const Knot_4_3 & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Knot_4_3 & operator= ( const Knot_4_3 & other );


    // ------------------------- Internals ------------------------------------
private:
    static constexpr double PERIOD = M_PI * 2.0;

}; // end of class Knot_4_3


/**
 * Overloads 'operator<<' for displaying objects of class 'Knot_4_3'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Knot_4_3' to write.
 * @return the output stream after the writing.
 */
template <typename T>
std::ostream&
operator<< ( std::ostream & out, const Knot_4_3<T> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/parametric/Knot_4_3.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Knot_4_3_h

#undef Knot_4_3_RECURSES
#endif // else defined(Knot_4_3_RECURSES)
