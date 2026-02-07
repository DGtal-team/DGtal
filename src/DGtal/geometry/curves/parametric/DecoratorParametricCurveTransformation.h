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
 * @file DecoratorParametricCurveTransformation.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/10
 *
 * Header file for module DecoratorParametricCurveTransformation.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DecoratorParametricCurveTransformation_RECURSES)
#error Recursive header files inclusion detected in DecoratorParametricCurveTransformation.h
#else // defined(DecoratorParametricCurveTransformation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DecoratorParametricCurveTransformation_RECURSES

#if !defined DecoratorParametricCurveTransformation_h
/** Prevents repeated inclusion of headers. */
#define DecoratorParametricCurveTransformation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/parametric/C3DParametricCurve.h"
#include <functional>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class DecoratorParametricCurveTransformation
/**
 * Description of class 'DecoratorParametricCurveTransformation' <p>
 * \brief Aim: Implements a decorator for applying transformations to
 * parametric curves.
 * @tparam TCurve a model of C3DParametricCurve
 * @tparam TTransfromation a functor taking an instance of TCurve::Space::RealPoint and returning an
 * instance of TCurve::Space::RealPoint
 */
template <typename TCurve, typename TTransfromation >
class DecoratorParametricCurveTransformation
{
    BOOST_CONCEPT_ASSERT(( concepts::C3DParametricCurve < TCurve > ));
    // ----------------------- Standard services ------------------------------
public:

    typedef TCurve TypeCurve;
    typedef typename TCurve::Space Space;
    typedef typename Space::RealPoint RealPoint;
    typedef typename Space::Point Point;

    DecoratorParametricCurveTransformation ( const TCurve &, const TTransfromation & );
    /**
     * Destructor.
     */
    ~DecoratorParametricCurveTransformation() = default;

    // ----------------------- Interface --------------------------------------
public:

    const TCurve & curve;

    /**
     * @param t any angle between 0 and k*Pi.
     *
     * @return the vector (x(t),y(t), z(t))
     */
    RealPoint x ( double t ) const;

    /**
     * @param t any angle between 0 and k*Pi.
     *
     * @return the vector (x(t)',y(t)', z(t)')
     */
    RealPoint xp ( double t ) const;


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

    // ------------------------- Protected Data ------------------------------
private:
    // ------------------------- Private Data --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DecoratorParametricCurveTransformation();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DecoratorParametricCurveTransformation ( const DecoratorParametricCurveTransformation & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DecoratorParametricCurveTransformation & operator= ( const DecoratorParametricCurveTransformation & other );

    // ------------------------- Internals ------------------------------------
private:
    const TTransfromation & trans;


}; // end of class DecoratorParametricCurveTransformation


/**
 * Overloads 'operator<<' for displaying objects of class 'DecoratorParametricCurveTransformation'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'DecoratorParametricCurveTransformation' to write.
 * @return the output stream after the writing.
 */
template <typename TCurve, typename TTransfromation>
inline
std::ostream&
operator<< ( std::ostream & out, const DecoratorParametricCurveTransformation< TCurve, TTransfromation > & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/parametric/DecoratorParametricCurveTransformation.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DecoratorParametricCurveTransformation_h

#undef DecoratorParametricCurveTransformation_RECURSES
#endif // else defined(DecoratorParametricCurveTransformation_RECURSES)
