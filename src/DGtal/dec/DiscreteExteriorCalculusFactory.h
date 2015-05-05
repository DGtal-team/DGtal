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
 * @file DiscreteExteriorCalculusFactory.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2015/05/04
 *
 * Header file for DiscreteExteriorCalculusFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(DiscreteExteriorCalculusFactory_RECURSES)
#error Recursive header files inclusion detected in DiscreteExteriorCalculusFactory.h
#else // defined(DiscreteExteriorCalculusFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DiscreteExteriorCalculusFactory_RECURSES

#if !defined DiscreteExteriorCalculusFactory_h
/** Prevents repeated inclusion of headers. */
#define DiscreteExteriorCalculusFactory_h

//////////////////////////////////////////////////////////////////////////////
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/topology/DigitalSurface.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class DiscreteExteriorCalculusFactory
/**
 * Description of class 'DiscreteExteriorCalculusFactory' <p>
 * \brief Aim: This class provides static members to create DEC structures from various other DGtal structures.
 */

template <typename TLinearAlgebraBackend, typename TInteger = DGtal::int32_t>
class DiscreteExteriorCalculusFactory
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Create DEC structure from digital set.
     * DEC embedded and ambient dimensions are equal to digital set point dimension.
     * Set points get attached to primal n-cell <-> dual 0-cell.
     * @tparam TDigitalSet type of digital set passed as argument. must be a model of concepts::CDigitalSet.
     * @param set the set from which to build to DEC structure.
     * @param add_border add border to the computed structure.
     */
    template <typename TDigitalSet>
    static
    DiscreteExteriorCalculus<TDigitalSet::Point::dimension, TDigitalSet::Point::dimension, TLinearAlgebraBackend, TInteger>
    createFromDigitalSet(const TDigitalSet& set, const bool add_border = true);

    /**
     * Create DEC structure range of signed n-cells.
     * DEC embedded dimension is equal to n.
     * DEC ambient dimension is equal to n-cells kspace dimension.
     * n-cells get attached to primal n-cell <-> dual 0-cell.
     * @tparam TDigitalSurfaceContainer type of digital surface container. must be a model of concepts::CDigitalSurfaceContainer.
     * @param surface the digital surface from which to build to DEC structure.
     * @param add_border add border to the computed structure.
     */
    template <Dimension dim_embedded, typename TNSCellConstIterator>
    static
    DiscreteExteriorCalculus<dim_embedded, TNSCellConstIterator::value_type::Point::dimension, TLinearAlgebraBackend, TInteger>
    createFromNSCells(const TNSCellConstIterator begin, const TNSCellConstIterator end, const bool add_border = true);

    // ----------------------- Interface --------------------------------------
public:

    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

private:

    /**
     * Constructor.
     * Forbidden by default.
     */
    DiscreteExteriorCalculusFactory();


    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DiscreteExteriorCalculusFactory ( const DiscreteExteriorCalculusFactory & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DiscreteExteriorCalculusFactory & operator= ( const DiscreteExteriorCalculusFactory & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class DiscreteExteriorCalculusFactory

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/dec/DiscreteExteriorCalculusFactory.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DiscreteExteriorCalculusFactory_h

#undef DiscreteExteriorCalculusFactory_RECURSES
#endif // else defined(DiscreteExteriorCalculusFactory_RECURSES)
