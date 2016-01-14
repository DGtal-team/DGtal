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
 * @file ParDirCollapse.h
 * @author Mohamad ONAYSSI (\c mohamad.onayssi@edu.esiee.fr )
 * @author Bibiana MARTINEZ (\c bibiana.martinez@edu.esiee.fr )
 * @author Mohamed MELLOULI (\c mohamed.mellouli@edu.esiee.fr )
 * ESIEE Paris
 *
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 *
 * @date 2015/12/22
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(ParDirCollapse_RECURSES)
#error Recursive header files inclusion detected in ParDirCollapse.h
#else // defined(ParDirCollapse_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ParDirCollapse_RECURSES

#if !defined ParDirCollapse_h
/** Prevents repeated inclusion of headers. */
#define ParDirCollapse_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include <kernel/PointVector.h>
// Cellular grid
#include "DGtal/topology/CubicalComplex.h"
#include "DGtal/topology/CubicalComplexFunctions.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// class ParDirCollapse
/**
 * Description of class 'ParDirCollapse' <p>
 * \brief Aim: Implements thinning algorithm in cubical complexes.
 * Paper: Chaussard, J. and Couprie, M., Surface Thinning in 3D Cubical Complexes, Combinatorial Image Analysis, (2009)
 * @tparam CC cubical complex.
 */
template < typename CC >
class ParDirCollapse
{
    // ----------------------- Types ------------------------------
public:
    /// Any model of concepts::CCellularGridSpaceND, i.e. a type that models a Khalimsky space.
    typedef typename CC::KSpace KSpace;
    /// Type of Nd integer vector
    typedef typename Vector<KSpace::dimension, int > Vector;
    /// Type of cells in Khalimsky space
    typedef typename KSpace::Cell Cell;
    /// Type of collection of cells
    typedef typename KSpace::Cells Cells;
    /// Type of const iterator over a map of cells.
    typedef typename CC::CellMapConstIterator CellMapConstIterator;

    // ----------------------- Standard services ------------------------------
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ----------------------- Interface --------------------------------------
public:
    /**
     * Constructor.
     * @param k -- const reference to Khalimsky space
     */
    ParDirCollapse( const KSpace & k);

     /**
     * Constructor.
     * @param pComplex -- Cubical complex
     */
    void attach ( Alias<CC> pComplex );

     /**
     * @param iterations -- number of iterations
     * @return total number of removed cells.
     */
     unsigned int eval (int iterations );

    /**
     * Extension of basic algorithm which preserve KSpace::dimension - 1 faces which are not included
     * in any KSpace::dimension cells.
     */
    void collapseSurface();

    /**
     * Extension of basic algorithm which preserve KSpace::dimension - 1 faces which are not included
     * in any KSpace::dimension cells. Moreover, cells to be kept have not to be collapsible.
     */
    void collapseIsthmus();

    // ------------------------- Internals ------------------------------------
private:
    /**
     * Calculate an orientation of a freepair.
     * @param F -- cell of a dimension one lower than G.
     * @param G -- cell of a dimension one higher than F.
     */
    int getOrientation ( const Cell& F, const Cell& G );

     /**
     * Calculate an orientation of a freepair.
     * @param F -- cell of a dimension one lower than G.
     * @param G -- cell of a dimension one higher than F.
     */
    int getDirection ( const Cell& F, const Cell& G );

    // ------------------------- Hidden services ------------------------------
protected:
    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ParDirCollapse();

private:
    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ParDirCollapse ( const ParDirCollapse & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ParDirCollapse & operator= ( const ParDirCollapse & other );


    // ------------------------- Private Datas --------------------------------
private:

    /// Reference to Khalimsky space in which a given complex is embedded.
    const KSpace& K;
    // Pointer to complex.
    CC * complex;

}; // end of class ParDirCollapse

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/topology/ParDirCollapse.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ParDirCollapse_h

#undef ParDirCollapse_RECURSES
#endif // else defined(ParDirCollapse_RECURSES)
