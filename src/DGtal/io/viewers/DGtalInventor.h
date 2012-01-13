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
 * @file DGtalInventor.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/06
 *
 * Header file for module DGtalInventor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DGtalInventor_RECURSES)
#error Recursive header files inclusion detected in DGtalInventor.h
#else // defined(DGtalInventor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DGtalInventor_RECURSES

#if !defined DGtalInventor_h
/** Prevents repeated inclusion of headers. */
#define DGtalInventor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <map>
#include <set>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoNormalBinding.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTranslation.h>
#include "DGtal/io/viewers/IVViewer.h"
#include "DGtal/io/viewers/Lattice.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DGtalInventor
  /**
   * Description of template class 'DGtalInventor' <p> \brief Aim: A
   * stream object based on Open Inventor for exporting or displaying
   * DGtal objects.
   * 
   * Digital points, cells and objects may be outputed in that kind of
   * object. The user may then ask to view the object or simply export
   * its content.
   *
   * @tparam TSpace the type of digital space (should realize a CSpace).
   */
  template <typename TSpace>
  class DGtalInventor
  {
    // ----------------------- Concept checks ------------------------------
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));

    // ----------------------- Associated types ------------------------------
  public:
    typedef TSpace Space;
    typedef typename TSpace::Point Point;
    typedef typename TSpace::Dimension Dimension;
    typedef typename TSpace::Integer Integer;

    /**
     * The associated map type for storing possible modes used for
     * displaying for digital objects.
     */
    typedef std::map< std::string, std::string > ModeMapping;

    typedef SbVec3f Vec3f;
    typedef Vec3f Color;
    typedef std::map< Point, bool > CellSet;
    typedef std::map< Point, Vec3f > ColorMapping;
    typedef std::map< Point, Color > NormalMapping;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~DGtalInventor();

    /**
     * Constructor. Creates a default lattice for display.
     */
    DGtalInventor();

    /**
     * All further operations are immersed in Inventor 3D space with
     * the lattice [l].
     *
     * @param l the lattice space which should be of dimension
     * TSpace::dimension.
     */
    void setLattice( const Lattice<Space> & l );

    /**
     * Clears everything. The object is ready for a new visualization
     * and keeps its lattice.
     */
    void clear();

    /**
     * @param objectName the name of the object (generally obtained
     * with a 'object.className()').
     *
     * @return the current mode for the given object name or "" if no
     * specific mode has been set.
     */
    std::string getMode( const std::string & objectName ) const;

    // ----------------------- Graphics services ------------------------------
  public:

    /**
     * @param color the diffuse color.
     */
    void setDiffuseColor( const float* color );
    /**
     * @param color the diffuse color.
     */
    void setDiffuseColor( const Color & color );

    /**
     * Adds a cell for visualisation. Gives an optional normal.  Nb :
     * if a cell has dimension greater than 2, the boundary of the
     * cell is taken and this method is recursively called.
     *
     * @param c the cell to visualize (in Khalimsky coordinates).
     * @param n the (optional) normal to the cell as a 3D vector.
     */
    void drawCell( const Point & c, bool orient, const float* n = 0 );

    /**
     * Adds a Zn-point for visualisation. Gives an optional normal. 
     *
     * @param p the point to visualize.
     * @param n the (optional) normal to the cell as a 3D vector.
     */
    void drawPoint( const Point & c, const float* n = 0 );

    // ----------------------- Inventor methods --------------------------------
  public:

    /**
     * Generates the IV surface/lines/points with children rooted at
     * the SoGroup [result].
     *
     * @param result (returns) an updated IV node.
     */
    void generate( SoGroup* result ) const;

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
     * lattice for embedding of digital space.
     */
    Lattice<Space> myLattice;

    /**
     * May associate a current mode for a given class.
     * myModes[ "HyperRectDomain" ] = "Paving".
     *
     * Next display of a HyperRectDomain object will used the mode
     * "Paving".  Modes may only be used in objects implementing the
     * concept CDrawableWithBoard2D.
     */
    ModeMapping myModes;

    Color myDiffuseColor;
    CellSet myCells[ 3 ];
    ColorMapping myColors;
    NormalMapping myNormals;

    // ------------------------- Hidden services ------------------------------
  protected:


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DGtalInventor ( const DGtalInventor & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DGtalInventor & operator= ( const DGtalInventor & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DGtalInventor


  /**
   * Overloads 'operator<<' for displaying objects of class 'DGtalInventor'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DGtalInventor' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out, const DGtalInventor<TSpace> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/DGtalInventor.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DGtalInventor_h

#undef DGtalInventor_RECURSES
#endif // else defined(DGtalInventor_RECURSES)
