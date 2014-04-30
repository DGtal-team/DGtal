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
 * @file AllSCellMap.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/27
 *
 * Header file for module AllSCellMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AllSCellMap_RECURSES)
#error Recursive header files inclusion detected in AllSCellMap.h
#else // defined(AllSCellMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AllSCellMap_RECURSES

#if !defined AllSCellMap_h
/** Prevents repeated inclusion of headers. */
#define AllSCellMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/base/Clone.h"
#if defined(WITH_ITK)
#include <DGtal/images/ImageSelector.h>
#include <DGtal/io/writers/ITKWriter.h>
#endif
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class AllSCellMap
  /**
   * Description of template class 'AllSCellMap' <p>
   * \brief Aim:
   * This containter holds data associated with DiscreteExteriorCalculus cells.
   *
   * @tparam TCalculus should be DiscreteExteriorCalculus.
   * @tparam TValue is type of value stored per cells.
   */
  template <typename TCalculus, typename TValue>
  class AllSCellMap : public std::map<typename TCalculus::SCell, TValue>
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TCalculus Calculus;
    typedef TValue Value;

    typedef typename Calculus::SCell SCell;
    typedef SCell Key;

    typedef typename std::map<Key, Value> Container;
    typedef typename Container::const_iterator ConstIterator;
    typedef typename Container::iterator Iterator;

    /**
     * Constructor.
     * @param calculus the discrete exterior calculus to use.
     */
    AllSCellMap(ConstAlias<Calculus> calculus);

    /**
     * Assignment.
     * @param scell_map the object to copy.
     * @return a reference on 'this'.
     */
    AllSCellMap& operator=(const AllSCellMap& scell_map);

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Pointer to const calculus.
     */
    const Calculus* myCalculus;

    /**
     * Get class name string "AllSCellMap".
     */
    std::string className() const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

#if defined(WITH_ITK)
    /**
     * Writes the object on an ITK image.
     * @param filename the name of the file written.
     * @param value_outside image value for non set order of cells.
     * @params value_inside_default image value for cells of correct order without values.
     */
    void writeITKImage(const std::string& filename, const Value& value_outside = -1, const Value& value_inside_default = 0) const;
#endif

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    AllSCellMap();

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class AllSCellMap

  /**
   * Overloads 'operator<<' for displaying objects of class 'AllSCellMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'AllSCellMap' to write.
   * @return the output stream after the writing.
   */
  template <typename Calculus, typename Value>
  std::ostream&
  operator<<(std::ostream& out, const AllSCellMap<Calculus, Value>& object);

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/dec/AllSCellMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AllSCellMap_h

#undef AllSCellMap_RECURSES
#endif // else defined(AllSCellMap_RECURSES)
