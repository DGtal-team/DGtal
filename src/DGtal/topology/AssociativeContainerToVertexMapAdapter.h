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
 * @file AssociativeContainerToVertexMapAdapter.h
 * @author Jérémy Gaillard (\c jeremy.gaillard@insa-lyon.fr )
 * Institut National des Sciences Appliquées - INSA, France
 *
 * @date 2012/07/11
 *
 * Header file for template class AssociativeContainerToVertexMapAdapter
 *
 * This file is part of the DGtal library.
 */

#if defined(AssociativeContainerToVertexMapAdapter_RECURSES)
#error Recursive header files inclusion detected in AssociativeContainerToVertexMapAdapter.h
#else // defined(AssociativeContainerToVertexMapAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AssociativeContainerToVertexMapAdapter_RECURSES

#if !defined AssociativeContainerToVertexMapAdapter_h
/** Prevents repeated inclusion of headers. */
#define AssociativeContainerToVertexMapAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include <../Lib/AQA/src/ntl-5.5.2/include/NTL/GF2XVec.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class AssociativeContainerToVertexMapAdapter
  /**
  Description of template class 'AssociativeContainerToVertexMapAdapter' <p> \brief
  Aim: This class adapts an associative container of the STL (such as map) to
  the CVertexMap concept
  
 
  @tparam TAssociativeContainer the type of the associative container.
 

   */
  template < typename TAssociativeContainer >
  class AssociativeContainerToVertexMapAdapter :
    TAssociativeContainer
  {
    // ----------------------- Associated types ------------------------------
  public:
    typedef AssociativeContainerToVertexMapAdapter<TAssociativeContainer> Self;
    typedef TAssociativeContainer Container;
    typedef typename Container::value_compare Compare;
    typedef typename Container::allocator_type Allocator;
    typedef typename Container::key_type Vertex;
    typedef typename Container::mapped_type Value;
    

    // Cannot check this since some types using it are incomplete.
    // BOOST_CONCEPT_ASSERT(( CUndirectedSimpleLocalGraph< Graph > ));
    // BOOST_CONCEPT_ASSERT(( CSet< MarkSet, Vertex > ));


    // ----------------------- Standard services ------------------------------
  public:

    AssociativeContainerToVertexMapAdapter() : Container() {}
    
    template <class InputIterator> AssociativeContainerToVertexMapAdapter( InputIterator first,
	InputIterator last, const Compare& comp = Compare(), const Allocator& alloc = Allocator() )
	: Container( first, last, comp, alloc ) {}
    
    AssociativeContainerToVertexMapAdapter( AssociativeContainerToVertexMapAdapter & other ) 
	: Container( other ) {}

    void setValue(Vertex v, Value val)
    {
      *this[v] = val;
    }
    
    Value operator()(Vertex v)
    {
      return find(v);
    }



    // ----------------------- Interface --------------------------------------
  public:


    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class AssociativeContainerToVertexMapAdapter


 

} // namespace DGtal



#endif // !defined AssociativeContainerToVertexMapAdapter_h

#undef AssociativeContainerToVertexMapAdapter_RECURSES
#endif // else defined(AssociativeContainerToVertexMapAdapter_RECURSES)
