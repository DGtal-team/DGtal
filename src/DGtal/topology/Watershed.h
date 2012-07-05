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
 * @file Watershed.h
 * @author Jérémy Gaillard (\c jeremy.gaillard@insa-lyon.fr )
 * 
 * @date 2012/06/06
 *
 * Header file for module Watershed.cpp
 *
 * This file is part of the DGtal library. TODO : not yet
 */

#if defined(Watershed_RECURSES)
#error Recursive header files inclusion detected in Watershed.h
#else // defined(Watershed_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Watershed_RECURSES

#if !defined Watershed_h
/** Prevents repeated inclusion of headers. */
#define Watershed_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/base/CowPtr.h"
// TODO : clean includes

//////////////////////////////////////////////////////////////////////////////

// namespace DGtal (not yet)
/////////////////////////////////////////////////////////////////////////////
// class Watershed
/**
 * Description of class 'Watershed' <p>
 * 
 * @brief Define utilites to transform a thickness-parametered image into a segmented image
 * using a Watershed algorithm
  TODO : better description
 */
// Template class Image
template < typename TGraph, typename TVertexMap/*typename TValue*/, class TLessFunctor >
class Watershed
{

  
  // ----------------------- Standard services ------------------------------
  public:
  //BOOST_CONCEPT_ASSERT(( CUndirectedSimpleLocalGraph<TGraph> ));
  typedef TGraph Graph;
  typedef typename TVertexMap::Value Value;
  typedef typename TGraph::Vertex Vertex;
  typedef set<Vertex>/*typename TGraph::VertexSet*/ VertexSet;
  template <typename TPair>
  struct PairComparison
  {
    public:
      PairComparison() {}
      bool
      operator() (const TPair& p1, const TPair& p2) const
      {
	TLessFunctor comp;
	return !comp.operator()(p1.second, p2.second);
      }
  };
  struct MapComparison
  {
    public:
      MapComparison() {}
      bool
      operator() (const Value& e1, const Value& e2) const
      {
	TLessFunctor comp;
	return comp.operator()(e1, e2);
      }
  };
  typedef /*typename TGraph::template VertexMap<TValue>::Type*/TVertexMap VertexMap;
  
  
  //template<typename Value> struct VertexMap {
  //  typedef typename TGraph::template VertexMap<Value>::map map;    
  //};
  
  // ----------------------- Interface --------------------------------------
  public:
  
    Watershed(const Graph & aGraph,  const VertexMap & values);
    
  /**
   * Return a segmented image where each point of the image contains the
   * segment number which he is associated to. TODO : rewrite, outdated
   * @param an image containing information about thickness
   * @return the segmented image
   */ 
    VertexMap segmentation();
    
    VertexMap segmentation_marker(const VertexSet &markerSet);
    
    VertexMap segmentation_stochastic(int N, int M);
    
    VertexMap segmentation_stochastic_heuristic(int N, int M);
  
  
    // ------------------------- Protected Datas ------------------------------
  protected:
    //CowPtr<ImageContainer> image;
    const VertexMap & myValueMap;
    CowPtr<VertexSet> mySet;
    const Graph & myGraph;
    
    // ------------------------- Private Datas --------------------------------
  private:
  
  
    // ------------------------- Hidden services ------------------------------
  protected:
    Value gaussianFilter(const VertexMap & vMap, const Vertex & v);

    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class Watershed

// } // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "Watershed.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Watershed_h

#undef Watershed_RECURSES
#endif // else defined(Watershed_RECURSES)
