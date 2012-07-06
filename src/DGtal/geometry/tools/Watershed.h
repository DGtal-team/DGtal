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
#include "DGtal/topology/CUndirectedSimpleGraph.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal {
/////////////////////////////////////////////////////////////////////////////
// class Watershed
/**
 * Description of class 'Watershed' <p>
 * 
 * @brief Define utilites to perform a segmentation of a value-mapped graph 
 * using watershed algorithms
 * 
 * 
 * @tparam TGraph any model of CUndirectedSimpleGraph.
 * @tparam TVertexMap any model of C?.
 * @tparam TLessFunctor a functor defining the less comparison
 * 
 */
// Template class Image
template < typename TGraph, typename TVertexMap, class TLessFunctor = std::less<typename TVertexMap::Value> >
class Watershed
{

  
  // ----------------------- Standard services ------------------------------
  public:
  BOOST_CONCEPT_ASSERT(( CUndirectedSimpleGraph<TGraph> ));
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
  
  /**
   * Constructor
   * 
   * @param aGraph graph on which the segmentation is made
   * 
   * @param values a data structure which binds a value for each graph vertex
   * 
   */
    Watershed(const Graph & aGraph, const VertexMap & values);
    
  /**
   * Return a segmented image where each point of the image contains the
   * segment number which he is associated to.
   * 
   * @return the segmented image
   */ 
    VertexMap segmentation();
    
  /**
   * Return a segmented image where each point of the image contains the
   * segment number which he is associated to. The watershed uses a 
   * set of markers instead of local minima as water sources.
   * 
   * @tparam TVertexSet any model of CSinglePassRange on vertices
   * 
   * @param markerSet a set of vertices contained by the graph
   * 
   * @return the segmented image
   */ 
    template<typename TVertexSet> VertexMap segmentationFromMarkers(const TVertexSet &markerSet);
    
  /**
   * Return an image where each point contains a value representing
   * the probability of the presence of a watershed. The higher the value, the
   * bigger the probability.
   * This method executes several watershed based on a random set of markers 
   * to deduce the probabilistic function.
   * 
   * @param N the number of markers placed at each iteration
   * 
   * @param M the number of iterations
   * 
   * @return the watershed probabilistic function
   */ 
    VertexMap segmentationStochastic(int N, int M);
        
  /**
   * Return a segmented image where each point of the image contains the
   * segment number which he is associated to. The watershed is based on
   * the probabilistic function computed by segmentation_stochastic(N, M).
   * 
   * @param N the number of markers placed at each iteration
   * 
   * @param M the number of iterations
   * 
   * @return the watershed probabilistic function
   */     
    VertexMap segmentationStochasticHeuristic(int N, int M);
    
    inline
    Value getWatershedValue()
    {
      return WSHED;
    }
  
  
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /**
     * Reference to the graph given in the constructor
     */
    const Graph & myGraph;
    
    /**
     * Reference to the data structure containing the value mapping of myGraph
     */
    const VertexMap & myValueMap;
    
    /**
     * Value assigned to a watershed
     */
    Value WSHED;
    
    // ------------------------- Private Datas --------------------------------
  private:
  
  
    // ------------------------- Hidden services ------------------------------
  protected:
    Value gaussianFilter(const VertexMap & vMap, const Vertex & v);
    Watershed();
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class Watershed

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "Watershed.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Watershed_h

#undef Watershed_RECURSES
#endif // else defined(Watershed_RECURSES)
