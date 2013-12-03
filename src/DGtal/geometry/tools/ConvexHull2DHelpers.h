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
 * @file ConvexHull2DHelpers.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/12/02
 *
 * Header file for module ConvexHull2DHelpers.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConvexHull2DHelpers_RECURSES)
#error Recursive header files inclusion detected in ConvexHull2DHelpers.h
#else // defined(ConvexHull2DHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConvexHull2DHelpers_RECURSES

#if !defined ConvexHull2DHelpers_h
/** Prevents repeated inclusion of headers. */
#define ConvexHull2DHelpers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "boost/utility.hpp"

#include "DGtal/base/Common.h"
#include "DGtal/base/IteratorTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   * @namespace ConvexHull2D gathers useful functions to compute 
   * and return the convex hull of a range of 2D points. 
   */
  namespace ConvexHull2D
  {

    /**
     * @brief Procedure that updates the hull when
     * an extra point @a aNewPoint is considered: 
     * the sets of the last three consecutive points, ie. @a aNewPoint
     * and the last two points of the container, should be oriented 
     * such that the predicate @a aPredicate returns 'true'. 
     * Otherwise, the last point of the container is removed and the
     * orientation is tested again until the predicate returns 'true'
     * (or the container is empty). 
     * @see grahamScan   
     * 
     * @param aContainer (returned) stack-like container
     * @param aNewPoint new point
     * @param aPredicate point predicate  
     * 
     * @tparam Container a model of 'back-pushable' sequence container 
     * @tparam Point a model of points    
     * @tparam Predicate a model of ternary predicate 
     */
    template <typename Container, typename Point, typename Predicate>
    void updateHull(Container& aContainer, const Point& aNewPoint, const Predicate& aPredicate)
    {
      Point Q = aContainer.back(); 
      aContainer.pop_back(); 
      if (aContainer.size() != 0) 
	{
	  Point P = aContainer.back(); 
	  while ( ( !aPredicate(P,Q,aNewPoint) )&&(aContainer.size() != 0) )
	    {
	      //remove Q
	      std::cerr << Q << " removed" << std::endl; 
	      Q = P; 
	      aContainer.pop_back(); 
	      if (aContainer.size() != 0) 
	        P = aContainer.back(); 
	    }
	  //add Q
	  aContainer.push_back(Q); 
	}
    }

    /**
     * @brief Procedure that retrieves the vertices of 
     * the hull of a range of 2D points [@e itb , @e ite )
     * and that stores them in an input container @e aContainer. 
     * @see grahamScan   
     * 
     * @param aContainer (returned) stack-like container
     * @param itb begin iterator
     * @param ite end iterator 
     * @param aPredicate predicate  
     * 
     * @tparam Container a model of 'back-pushable' sequence container 
     * @tparam ForwardIterator a model of forward iterator
     * @tparam Predicate a model of ternary predicate
     */
    template <typename Container, typename ForwardIterator, typename Predicate>
    void buildHull(Container& aContainer, 
		   const ForwardIterator& itb, const ForwardIterator& ite,
		   const Predicate& aPredicate)
    {
      //for all points
      for(ForwardIterator it = itb; it != ite; ++it)
	{
	  if(aContainer.size() < 2)
	    {
	      aContainer.push_back( *it ); 
	    }
	  else
	    {
	      //we update the hull so that the predicate returns 'true'
	      //for each sets of three consecutive points
	      updateHull(aContainer, *it, aPredicate); 
	      //add new point
	      aContainer.push_back( *it ); 
	    }
	}//end for all points
    }

    /**
     * @brief Procedure that retrieves the vertices
     * of the hull of a sorted list of 2D points
     * by the well-known Graham's scan.  
     * 
     * @param itb begin iterator
     * @param ite end iterator 
     * @param res output iterator used to export the retrieved points
     * @param aPredicate any ternary predicate  
     * 
     * @tparam ForwardIterator a model of forward iterator
     * @tparam OutputIterator a model of output iterator   
     * @tparam Predicate a model of ternary predicate
     */
    template <typename ForwardIterator, typename OutputIterator, typename Predicate>
    void openGrahamScan(const ForwardIterator& itb, const ForwardIterator& ite,  
			OutputIterator res, const Predicate& aPredicate)
    {
      typedef typename IteratorCirculatorTraits<ForwardIterator>::Value Point; 
      std::list<Point> container; 

      //hull computation
      buildHull(container, itb, ite, aPredicate); 

      //copy
      std::copy(container.begin(), container.end(), res); 
    }

    /**
     * @brief Procedure that retrieves the vertices
     * of the hull of a sorted circular list of 2D points
     * by the well-known Graham's scan.  
     * 
     * @param itb begin iterator
     * @param ite end iterator 
     * @param res output iterator used to export the retrieved points
     * @param aPredicate any ternary predicate  
     * 
     * @tparam ForwardIterator a model of forward iterator
     * @tparam OutputIterator a model of output iterator   
     * @tparam Predicate a model of ternary predicate
     */
    template <typename ForwardIterator, typename OutputIterator, typename Predicate>
    void closedGrahamScan(const ForwardIterator& itb, const ForwardIterator& ite,  
			  OutputIterator res, const Predicate& aPredicate)
    {
      typedef typename std::iterator_traits<ForwardIterator>::value_type Point; 
      std::list<Point> container; 

      //hull computation
      buildHull(container, itb, ite, aPredicate); 

      if (container.size() > 3)
	{
	  //we update the hull to take into account the starting point
	  updateHull(container, *container.begin(), aPredicate); 

	  //we update the hull to take into account the first vertices
	  Point last = container.back();  
	  typename std::list<Point>::iterator it = container.begin(); 
	  typename std::list<Point>::iterator itEnd = container.end();
	  ASSERT( it != itEnd ); //because the container has at least three points
	  ++it; //we advance the iterator until the predicate returns 'true'
	  while ( (it != itEnd) && 
	  	  ( !aPredicate( last, *boost::prior(it), *it ) ) ) 
	    ++it; 
	  //partial copy
	  std::copy( (--it), itEnd, res);  
	}
      else //full copy
	std::copy(container.begin(), container.end(), res); 

    }


  } // namespace convexHull2D


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/tools/ConvexHull2DHelpers.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConvexHull2DHelpers_h

#undef ConvexHull2DHelpers_RECURSES
#endif // else defined(ConvexHull2DHelpers_RECURSES)
