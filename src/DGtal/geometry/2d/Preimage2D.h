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
//LICENSE-END
#pragma once

/**
 * @file Preimage2D.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/26
 *
 * Header file for module Preimage2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Preimage2D_RECURSES)
#error Recursive header files inclusion detected in Preimage2D.h
#else // defined(Preimage2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Preimage2D_RECURSES

#if !defined Preimage2D_h
/** Prevents repeated inclusion of headers. */
#define Preimage2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Common.h"
#include "DGtal/utils/OpInSTLContainers.h"

#include "DGtal/geometry/2d/Point2ShapePredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Preimage2D
  /**
   * Description of template class 'Preimage2D' <p>
   * \brief Aim: Computes the preimage of the 2D Euclidean shapes 
   * crossing a sequence of straigth segments in linear-time
   * according to the algorithm of O'Rourke (1981). 

   * The straight
   * segment i is described by its two end points Pi and Qi.
   * The set of shapes considered here are those that 
   * can be uniquely defined by two points and that separate 
   * the 2D plane into two disjoint parts (e.g. straight lines, 
   * circles passing through a given point). Consequently, the 
   * points Pi and the points Qi are assumed to lie in either 
   * side of the shape. 
   * Nb: The user of this class has to decide from its input set 
   * of segments and the shape used whether a linear-time algorithm 
   * is possible or not. 
   * (if yes - e.g. preimage of straight lines crossing a set of 
   * vertical segments of increasing x-coordinate - this algorithm 
   * will return the right output). 
   * @code 
   
   typedef int Coordinate;
   typedef PointVector<2, Coordinate> Point;
   typedef StraightLine<Coordinate> StraightLine;
   typedef Preimage2D<StraightLine> Preimage2D;
   
   // Set input data segments as two vectors of endpoints.   
   std::vector<Point> P, Q;
   
   Q.push_back(Point(0, 10));
   Q.push_back(Point(1, 11));
   Q.push_back(Point(2, 11));
   Q.push_back(Point(3, 12));
   
   P.push_back(Point(0, 12));
   P.push_back(Point(1, 13));
   P.push_back(Point(2, 13));
   P.push_back(Point(3, 14));
   
   // Initialization 
   int i = 0;
   Preimage2D thePreimage(bInf.at(i), bSup.at(i));
   
   // Incremental computation of the preimage
   while ( (i < n) &&
   (thePreimage.addFront(bInf.at(i), bSup.at(i))) )
   {
     i++;
   }
   std::cout << thePreimage << std::endl;
   
   * @endcode
   */
  template <typename Shape>
  class Preimage2D
  {


    // ----------------------- Types ------------------------------
  public:

		typedef typename Shape::Coordinate CoordinateType;
		typedef DGtal::PointVector<2,CoordinateType> Point;
		typedef DGtal::PointVector<2,CoordinateType> Vector;

	private:

		//container of points
		typedef std::list<Point> Container;
		//Iterators on the container
		typedef typename std::list<Point>::iterator ForwardIterator;
		typedef typename std::list<Point>::reverse_iterator BackwardIterator;
		typedef typename std::list<Point>::const_iterator ConstForwardIterator;
		typedef typename std::list<Point>::const_reverse_iterator ConstBackwardIterator;

		//Predicates used to decide whether the preimage
    //has to be updated or not
		typedef Point2ShapePredicate<Shape,false,true> 
			PHullBackQHullFrontPred; 
		typedef Point2ShapePredicate<Shape,true,true> 
			QHullBackPHullFrontPred; 
		//Predicates used to update the hulls
		typedef Point2ShapePredicate<Shape,true,false> 
			PHullUpdateForPAddingPred; 
		typedef Point2ShapePredicate<Shape,false,false> 
			QHullUpdateForQAddingPred; 
		typedef Point2ShapePredicate<Shape,false,false> 
			QHullUpdateForPAddingPred; 
		typedef Point2ShapePredicate<Shape,true,false> 
			PHullUpdateForQAddingPred; 

    


    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * \param firstPoint, secondPoint, the two end points of 
     * the first straight segment
     */
    Preimage2D(const Point & firstPoint, const Point & secondPoint);

    /**
     * Destructor. Does nothing.
     */
    ~Preimage2D();

    /**
		 * Updates the current preimage with 
     * the constraints involved by the two 
     * end points of a new segment
     * (adding to the front of the sequence of 
     * segments with respect to the scan orientaion
     * e.g. back => seg1 => ... segn => front) 
		 * Nb: in O(n)
     * @param aP, aQ, 
		 * the two ends of the new straight segment 
	   * assumed to lie on either side of the shapes. 
     * @return 'false' if the updated preimage is empty, 
     * 'true' otherwise.
     */
    bool addFront(const Point & aP, const Point & aQ);


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

//------------------ display -------------------------------
    /**
     * Draw the preimage
     * @param board the output board where the object is drawn.
     * @tparam Functor a Functor to specialize the Board style
     */
    template<typename Functor>
      void selfDraw( LibBoard::Board & board ) const;


    /**
     * Draw the preimage on a LiBoard board
     * @param board the output board where the object is drawn.
     * @tparam Functor a Functor to specialize the Board style
     */
    void selfDraw( LibBoard::Board & board ) const
      {
				selfDraw<selfDrawStyle>(board);
      }

private:

   /** 
     * Default Style Functor for drawing
     * 
     * @param aBoard 
     */

    struct selfDrawStyle
    {
      selfDrawStyle(LibBoard::Board & aBoard) 
      {
				aBoard.setPenColor(LibBoard::Color::Red);
      }
    };


    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

		//lists of the vertices of the preimage
    //coorresponding to the points Pi and Qi
		Container myPHull, myQHull;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Preimage2D();

  private:

    /**
		 * Updates the current preimage
		 * Nb: in O(n)
     * @param 
     * aPoint a new vertex of the preimage,
     * aContainer the container to be updated,
     * anIterator an iterator to its front (resp. back)
     * anEndIterator an iterator pointing after its back 
		 * (resp. before its front). 
     */
		template <typename Iterator, typename Predicate>
    void update(const Point & aPoint, 
								Container & aContainer,
								Iterator & anIterator,
								const Iterator & anEndIterator);



    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Preimage2D ( const Preimage2D & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Preimage2D & operator= ( const Preimage2D & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Preimage2D


  /**
   * Overloads 'operator<<' for displaying objects of class 'Preimage2D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Preimage2D' to write.
   * @return the output stream after the writing.
   */
   template <typename Shape>
  std::ostream&
  operator<< ( std::ostream & out, const Preimage2D<Shape> & object );

//////////////////////////////////////////////////////////////////////////////
// generic classes for erasure in STL lists
// that cannot be performed with reverse_iterator
/*
		//default
		template <typename Container, typename Iterator>
		struct ErasureInSTLContainers 
		{
			static Iterator erase(Container& aContainer,Iterator& anIterator) 
			{
				return aContainer.erase(anIterator);
			}
		};

		//specialisation for reverse_iterator
		template <typename Container>
		struct ErasureInSTLContainers<
			Container, 
			std::reverse_iterator<typename Container::iterator> > 
		{
			typedef std::reverse_iterator<typename Container::iterator> reverseIterator;
			static reverseIterator erase(
									Container& aContainer,
									reverseIterator& anIterator) 
			{
				aContainer.erase((++anIterator).base());
				return anIterator;
			}
		};
*/
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/Preimage2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Preimage2D_h

#undef Preimage2D_RECURSES
#endif // else defined(Preimage2D_RECURSES)
