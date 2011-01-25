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
 * @file GreedyDecomposition.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/25
 *
 * Header file for module GreedyDecomposition.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GreedyDecomposition_RECURSES)
#error Recursive header files inclusion detected in GreedyDecomposition.h
#else // defined(GreedyDecomposition_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GreedyDecomposition_RECURSES

#if !defined GreedyDecomposition_h
/** Prevents repeated inclusion of headers. */
#define GreedyDecomposition_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
	
  /////////////////////////////////////////////////////////////////////////////
  // template class GreedyDecomposition
  /**
   * Description of template class 'GreedyDecomposition' <p>
   * \brief Aim: Computes the greedy decomposition of a sequence of 
   * points viewed as a digital curve into subsequences called segments
   * (the last point of a given segment is the first point of the next segment).

   * This class is templated by 'TIterator', an iterator that can provide the
   * points (in order) of the digital curve, and by 'TSegment', the type of 
   * the segment (4-connected DSS, 8-connected DSS, thick segment, etc.)
   * The segment must have a method extend() taking as input what is returned
   * by the iterator (usually a point) and returning a boolean equal to TRUE
   * if the extension is possible and has been successfully performed and 
   * FALSE otherwise.
   
   * In the short example below, a contour stored as a Freeman chain is decomposed 
   * into 4-connected DSSs whose parameters are sent to the standard output.
   * @code 
   

  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<int,4> PrimitiveType;
  typedef FreemanChain<int> ContourType; 
	typedef GreedyDecomposition<ContourType::ConstIterator,PrimitiveType> DecompositionType;

	//A contour stored as a Freeman chain
  std::string filename = testPath + "samples/manche.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);

  //Greedy segmentation of the contour
  DecompositionType theDecomposition(theContour.begin(), theContour.end());
  DecompositionType::ConstIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
    PrimitiveType segment(*i); 
    std::cout << segment << std::endl;	//standard output  
  } 

   * @endcode
   */
  template <typename TIterator, typename TSegment>
  class GreedyDecomposition
  {

	public: 
		typedef TIterator Iterator;
		typedef TSegment Segment;

    // ----------------------- Standard services ------------------------------
  public:



    /**
     * This class is an iterator on a digital curve 
     * storing the current segment.
     */
    class ConstIterator
    {
      	   
			   // ------------------------- data -----------------------
    private:

      /**
       * Pointer to the decomposition
       */
			const GreedyDecomposition<TIterator,TSegment> *myDec;


      /**
       * An iterator of the digital curve  
       * at the front of the current segment
       */
      Iterator myFront;

      /**
       * An iterator of the contour 
       * at the back of the current segment
       */
      Iterator myBack;


      /**
       * The current segment
       */
      Segment  mySegment;
      


      // ------------------------- Standard services -----------------------
    public:
       friend class GreedyDecomposition<TIterator,TSegment>;
			   


      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aDec a greedy decomposition of a digital curve
       * @param aBack an iterator at the back of the first segment
       */
      ConstIterator( const GreedyDecomposition<TIterator,TSegment> *aDec,
		     const TIterator& aBack);


      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      ConstIterator( const ConstIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      ConstIterator& operator=( const ConstIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~ConstIterator();
    
      // ------------------------- iteration services -------------------------
    public:
      
      /**
       * @return the current segment
       */
      Segment operator*() const;

      /**
       * @return the current segment.
       */
      Segment get() const;

      /**
       * Pre-increment.
       * Goes to the next segment on the contour (if possible).
       * Nb: complexity in O(n).
       */
      ConstIterator& operator++();
      
      /**
       * Goes to the next segment on the contour (if possible).
       * Nb: complexity in O(n).
       */
      void next();


      /**
       * @return an iterator of a digital curve
       * at the front of the segment.
       */
      const Iterator getFront() const;

      /**
       * @return an iterator of a digital curve
       * at the back of the segment.
       */
      const Iterator getBack() const;

      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with 
       *
       * @return 'true' if their current positions coincide.
       * (same front and back iterators)
       */
      bool operator==( const ConstIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with 
       *
       * @return 'true' if their current positions differs.
       * (different front and back iterators)
       */
      bool operator!=( const ConstIterator & aOther ) const;

    // ----------------------- hidden services --------------------------------------

			private: 

      /**
       * Computes the longest possible segment from 
       * two consecutive points.
       * Nb: complexity in O(n).
       */
      void longestSegment();
      
    };


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor.
		 * Nb: The digital curve is decompose as a closed one by default.
     * @param aBegin, begin iterator on a digital curve
     * @param aEnd, end iterator on a digital curve
     */
    GreedyDecomposition(const Iterator& aBegin, const Iterator& aEnd);

    /**
     * Constructor.
		 * Nb: The digital curve is decompose as a closed one by default.
     * @param aBegin, begin iterator on a digital curve
     * @param aEnd, end iterator on a digital curve
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    GreedyDecomposition(const Iterator& aBegin, const Iterator& aEnd, const bool& aFlag);

    /**
     * Destructor.
     */
    ~GreedyDecomposition();

    /**
     * Iterator service.
     * @return an iterator pointing on the first segment of a digital curve.
     */
    typename GreedyDecomposition::ConstIterator begin() const;

    /**
     * Iterator service.
     * @return an iterator pointing after the last segment of a digital curve.
     */
    typename GreedyDecomposition::ConstIterator end() const;


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

		Iterator myBegin, myEnd;

		bool isClosed;

    // ------------------------- Hidden services ------------------------------


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GreedyDecomposition ( const GreedyDecomposition & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GreedyDecomposition & operator= ( const GreedyDecomposition & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class GreedyDecomposition


  /**
   * Overloads 'operator<<' for displaying objects of class 'GreedyDecomposition'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GreedyDecomposition' to write.
   * @return the output stream after the writing.
   */
  template <typename Contour, typename Primitive>
  std::ostream&
  operator<< ( std::ostream & out, const GreedyDecomposition<Contour, Primitive> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/GreedyDecomposition.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GreedyDecomposition_h

#undef GreedyDecomposition_RECURSES
#endif // else defined(GreedyDecomposition_RECURSES)
