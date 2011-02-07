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
 * @file MaximalSegments.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/14
 *
 * Header file for module MaximalSegments.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MaximalSegments_RECURSES)
#error Recursive header files inclusion detected in MaximalSegments.h
#else // defined(MaximalSegments_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MaximalSegments_RECURSES

#if !defined MaximalSegments_h
/** Prevents repeated inclusion of headers. */
#define MaximalSegments_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
	
  /////////////////////////////////////////////////////////////////////////////
  // template class MaximalSegments
  /**
   * Description of template class 'MaximalSegments' <p>
   * \brief Aim: Computes the set of maximal segments of a sequence of 
   * points viewed as a digital curve into subsequences called segments,
   * which are maximal for inclusion (they cannot be included in other segments. 

   * This class is templated by 'TIterator', an iterator that can provide the
   * points (in order) of the digital curve, and by 'TSegment', the type of 
   * the segment (4-connected DSS, 8-connected DSS, thick segment, etc.)
   * The segment must have a method extend() taking as input what is returned
   * by the iterator (usually a point) and returning a boolean equal to TRUE
   * if the extension is possible and has been successfully performed and 
   * FALSE otherwise.
   
   * In the short example below, a contour stored as a Freeman chain is decomposed 
   * into 4-connected DSSs whose parameters are sent to the standard output.
   * TODO
   * @code 
   

  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<int,4> PrimitiveType;
  typedef FreemanChain<int> ContourType; 
	typedef MaximalSegments<ContourType::ConstIterator,PrimitiveType> DecompositionType;

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
  template <typename TSegment>
  class MaximalSegments
  {

	public: 

		typedef TSegment Segment;
		typedef typename TSegment::Iterator Iterator;


    // ----------------------- Standard services ------------------------------
  public:



    /**
     * This class is an iterator on a digital curve 
     * storing the current segment.
     */
    class ConstIterator
    {


			   // ------------------------- private data -----------------------
    private:
			

      /**
       * Pointer to the cover of maximal segments
       */
			MaximalSegments<TSegment> *myCov;

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
      
      /**
       * A flag equal to FALSE if the current segment
       * lies between the begin and the end iterator 
       * of the digital curve, TRUE otherwise. 
       * Nb: always FALSE if the digital curve is 
       * processed as open. 
       */
      bool  myFlag;

      // ------------------------- Standard services -----------------------
    public:
       friend class MaximalSegments<TSegment>;
			   
      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aCov a greedy decomposition of a digital curve
       * @param aBack an iterator at the back of the first segment
       */
      ConstIterator( MaximalSegments<TSegment> *aCov,
										 const typename TSegment::Iterator& aBack,
										 const TSegment& aSegment);


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
       * Goes to the next maximal segment on the digital curve (if possible).
       * Nb: complexity in O(n).
       */
      ConstIterator& operator++();
      
      /**
       * Retrieves the first maximal segment found on the digital curve.
       * Nb: complexity in O(n).
       */
      void firstMaximalSegment();

      /**
       * Goes to the next maximal segment on the digital curve (if possible).
       * Nb: complexity in O(n).
       */
      void nextMaximalSegment();


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

      // ------------------------- hidden services -------------------------
 
			private: 

      /**
       * @param anIt a reference of a given iterator
       * @param aBegin begin iterator
       * @param aEnd end iterator
       * @return anIt incremented (but equal to begin instead of end)

       */
      Iterator incrementInLoop(Iterator& anIt, const Iterator& aBegin, const Iterator& aEnd);

      /**
       * @param anIt a reference of a given iterator
       * @param aFirst an iterator pointing at the first point
       * @param aLast an iterator pointing at the last point
       * @return anIt decremented (but equal to aLast if anIt equal to aFirst)

       */
      Iterator decrementInLoop(Iterator& anIt, const Iterator& aFirst, const Iterator& aLast);

      
    };


    // ----------------------- Interface --------------------------------------
  public:


    /**
     * Constructor.
		 * Nb: The digital curve is decompose as a closed one by default.
     * @param aBegin, begin iterator on a digital curve
     * @param aEnd, end iterator on a digital curve
     * @param aSegment, a segment computer
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    MaximalSegments(const Iterator& aBegin, 
                    const Iterator& aEnd, 
                    const Segment& aSegment, 
                    const bool& aFlag);

    /**
     * Destructor.
     */
    ~MaximalSegments();

    /**
     * Iterator service.
     * @return an iterator pointing on the first segment of a digital curve.
     */
    typename MaximalSegments::ConstIterator begin();

    /**
     * Iterator service.
     * @return an iterator pointing after the last segment of a digital curve.
     */
    typename MaximalSegments::ConstIterator end();


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
     * begin iterator (pointing at the first point)
     */
		Iterator myBegin;
    /**
     * iterator pointing at the last point
     */
		Iterator myLast;

    /**
     * end iterator (pointing after the last point)
     */
		Iterator myEnd;

    /**
     * back iterator (first point) 
     * of the first maximal segment
     */
		Iterator myFirstMaximalSegmentBack;

    /**
     * boolean equal to TRUE if the digital curve
     * has to be processed as closed, FALSE otherwise
     */
		bool isClosed;

    /**
     * a segment Computer
     */
		Segment mySegment;

    // ------------------------- Hidden services ------------------------------


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    MaximalSegments ( const MaximalSegments & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    MaximalSegments & operator= ( const MaximalSegments & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class MaximalSegments


  /**
   * Overloads 'operator<<' for displaying objects of class 'MaximalSegments'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'MaximalSegments' to write.
   * @return the output stream after the writing.
   */
  template <typename TSegment>
  std::ostream&
  operator<< ( std::ostream & out, const MaximalSegments<TSegment> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/MaximalSegments.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MaximalSegments_h

#undef MaximalSegments_RECURSES
#endif // else defined(MaximalSegments_RECURSES)
