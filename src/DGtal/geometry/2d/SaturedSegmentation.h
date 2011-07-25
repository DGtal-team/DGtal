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
 * @file SaturedSegmentation.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/07/21
 *
 * Header file for module SaturedSegmentation.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SaturedSegmentation_RECURSES)
#error Recursive header files inclusion detected in SaturedSegmentation.h
#else // defined(SaturedSegmentation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SaturedSegmentation_RECURSES

#if !defined SaturedSegmentation_h
/** Prevents repeated inclusion of headers. */
#define SaturedSegmentation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/geometry/2d/SegmentComputerTraits.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
	
  /////////////////////////////////////////////////////////////////////////////
  // template class SaturedSegmentation
  /**
   * Description of template class 'SaturedSegmentation' <p>
   * \brief Aim: Computes the satured segmentation, that is
   * the whole set of maximal segments within a range given by 
   * a pair of ConstIterators (maximal segments are segments
   * that cannot be included in greater segments). 
   *
   * This class is a model of CSegmentation.
   * 
   * @tparam TSegmentComputer a model of CSegmentComputer
   * (an online algorithm for the recognition of some segment). 
   *
   * In the short example below, a digital curve stored in a STL vector
   * is decomposed into maximal 8-connected DSSs whose parameters are sent to 
   * the standard output.
   * @code 
   
  //types definition
  typedef PointVector<2,int> Point;
  typedef std::vector<Point> Range;
  typedef Range::const_iterator ConstIterator;
  typedef ArithmeticalDSS<ConstIterator,int,8> SegmentComputer;
	typedef SaturedSegmentation<SegmentComputer> Segmentation;

	//input points
	Range curve;
	curve.push_back(Point(1,1));
	curve.push_back(Point(2,1));
	curve.push_back(Point(3,2));
	curve.push_back(Point(4,2));
	curve.push_back(Point(5,2));
	curve.push_back(Point(6,2));
	curve.push_back(Point(7,2));
	curve.push_back(Point(8,1));
	curve.push_back(Point(9,1));

  //Segmentation
	SegmentComputer recognitionAlgorithm;
  Segmentation theSegmentation(curve.begin(), curve.end(), recognitionAlgorithm);
				 
  Segmentation::SegmentComputerIterator i = theSegmentation.begin();
  Segmentation::SegmentComputerIterator end = theSegmentation.end();
  for ( ; i != end; ++i) {
		SegmentComputer current(*i);
		trace.info() << current << std::endl;	//standard output
  } 

   * @endcode
   *
   * If you want to get the satured segmentation of a part of the 
   * digital curve (not the whole digital curve), you can give 
   * the range to process as a pair of iterators when calling 
   * the setSubRange() method as follow: 
   * @code 
  theSegmentation.setSubRange(beginIt, endIt);
   * @endcode   
   *
   * Obviously, [beginIt, endIt) has to be a valid range included
   * in the wider range [curve.begin(), curve.end()). 
   *
   * Moreover, a part of a digital curve may be processed either
   * as an independant (open) digital curve or as a part whose 
   * segmentation at the ends depends of the underlying digital 
   * curve. That's why several processing modes are available
   * for both the beginning and the ending of the digital curve.
   * TODO 
   * - "Truncate" (default), the extension of the last maximal 
   * segment (and the segmentation) stops just before endIt.
   * - "First", "MostCentered", "Last", the first, most centered,
   * last maximal segment containing the last element is the  
   * last maximal segment of the segmentation. 
   * 
   * In order to set a mode (before getting a SegmentComputerIterator),
   * use the setMode() method as follow: 
   * @code 
  theSegmentation.setMode("First","Last");
   * @endcode  
   * Note that the default mode will be used for any unknown modes.  
   */

  template <typename TSegmentComputer>
  class SaturedSegmentation
  {

	public: 

		typedef TSegmentComputer SegmentComputer;
		typedef typename SegmentComputer::ConstIterator ConstIterator;

	private: 

		typedef typename TSegmentComputer::Reverse ReverseSegmentComputer;
		typedef typename ReverseSegmentComputer::ConstIterator ConstReverseIterator;

    // ----------------------- Standard services ------------------------------
  public:



    /**
     * This class is an iterator storing the current 'SegmentComputer'.
     */
    class SegmentComputerIterator
    {

			   // ------------------------- inner Types -----------------------

    public: 
		  typedef typename SaturedSegmentation::SegmentComputer SegmentComputer;
		  typedef typename SegmentComputer::ConstIterator ConstIterator;

			   // ------------------------- data -----------------------
    private:

      /**
       * Pointer to the segmentation
       */
			const SaturedSegmentation<TSegmentComputer> *myS;

      /**
       * The current segment
       */
      SegmentComputer  mySegmentComputer;
      

      /**
       * A flag equal to TRUE if the current segment
       * intersects the next one, FALSE otherwise 
       * (and FALSE if the current segment is the last one) 
       */
      bool  myFlagIntersectNext;

      /**
       * A flag equal to TRUE if the current segment
       * intersects the previous one, FALSE otherwise 
       * (and FALSE if the current segment is the first one) 
       */
      bool  myFlagIntersectPrevious;

      /**
       * A flag equal to TRUE if *this has reached the end, FALSE otherwise 
       */
      bool  myFlagIsLast;

      /**
       * A flag equal to TRUE if *this is valid, FALSE otherwise 
       */
      bool  myFlagIsValid;



      // ------------------------- Standard services -----------------------
    public:
       friend class SaturedSegmentation<TSegmentComputer>;
			   


      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aSegmentation, the object that knows the range bounds
       * @param aSegmentComputer, an online segment recognition algorithm
       * @param aFlag, 'true' to build a valid object, 'false' otherwise
       */
      SegmentComputerIterator( const SaturedSegmentation<TSegmentComputer> *aSegmentation,
				 const TSegmentComputer& aSegmentComputer,
         const bool& aFlag );


      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      SegmentComputerIterator( const SegmentComputerIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      SegmentComputerIterator& operator=( const SegmentComputerIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~SegmentComputerIterator();

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const { return myFlagIsValid; }
    
      // ------------------------- iteration services -------------------------
    public:
      
      /**
       * @return the current segment
       */
      const SegmentComputer& operator*() const;

      /**
       * @return the current segment.
       */
      SegmentComputer get() const;

      /**
       * @return the pointer to the current segment
       */
      const SegmentComputer* operator->() const;

      /**
       * Pre-increment.
       * Goes to the next maximal segment (if possible).
       * Nb: complexity in O(n).
       */
      SegmentComputerIterator& operator++();
      
      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with 
       *
       * @return 'true' if their current positions coincide.
       * (same front and back iterators)
       */
      bool operator==( const SegmentComputerIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with 
       *
       * @return 'true' if their current positions differs.
       * (different front and back iterators)
       */
      bool operator!=( const SegmentComputerIterator & aOther ) const;

    // ----------------------- accessors --------------------------------------

      /**
       * @return TRUE if the current segment intersects
			 * the next one, FALSE otherwise.
       */
      const bool intersectNext() const;

      /**
       * @return TRUE if the current segment intersects
			 * the previous one, FALSE otherwise.
       */
      const bool intersectPrevious() const;

      /**
       * @return begin iterator on the segment.
       */
      const ConstIterator begin() const;

      /**
       * @return end iterator on the segment.
       */
      const ConstIterator end() const;

    // ----------------------- hidden services --------------------------------------

			private: 

      
      /**
       * Checks if the current segment intersects the next one (if exists).
       * @param it a given iterator
       */
      bool doesIntersectNext(const ConstIterator& it);

      /**
       * Computes the longest possible segment from [it]
       * @param it, a given iterator
       * Nb: complexity in O(n).
       */
      template <typename SC>
      void longestSegment(SC& s, 
                         const typename SC::ConstIterator& i, 
                         const typename SC::ConstIterator& end);

////////////////////////////////////////////////////////// next
      /**
       * Goes to the next maximal segment (if possible).
       * Nb: complexity in O(n).
       */
      void nextMaximalSegment();

      /**
       * Goes to the next maximal segment (if possible)
       * using only the extend() method. 
       * @param i, assumed to be the end iterator of the current segment
       * Nb: linear complexity in the range size
       */
      void nextMaximalSegment(const ConstIterator& i, ForwardSegmentComputer);
      /**
       * Goes to the next maximal segment (if possible)
       * using the extendOppositeEnd() and extend() methods. 
       * @param i, assumed to be the end iterator of the current segment
       * Nb: linear complexity in the range size
       */
      void nextMaximalSegment(const ConstIterator& i, BidirectionalSegmentComputer);
      /**
       * Goes to the next maximal segment (if possible)
       * using the retract(), isExtendable() and extend() methods. 
       * @param i, assumed to be the end iterator of the current segment
       * Nb: linear complexity in the range size
       */
      void nextMaximalSegment(const ConstIterator& i, DynamicSegmentComputer);
      /**
       * Same as       
      void nextMaximalSegment(const ConstIterator i, DynamicSegmentComputer);
       */
      void nextMaximalSegment(const ConstIterator& i, DynamicBidirectionalSegmentComputer);

////////////////////////////////////////////////////////// first
      /**
       * Computes the first maximal segment passing throught i
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void firstMaximalSegment(const ConstIterator& i);

      /**
       * Computes the first maximal segment passing throught i
       * using only the extend() method. 
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void firstMaximalSegment(const ConstIterator& i, ForwardSegmentComputer);

      /**
       * Computes the first maximal segment passing throught i
       * using the extendOppositeEnd() and extend() methods.  
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void firstMaximalSegment(const ConstIterator& i,  BidirectionalSegmentComputer);

      /**
       * Same as 
      void firstMaximalSegment(const ConstIterator& i, ForwardSegmentComputer);
       */
      void firstMaximalSegment(const ConstIterator& i, DynamicSegmentComputer);

      /**
       * Same as 
      void firstMaximalSegment(const ConstIterator& i, BidirectionalSegmentComputer);
       */
      void firstMaximalSegment(const ConstIterator& i, DynamicBidirectionalSegmentComputer);


////////////////////////////////////////////////////////// last
      /**
       * Computes the last maximal segment passing throught i
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void lastMaximalSegment(const ConstIterator& i);

      /**
       * Computes the last maximal segment passing throught i
       * using only the extend() method. 
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void lastMaximalSegment(const ConstIterator& i, ForwardSegmentComputer);

      /**
       * Computes the last maximal segment passing throught i
       * using the extendOppositeEnd() and extend() methods.  
       * @param i any ConstIterator
       * Nb: linear complexity in the range size
       */
      void lastMaximalSegment(const ConstIterator& i,  BidirectionalSegmentComputer);

      /**
       * Same as 
      void firstMaximalSegment(const ConstIterator& i, ForwardSegmentComputer);
       */
      void lastMaximalSegment(const ConstIterator& i, DynamicSegmentComputer);

      /**
       * Same as 
      void firstMaximalSegment(const ConstIterator& i, BidirectionalSegmentComputer);
       */
      void lastMaximalSegment(const ConstIterator& i, DynamicBidirectionalSegmentComputer);


    };


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Default constructor.
		 * Nb: not valid
     */
    SaturedSegmentation() {};

    /**
     * Constructor.
     * @param itb, begin iterator of the underlying range
     * @param ite, end iterator of the underlying range
     * @param aSegmentComputer, an online segment recognition algorithm. 
     */
    SaturedSegmentation(const ConstIterator& itb, 
												const ConstIterator& ite, 
												const SegmentComputer& aSegmentComputer);

    /**
     * Init.
     * @param itb, begin iterator the range to processed
     * @param ite, end iterator the range to processed
		 * Nb: must be a valid range included in the underlying range.
     */
    void setSubRange(const ConstIterator& itb, 
							       const ConstIterator& ite);


    /**
     * Set processing mode
     * @param aMode one of the 3 available modes :
     * "Truncate" (default), "Truncate+1", "DoNotTruncate". 
     */
    void setMode(const std::string& aMode);


    /**
     * Destructor.
     */
    ~SaturedSegmentation();

    /**
     * ConstIterator service.
     * @return an iterator pointing on the first segment of a digital curve.
     */
    typename SaturedSegmentation::SegmentComputerIterator begin() const;

    /**
     * ConstIterator service.
     * @return an iterator pointing after the last segment of a digital curve.
     */
    typename SaturedSegmentation::SegmentComputerIterator end() const;


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

    //Begin and end iterators of the underlying range
		ConstIterator myBegin, myEnd;

    //Begin and end iterators of the subrange to be segmented
		ConstIterator myStart, myStop;

    //Mode
    //eiter "Truncate" (default), 
    //"Truncate+1", or "DoNotTruncate". 
    std::string myMode; 

    //SegmentComputer
		SegmentComputer mySegmentComputer;

    // ------------------------- Hidden services ------------------------------


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    SaturedSegmentation ( const SaturedSegmentation & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SaturedSegmentation & operator= ( const SaturedSegmentation & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class SaturedSegmentation


  /**
   * Overloads 'operator<<' for displaying objects of class 'SaturedSegmentation'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SaturedSegmentation' to write.
   * @return the output stream after the writing.
   */
  template <typename SegmentComputer>
  std::ostream&
  operator<< ( std::ostream & out, const SaturedSegmentation<SegmentComputer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/SaturedSegmentation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SaturedSegmentation_h

#undef SaturedSegmentation_RECURSES
#endif // else defined(SaturedSegmentation_RECURSES)
