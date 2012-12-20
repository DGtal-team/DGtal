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
  namespace deprecated
  {
  /////////////////////////////////////////////////////////////////////////////
  // template class MaximalSegments
  /**
   * @warning This class is deprecated since SaturatedSegmentation has been created. 
   * This class is however used now in @href testMaximalSegments.cpp , 
   * @href MostCenteredMaximalSegmentEstimator.h and @href displayContours.cpp
   * 
   * Description of template class 'MaximalSegments' <p>
   * \brief Aim: Computes the set of maximal segments of a sequence.
   * Maximal segments are segments that cannot be included in other segments. 
   * This class is a model of CDecomposition.
   * 
   * This class is templated by 'TSegment', a model of CBidirectionnalSegmentComputer,
   * i.e. an object that is able to manage the on-line recognition of a given class of 
   * segments (4-connected DSS, 8-connected DSS, thick segment, etc.)
   * 'TSegment' must have an internal type 'Iterator' that is a means of 
   * of accessing the sequence elements. 
   * 'TSegment' must have the methods init(), extendForward() and extendBackward(), 
   * taking an input parameter of type 'Iterator'. The last two methods must return 
   * a boolean equal to TRUE if the extension is possible and has been successfully
   * performed and FALSE otherwise.
   *    
   * In the short example below, the parameters of the maximal 8-connected DSSs
   * of a sequence stored in a STL vector are sent to the standard output.
   * The set of maximal DSSs of a sequence is also called tangential cover. 
   * @code 
  //types definition
  typedef PointVector<2,int> Point;
  typedef std::vector<Point> Sequence;
  typedef Sequence::iterator Iterator;
  typedef ArithmeticalDSS<Iterator,int,8> DSSComputer;
  typedef MaximalSegments<DSS> Cover;

  //sequence of input points
  Sequence curve;
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
  DSSComputer algo;
  Cover theCover(curve.begin(), curve.end(), algo, false);
         
  Cover::SegmentIterator i = theCover.begin();
  for ( ; i != theCover.end(); ++i) {
    DSS currentSegment(*i);
    trace.info() << currentSegment << std::endl;  //standard output
  } 

   * @endcode
   */
  template <typename TSegment>
  class MaximalSegments
  {

  public: 

    typedef TSegment Segment;
    typedef  typename TSegment::Reverse ReverseSegment;
    typedef typename TSegment::ConstIterator Iterator;    
    typedef typename ReverseSegment::ConstIterator ReverseIterator;


    // ----------------------- Standard services ------------------------------
  public:



    /**
     * This class is an iterator on a sequence
     * storing the current segment.
     */
    class SegmentIterator
    {

         // ------------------------- private data -----------------------
    private:
      

      /**
       * Pointer to the cover of maximal segments
       */
      MaximalSegments<Segment> *myCov;

      /**
       * An iterator of the sequence
       * at the front of the current segment
       */
      Iterator myFront;

      /**
       * An iterator of the sequence 
       * at the back of the current segment
       */
      Iterator myBack;

      /**
       * The current segment
       */
      Segment  mySegment;

    
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
       * A flag equal to FALSE if the current segment
       * lies between the begin and the end iterator 
       * of the sequence, TRUE otherwise. 
       * Nb: always FALSE if the sequence is 
       * processed as open. 
       */
      bool  myFlag;

      // ------------------------- Standard services -----------------------
    public:
       friend class MaximalSegments<Segment>;
         
      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aCov the set of maximal segments
       * @param aBack an iterator at the back of the first segment
       */
      SegmentIterator( MaximalSegments<Segment> *aCov,
		       const Iterator& aBack,
		       const Segment& aSegment);


      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      SegmentIterator( const SegmentIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      SegmentIterator& operator=( const SegmentIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~SegmentIterator();
    
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
       * Goes to the next maximal segment on the sequence (if possible).
       * Nb: complexity in O(n).
       */
      SegmentIterator& operator++();
      
      /**
       * @return TRUE if the current segment intersects
       * the next one, FALSE otherwise.
       */
      bool intersectNext() const;

      /**
       * @return TRUE if the current segment intersects
       * the previous one, FALSE otherwise.
       */
      bool intersectPrevious() const;

      /**
       * @return an iterator of a sequence
       * at the front of the segment.
       */
      const Iterator getFront() const;

      /**
       * @return an iterator of a sequence
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
      bool operator==( const SegmentIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with 
       *
       * @return 'true' if their current positions differs.
       * (different front and back iterators)
       */
      bool operator!=( const SegmentIterator & aOther ) const;

      // ------------------------- hidden services -------------------------
 
      private: 

      /**
       * Extension of the segment along the sequence while it is possible.
       * @param aSeg a segment computer
       * @param it an iterator on a sequence
       * @param end an iterator after the end of the sequence
       * Nb: complexity in O(n).
       */
      template <typename TypeSegment, typename TypeIterator>
      void extension(TypeSegment& aSeg, TypeIterator& it, const TypeIterator& end);

      /**
       * Extension of the segment along the sequence while it is possible.
       * @param aSeg a segment computer
       * @param it an iterator on a sequence
       * Nb: complexity in O(n).
       */
      template <typename TypeSegment, typename TypeIterator>
      void extension(TypeSegment& aSeg, TypeIterator& it);

       /**
       * Extension of the segment along the (circular) sequence while it is possible.
       * @param aSeg a segment computer
       * @param it an iterator on a sequence
       * @param begin an iterator at the beginning of the sequence
       * @param end an iterator after the end of the sequence
       * Nb: complexity in O(n).
       */
      template <typename TypeSegment, typename TypeIterator>
      void extensionInLoop(TypeSegment& aSeg, TypeIterator& it, 
                           const TypeIterator& begin, const TypeIterator& end);



      /**
       * Retrieves the first maximal segment found on the sequence.
       * Nb: complexity in O(n).
       */
      void firstMaximalSegment();

      /**
       * Goes to the next maximal segment on the sequence (if possible).
       * Nb: complexity in O(n).
       */
      void nextMaximalSegment();

      /**
       * Checks if the current segment intersects the next one (if exists).
       * @param it a given iterator
       */
      bool doesIntersectNext(const Iterator& it);
      
    };


    // ----------------------- Interface --------------------------------------
  public:


    /**
     * Default constructor.
     */
    MaximalSegments(){};

    /**
     * Constructor.
     * Nb: The sequence is processed as a closed one by default.
     * @param begin, begin iterator on a sequence
     * @param end, end iterator on a sequence
     * @param aSegment, a segment computer
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    MaximalSegments(const Iterator& beginIt, 
                    const Iterator& endIt, 
                    const Segment& aSegment, 
                    const bool& aFlag);

    /**
     * Init.
     * Nb: The sequence is processed as a closed one by default.
     * @param begin, begin iterator on a sequence
     * @param end, end iterator on a sequence
     * @param aSegment, a segment computer
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    void init(const Iterator& beginIt, 
              const Iterator& endIt, 
              const Segment& aSegment, 
              const bool& aFlag);


    /**
     * Destructor.
     */
    ~MaximalSegments();

    /**
     * Iterator service.
     * @return an iterator pointing on the first segment of a sequence.
     */
    typename MaximalSegments::SegmentIterator begin();

    /**
     * Iterator service.
     * @return an iterator pointing after the last segment of a sequence.
     */
    typename MaximalSegments::SegmentIterator end();


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
     * begin iterator (pointing at the first element)
     */
    Iterator myBegin;

    /**
     * end iterator (pointing after the last element)
     */
    Iterator myEnd;

    /**
     * back iterator (first element) 
     * of the first maximal segment
     */
    Iterator myFirstMaximalSegmentBack;

    /**
     * a segment Computer
     */
    Segment mySegment;

    /**
     * boolean equal to TRUE if the sequence
     * has to be processed as closed, FALSE otherwise
     */
    bool isClosed;


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

  } // namespace deprecated
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/MaximalSegments.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MaximalSegments_h

#undef MaximalSegments_RECURSES
#endif // else defined(MaximalSegments_RECURSES)
