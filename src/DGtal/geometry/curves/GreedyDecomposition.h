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
  namespace deprecated 
  {
    
  /////////////////////////////////////////////////////////////////////////////
  // template class GreedyDecomposition
  /**
   * @warning This class is deprecated since GreedySegmentation has been created. 
   * This class is used in @href testDecomposition.cpp @href DSSLengthEstimator.ih and @href displayContours.cpp, 
   * but GreedySegmentation will be preferred in the future. 
   *
   * Description of template class 'GreedyDecomposition' <p>
   * \brief Aim: Computes the greedy decomposition of a sequence 
   * into segments (the last element of a given segment is the first one
   * one of the next segment).
    
   * This class is a model of CDecomposition.
   * 
   * This class is templated by 'TSegment', a model of CSegmentComputer
   * that is able to manage the on-line recognition of a given class of 
   * segments (4-connected DSS, 8-connected DSS, thick segment, etc.)
   * 'TSegment' must have an internal type 'ConstIterator' that is a means of 
   * of accessing the sequence elements. 
   * 'TSegment' must have the methods init() and extendForward() taking as input
   * a parameter of type 'Iterator'. The extend method must return a boolean
   * equal to TRUE if the extension is possible and has been successfully
   * performed and FALSE otherwise.
   *
   * In the short example below, a digital curve stored in a STL vector
   * is decomposed into 8-connected DSSs whose parameters are sent to 
   * the standard output.
   * @code 
   
  //types definition
  typedef PointVector<2,int> Point;
  typedef std::vector<Point> Sequence;
  typedef Sequence::const_iterator ConstIterator;
  typedef ArithmeticalDSS<ConstIterator,int,8> DSS;
  typedef GreedyDecomposition<DSS> Decomposition;

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
  DSS dssRecognition;
  Decomposition theDecomposition(curve.begin(), curve.end(), dssRecognition, false);
         
  Decomposition::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
    DSS currentSegment(*i);
    trace.info() << currentSegment << std::endl;  //standard output
  } 

   * @endcode
   *
   * If you want to get the DSSs decomposition of the digital curve
   * when it is scanned in the reverse way, you can use the reverse
   * iterator of the STL vector:   
   * @code 
...
  typedef Sequence::const_reverse_iterator ConstReverseIterator;
...
  Decomposition theDecomposition(curve.rbegin(), curve.rend(), dssRecognition, false);
...
   * @endcode
   */

  template <typename TSegment>
  class GreedyDecomposition
  {

  public: 

    typedef TSegment Segment;
    typedef typename Segment::ConstIterator Iterator;

    // ----------------------- Standard services ------------------------------
  public:



    /**
     * This class is an iterator on a digital curve 
     * storing the current segment.
     */
    class SegmentIterator
    {
           
         // ------------------------- data -----------------------
    private:

      /**
       * Pointer to the decomposition
       */
      const GreedyDecomposition<TSegment> *myDec;


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



      // ------------------------- Standard services -----------------------
    public:
       friend class GreedyDecomposition<TSegment>;
         


      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aDec a greedy decomposition of a digital curve
       * @param aBack an iterator at the back of the first segment
       */
      SegmentIterator( const GreedyDecomposition<TSegment> *aDec,
         const typename TSegment::ConstIterator& aBack,
         const TSegment& aSegment);


      /**
       * Copy constructor.
       * @param aOther the iterator to clone.
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
       * Goes to the next segment on the contour (if possible).
       * Nb: complexity in O(n).
       */
      SegmentIterator& operator++();
      
      /**
       * Goes to the next segment on the contour (if possible).
       * Nb: complexity in O(n).
       */
      void next();

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

    // ----------------------- hidden services --------------------------------------

      private: 

      /**
       * Computes the longest possible segment from 
       * two consecutive points.
       * Nb: complexity in O(n).
       */
      void longestSegment();
      
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
     * Nb: not valid
     */
    GreedyDecomposition() {};

    /**
     * Constructor.
     * Nb: The digital curve is decompose as a closed one by default.
     * @param aBegin begin iterator on a digital curve
     * @param aEnd end iterator on a digital curve
     * @param aSegment a segment computer
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    GreedyDecomposition(const Iterator& aBegin, 
                        const Iterator& aEnd, 
                        const Segment& aSegment, 
                        const bool& aFlag);

    /**
     * Init.
     * Nb: The digital curve is decompose as a closed one by default.
     * @param aBegin begin iterator on a digital curve
     * @param aEnd end iterator on a digital curve
     * @param aSegment a segment computer
     * @param aFlag a boolean equal to TRUE to decompose the digital
     * curve as a closed one, FALSE otherwise
     */
    void init(const Iterator& aBegin, 
              const Iterator& aEnd, 
              const Segment& aSegment, 
              const bool& aFlag);


    /**
     * Destructor.
     */
    ~GreedyDecomposition();

    /**
     * Iterator service.
     * @return an iterator pointing on the first segment of a digital curve.
     */
    typename GreedyDecomposition::SegmentIterator begin() const;

    /**
     * Iterator service.
     * @return an iterator pointing after the last segment of a digital curve.
     */
    typename GreedyDecomposition::SegmentIterator end() const;


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

    Segment mySegment;

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
  template <typename Segment>
  std::ostream&
  operator<< ( std::ostream & out, const GreedyDecomposition<Segment> & object );

  } // namespace deprecated
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/GreedyDecomposition.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GreedyDecomposition_h

#undef GreedyDecomposition_RECURSES
#endif // else defined(GreedyDecomposition_RECURSES)
