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
 * @file AlphaThickSegmentComputer.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 * @author Alexandre Faure
 * @author Fabien Feschet 
 * @author Mohammad Said
 * @author Jacques-Olivier Lachaud
 *
 * @date 2015/01/05
 *
 * Header file for module AlphaThickSegmentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AlphaThickSegmentComputer_RECURSES)
#error Recursive header files inclusion detected in AlphaThickSegmentComputer.h
#else // defined(AlphaThickSegmentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AlphaThickSegmentComputer_RECURSES

#if !defined AlphaThickSegmentComputer_h
/** Prevents repeated inclusion of headers. */
#define AlphaThickSegmentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/geometry/surfaces/ParallelStrip.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class AlphaThickSegmentComputer
/**
 * Description of class 'AlphaThickSegmentComputer' <p>
 *
 * \brief Aim: This class is devoted to the recognition of the alpha
 * thick segment as described in \cite FaureTangential2008 . From a
 * maximal diagonal alphaMax width, it allows to apply the recognition
 * of a thick segment with the ability to take into accounts some
 * noise. Moreover the segment can be detected from points not
 * necessary connected and/or with floating coordinates.
 *
 * As other solutions like \cite DebledRennessonBlurred2005 the
 * algorithm given here is based on the height/width computation from
 * the convex hull of a given set of points. The actual implementation
 * exploits the height/width maintenance defined from the \cite
 * FaureTangential2008 (see \cite FaureTangential2008 page 363) which
 * reduces the complexity from \f$O(n\ log\ n) \f$ into  \f$O( log\ n) \f$.
 * Note that the convexhull maintenance in linear time (with point
 * substraction) proposed by Buzer \cite lilianComputing2007 is not
 * yet implemented.
 *  
 *
 * The proposed implementation is mainly a backport from
 * [ImaGene](https://gforge.liris.cnrs.fr/projects/imagene) with some
 * various refactoring.
 */


template <typename TSpace, 
          typename TInputPoint,
          typename TInternalScalar, typename TConstIterator = typename std::vector< TInputPoint >::const_iterator >

class AlphaThickSegmentComputer
{

  // ----------------------- public types --------------------------------------
  BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));
  BOOST_CONCEPT_ASSERT(( concepts::CSignedNumber< TInternalScalar > ));
  BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
  BOOST_STATIC_ASSERT(( TInputPoint::dimension == 2 ));
  
public:
  typedef TSpace Space;
  typedef TInputPoint InputPoint;
  typedef TInternalScalar InternalScalar;
  typedef InputPoint InputVector;
  typedef InternalScalar InternalVector[ 2 ];

  
  typedef std::vector< InputPoint > InputPointContainer;
  typedef typename InputPointContainer::size_type Size;
  typedef typename InputPointContainer::const_iterator ContainerConstIterator;
  typedef typename InputPointContainer::iterator Iterator;
  typedef TConstIterator ConstIterator;
  typedef ParallelStrip<Space,true,true> Primitive;  
  
  /**
   * Type of embedded points 
   * @see getBasicBoundingBox, getRealBoudingBox, getBoundingBoxFromExtremPoints
   */
  typedef DGtal::PointVector<2, double> PointD; 

  typedef AlphaThickSegmentComputer<Space, InputPoint,InternalScalar, ConstIterator> Self; 
  typedef AlphaThickSegmentComputer<Space, InputPoint,InternalScalar, ReverseIterator<ConstIterator> > Reverse; 


  // ----------------------- internal types --------------------------------------

private: 
  struct State{
    std::deque<InputPoint> melkmanQueue; /** Melkman algorithm main dequeu */
    InputPoint lastFront; /** the last point added at the front of the alpha thick segment */
    InputPoint lastBack; /** the last point added at the back of the alpha thick segment */
    InputPoint edgePh; /** one the convexhull edge point of the (edge, vertex) pair used to compute the convexhull height */
    InputPoint edgeQh; /** one the convexhull edge point of the (edge, vertex) pair used to compute the convexhull height */
    InputPoint vertexSh; /** one the convexhull vertex of the (edge, vertex) pair used to compute the convexhull height */
    bool isMelkmanInitialized; /** well initialized when at least 3 points are given.  */ 
    double convexHullHeight;  /** Used in melkmanMainDiagonal() */
    double convexHullWidth;  /** Used in melkmanMainDiagonal() */ 
  };
    
  
  // ----------------------- Standard services ------------------------------
public:
  

  /**
   * Constructor.
   */
  AlphaThickSegmentComputer();


  /**
   * Destructor.
   */
  ~AlphaThickSegmentComputer();
  

  /**
   * Copy constructor.
   *
   * @param[in] other the object to clone.
   */
  AlphaThickSegmentComputer ( const AlphaThickSegmentComputer & other );
  

  /**
   * Assignment.
   *
   * @param other the object to copy.
   * @return a reference on 'this'.
   */
  AlphaThickSegmentComputer & operator= ( const AlphaThickSegmentComputer & other );
  

  /** 
   * @return a default-constructed instance of Self 
   */
  Self getSelf() const;

  
  /**
   * @return a default-constructed instance of Reverse
   */
  Reverse getReverse() const;
  
  /**
   * Equality operator.
   * @param other the object to compare with.
   * @return 'true' if the AlphaThickSegment representations
   * and the ranges of the two objects match,
   * 'false' otherwise
   */
  bool operator==( const AlphaThickSegmentComputer & other ) const;
  
  /**
   * Difference operator.
   * @param other the object to compare with.
   * @return 'false' if equal
   * 'true' otherwise
   */
  bool operator!=( const AlphaThickSegmentComputer & other ) const;

  /**
   * Initialisation.
   * @param[in] aThickness the maximal thickness of the alpha thick segment (alpha_max).
   */  
  void init(double aThickness);  
  
  


  //-------------------- model of ForwardContainer -----------------------------
public:

  /**
   * @return the number of distinct points stored in the container.
   * @note it returns 0 if segment computer is initialized with a curve
   * iterator without the option of saving samples).
   */
  Size size() const;
  
  
  /**
   * @return 'true' if and only if the container contains no point. 
   * @note: returns always 'true' if the segment computer is initialized with a curve
   * const iterator without the option of saving samples).
   */
  bool empty() const;
  
  
  /**
   * @return a const iterator pointing on the first point given in the
   * container associated to the current alpha thick segment (is empty
   * if the saving option is not selected in the init method).
   */
  ContainerConstIterator containerBegin() const;


  /**
   * @return a const iterator pointing after the last point given in the
   * container associated to the current alpha thick segment (is
   * empty if the saving option is not selected in the init method).
   */
  ContainerConstIterator containerEnd() const;


  /**
   * @return a const iterator pointing on the first point of the
   * current alpha thick segment. Usefull oonly if the initialisation was
   * done with a contour iterator (else, it is empty).
   */
  ConstIterator begin() const;


  /**
   * @returns a const iterator pointing after the last point stored in
   * the current alpha thick segment container. Usefull oonly if the
   * initialisation was done with a contour iterator (else, it is
   * empty).
   */
  ConstIterator end() const;


  /**
   * NB: std version.
   * @return the maximal allowed number of points in the current alpha thick segment.
   * @see maxSize
   */
  Size max_size() const;

  
  /**
   * same as max_size
   * @return the maximal allowed number of points in the current alpha thick segment.
   */
  Size maxSize() const;  
  



  //-------------------- model of CForwardSegmentComputer ----------------

public:
     
  /**
   * Initialisation with an ConstIterator (to conform to CForwardSegmentComputer).
   * By default the thickness will be set to 1.
   *
   * @param[in] it an iterator on input points.
   * @param[in] aThickness the thickness of the alpha thick segment (default 1.0).
   */  
  void init(const ConstIterator &it, double aThickness=5.0);    
  
  
  /**
   * Tries to extend front the current alpha thick segment with the
   * next contour point and checks if we have still an alpha thick
   * segment of thickness less or equal to the initial value
   * alpha_max. If it is the case the new point is added and the
   * segment parameters are updated, otherwise the alpha thick segment
   * is keep in its original state.
   *
   * @return 'true' if the segment has been extended and 'false'
   * otherwise (the object is then in its original state).
   */
  bool extendFront();


  /**
   *  Tests whether the current alpha thick segment can be extended at
   *  the front with the next contour point i.e checks if we have
   *  still an alpha thick segment of width alpha_max after adding the
   *  given point \aPoint. The segment parameters are keep in its
   *  original state.
   *  
   * @return 'true' if the segment can be extended with the given point, 'false' otherwise.
   */
  bool isExtendableFront();

  
  /**
   *  Tests whether the current alpha thick segment can be extended, i.e
   *  checks if we have still an alpha thick segment of width alpha_max after
   *  adding the given point \aPoint. The segment parameters are keep
   *  in its original state.
   *  
   * @param[in] aPoint the point to be tested for the segment extension.
   * @return 'true' if the segment can be extended with the given point, 'false' otherwise.
   */
  bool isExtendableFront(const InputPoint &aPoint);


  /**
   * Tries to add the point \a aPoint at the front of the current alpha thick
   * segment and checks if we have still an alpha thick segment of thickness
   * less or equal to the initial value alpha_max. If it is the case
   * the new point is added and the segment parameters are updated,
   * otherwise the alpha thick segment is keep in its original state.
   *
   * @param[in] aPoint the new point to extend the current segment.
   * @return 'true' if the segment has been extended and 'false'
   * otherwise (the object is then in its original state).
   */
  bool extendFront(const InputPoint &aPoint);
  
 

  //-------------------- Primitive services -----------------------------
  
  
  /**
     Returns the current primitive recognized by this computer,
     which is a ParallelStrip of axis width smaller than the one
     specified at instanciation.
  */
  Primitive primitive() const;
  


  
    // ----------------------- Interface --------------------------------------
public:

  
  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  bool isValid() const;
  


 
   /**
   * @return the segment length defined from the basic bouding box (@see getBasicBoundingBox).
   **/
  double getBasicLength();


  /**
   * @return the segment length defined from the real bouding box (@see getRealBoudingBox).
   *
   **/
  double getRealLength();
  

  /**
   * @return 'true' if the points of the segment computer are stored in the main container.
   **/
  bool isStoringSegmentPoints() const;


  /**
   * @return the total number of points beeing recognized through in the segment construction.
   **/  
  unsigned int getNumberSegmentPoints() const;



  // ------------------------- Display services ------------------------------


  /**
   * @return the current alpha thick segment convexhull given as a vector containing the vertex points.
   **/
  std::vector<InputPoint> getConvexHull() const;


  
  /**
   * Computes the segment bounding box according to two extremity points (\a aFirstPt, \a aLastPt).
   * @param[in] aFirstPt the first extrem point.
   * @param[in] aLastPt the last extrem point. 
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   * @param[in] minVisibleWidthBounds the minimal width of the resulting bounding box (for drawing issue).
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/  
  void getBoundingBoxFromExtremPoints(const InputPoint &aFirstPt,
                                      const InputPoint &aLastPt,
                                      PointD &pt1LongestSegment1,
                                      PointD &pt2LongestSegment1,
                                      PointD &pt3LongestSegment2,
                                      PointD &pt4LongestSegment2,
                                      double minVisibleWidthBounds = 0.2) const;
  
  
  /**
   * Computes the basic segment bounding box according to the segment
   * extremity points (the last points added to the front and to the
   * back).
   *
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/
  void getBasicBoundingBox(PointD &pt1LongestSegment1,
                           PointD &pt2LongestSegment1,
                           PointD &pt3LongestSegment2,
                           PointD &pt4LongestSegment2) const;
  
  
  /**
   * Computes the real bounding box according to the real extremity
   * points of the segment. The real extremity points are computed
   * after a scan of all the segment points. The real bounding box can
   * differs of the basic bounding box when a large amount of noise
   * are give in the initial curve.
   *
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/
  void  getRealBoundingBox(PointD &pt1LongestSegment1,
                          PointD &pt2LongestSegment1,
                          PointD &pt3LongestSegment2,
                          PointD &pt4LongestSegment2) const;
  
  
  

   /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;



  
  /**
   * Writes/Displays the object on an output stream.
   *
   * @param[out] out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const;
  
    // ------------------------- Protected Datas ------------------------------
protected:
  
  
  
  /**
   * begin iterator (associated to input data)
   **/
  ConstIterator myBegin;
  
  /**
   * begin iterator (associated to input data)
   **/
  ConstIterator myEnd;
  
    


    // ------------------------- Private Datas --------------------------------
private:

  /**
   * The set of points contained in the alpha thick segment which can be changed during computations.
   **/
  mutable InputPointContainer myPointContainer; 

  /**
   * The maximal thickness of the segment.
   */
  double myThickness;  

  /**
   * State of the actual computer
   **/ 
  State myState;

  /**
   * Previous saved computer state
   **/   
  mutable State _state;

  
  bool myIsStoringPoints;
  
  /**
   * Used by the size method.
   **/
  unsigned int myNbPointsAddedFromIterators;


    // ------------------------- Hidden services ------------------------------
protected:

  
  /**
   *  Used to check if the initialisation with 3 non aligned points is well done.
   *
   * @return 'true' if the segment contains at least 3 non aligned points. 
   **/     
   bool melkmanIsWellInitialized() const;


  /**
   *  Updates the main height of the melkman convex set.
   **/     
  void melkmanUpdateMainHeight();
  
  
  /**
   * Adds a point in the convex set  using one step of Melkman algorithm.
   *
   * @param[in] aPoint the point to be added.
   */
  void melkmanAddPoint(const InputPoint  &aPoint );



  /**
   * Init melkman algorithm with the main queue and the 3 input first points.
   **/
  void melkmanInit( ) ; 


  /**
   * Depending on connexity, return true if a convex is valid.
   */
  bool melkmanIsConvexValid() ;
  
  
  /**
   * Tests if the point \a aPoint2 is Left|On|Right of an infinite line (defined from two points \a  aPoint0 and \a aPoint1).
   *
   * @param[in] aPoint0 the first point defining the line.
   * @param[in] aPoint1 the second point defining the line.
   * @param[in] aPoint2 the point to be tested.
   */
  InternalScalar melkmanIsLeft(const InputPoint &aPoint0, const InputPoint &aPoint1, const InputPoint &aPoint2) const;
  
  
  /**
   * Computes the projection of a Point \a ptC on the real line defined by the two points (\a ptA, \a ptB), and
   * return true if the projected point is inside the segment closed interval [A,B].
   * 
   * @param[in] ptA one of the two points defining the straight line.
   * @param[in] ptB one of the two points defining the straight line.
   * @param[in] ptC the point to be projected.
   * @param[out] ptProjected the projected point.
   * @return true if ptProjected is inside the segment [A,B].
   **/
  template<typename TPointD>
  bool projetOnStraightLine(const TPointD & ptA, const TPointD & ptB,
                            const TPointD & ptC, PointD & ptProjected) const;
  

  /**
   * From an point iterator (\a itBegin and \a itEnd) it computes the
   * two extrem points (\a aFirstExtrPt and \a aLastExtrPt) defined
   * according to the direction of the current segment.
   *
   * @param[in] itBegin the start iterator of the input points.
   * @param[in] itEnd the end iterator of the input points.
   * @param[out] aFirstExtrPt the first extrem point.
   * @param[out] aLastExtrPt the last extrem point.
   * 
   **/
  template<typename TConstIteratorG>
  void computeExtremaPoints(const TConstIteratorG & itBegin, const TConstIteratorG & itEnd,
                            InputPoint & aFirstExtrPt, InputPoint & aLastExtrPt) const;
  
  
    // ------------------------- Internals ------------------------------------
private:

}; // end of class AlphaThickSegmentComputer


/**
 * Overloads 'operator<<' for displaying objects of class 'AlphaThickSegmentComputer'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'AlphaThickSegmentComputer' to write.
 * @return the output stream after the writing.
 */
template <typename TSpace, typename TInputPoint, typename TInternalScalar, typename TConstIterator>
std::ostream&
operator<< ( std::ostream & out, const AlphaThickSegmentComputer<TSpace, TInputPoint, TInternalScalar, TConstIterator> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/AlphaThickSegmentComputer.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AlphaThickSegmentComputer_h

#undef AlphaThickSegmentComputer_RECURSES
#endif // else defined(AlphaThickSegmentComputer_RECURSES)
