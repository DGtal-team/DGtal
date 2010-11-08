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
 * @file HyperRectDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand (\c guillaume.damiand@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/25
 *
 * Header file for module HyperRectDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain.h
#else // defined(HyperRectDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_RECURSES

#if !defined HyperRectDomain_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/domains/HyperRectDomain_Iterator.h"
#include "DGtal/io/DGtalBoard.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class HyperRectDomain
  /**
   * Description of class 'HyperRectDomain' <p> \brief Aim:
   * Parallelepidec region of a digital space, model of a 'CDomain'.
   *
   * The following code snippet demonstrates how to use \p HyperRectDomain
   *
   *  \code
   *  #include <DGtal/kernel/Space.h>
   *  #include <DGtal/kernel/domains/HyperRectDomain.h>
   * ...
   *
   * //We create a digital Space based on 'int' integers and in dimension 4
   * typedef DGtal::Space<4> Space4DType;
   * typedef Space4DType::TPoint Point4DType;
   *
   * const int rawA[ ] = { 1, 2, 3 ,4};
   * const int rawB[ ] = { 5, 5, 3 ,4};
   * Point4DType A ( rawA );
   * Point4DType B ( rawB );
   *
   * //Domain construction from two points
   * DGtal::HyperRectDomain<Space4DType> myDomain ( A, B );
   *
   * //We just iterate on the Domain points and print out the point coordinates.
   * std::copy ( myDomain.begin(),
   *             myDomain.end(),
   *             std::ostream_iterator<Point4DType> ( std::cout, " " ) );
   *  \endcode
   *
   *
   * \see testHyperRectDomain.cpp
   * \see testHyperRectDomain-snippet.cpp
   */
  template<typename TSpace>
  class HyperRectDomain
  {
      // ----------------------- Standard services ------------------------------
    public:

      BOOST_CONCEPT_ASSERT(( CSpace<TSpace> ));


      // typedef TSpace DigitalSpace;
      // typedef TSpace SpaceType;
      typedef TSpace Space;

      // static constants
      static const typename Space::Dimension staticDimension = Space::staticDimension;

      typedef HyperRectDomain<Space> Domain;
      typedef typename Space::Point Point;
      typedef typename Space::Integer Integer;
      typedef typename Space::Vector Vector;
      typedef typename Space::Dimension Dimension;
      typedef typename Space::Size Size;
      typedef typename Point::Coordinate Coordinate;


      // BOOST_CONCEPT_ASSERT(( CDomain< HyperRectDomain >));

      ///Typedef of domain iterators
      typedef HyperRectDomain_Iterator<Point,Size> ConstIterator;
      typedef IsWithinPointPredicate<Point> Predicate;

      /**
       * Default Constructor.
       */
      HyperRectDomain();

      /**
       * Constructor from  two points \param aPointA and \param aPoint B
       * defining the space diagonal.
       *
       */
      HyperRectDomain ( const Point &aPointA, const Point &aPointB );


      /**
       * Destructor.
       */
      ~HyperRectDomain();

      /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
      HyperRectDomain ( const HyperRectDomain & other );


      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
      HyperRectDomain & operator= ( const HyperRectDomain & other );



      //------------- Global Iterator
      /**
       * begin() iterator.
       *
       **/
      const ConstIterator begin() const;

      /**
       * begin(aPoint) iterator. Returns an iterator starting at \param aPoint
       *
       **/
      ConstIterator begin ( const Point &aPoint ) const;

      /**
       * end() iterator.
       *
       **/
      const ConstIterator end() const;

      /**
       * end() iterator.
       * @returns a ConstIterator at the endpoint \param aPoint
       *
       **/
      ConstIterator end(const Point &aPoint) const;


      //------------ Subdomain/Permutation  Iterators

#ifdef CPP0X_INITIALIZER_LIST
      /**
       * begin() iterator on a sub-domain with a order than the lexicographic one.
       *
       * @param aSubDomain the sub-domain given as a constant list (e.g. {1,3,2}).
       * @return a ConstIterator
       **/
      ConstIterator subDomainBegin(std::initializer_list<Size> aSubDomain) const;

			/**
      * begin() iterator on a sub-domain with a order than the lexicographic one.
      *
      * @param aSubDomain the sub-domain given as a constant list (e.g. {1,3,2}).
      * @return a ConstIterator
      **/
      ConstIterator subDomainBegin(std::initializer_list<Size> aSubDomain,
          const Point & startingPoint) const;
#endif
      /**
       * begin() iterator on a sub-domain with a order than the lexicographic one.
       *
       * @param aSubDomain the sub-domain given by a vector of dimension.
       * @return a ConstIterator
       **/
      ConstIterator subDomainBegin(const std::vector<Size> & permutation) const;

      /**
      * begin() iterator on a sub-domain with a order than the lexicographic one.
      *
      * @param aSubDomain the sub-domain given by a vector of dimension.
      * @return a ConstIterator
      **/
      ConstIterator subDomainBegin(const std::vector<Size> & permutation,
          const Point & startingPoint) const;



#ifdef CPP0X_INITIALIZER_LIST
      /**
       * end() iterator with an order different from lexicographic.
       *
       **/
      ConstIterator subDomainEnd(std::initializer_list<Size> aSubDomain,
          const Point &startingPoint) const;
      /**
       * end() iterator with an order different from lexicographic.
       *
       **/
      ConstIterator subDomainEnd(std::initializer_list<Size> aSubDomain) const;
#endif
      /**
       * end() iterator with an order different from lexicographic.
       *
       **/
      ConstIterator subDomainEnd(const std::vector<Size> & aSubDomain,
          const Point &startingPoint) const;
      /**
       * end() iterator with an order different from lexicographic.
       *
       **/
      ConstIterator subDomainEnd(const std::vector<Size> & aSubDomain) const;



      //------------- Span Iterator
      /**
       * Returns a Span iterator starting at \param aPoint and moving toward the dimension \param aDimension.
       *
       **/
      ConstIterator spanBegin ( const Point &aPoint, const std::size_t aDimension) const;

      /**
       * Creates a end() Span iterator along the dimension \param aDimension.
       *
       **/
      ConstIterator spanEnd (const std::size_t aDimension) const;

      // ----------------------- Interface --------------------------------------
    public:

      /**
       * @return  the extent of the HyperRectDomain
       *
       **/
      std::size_t extent() const;


      /**
       * Returns the extent of the HyperRectDomain (static method).
       *
       * @param aLowerBound  a Point corresponding to the lower bound.
       * @param aUpperBound a Point corresponding to the upper bound.
       *
       * @return the extent.
       */
      static std::size_t extent(const Point &aLowerBound, const Point &aUpperBound)
      {
        std::size_t val = 1;
        for (Size k =  0; k < TSpace::staticDimension ; k++)
          val *= (aUpperBound.at(k) - aLowerBound.at(k) + 1);

        return val;
      }


      /**
       * Returns the lowest point of the space diagonal.
       *
       **/
      const Point &lowerBound() const;

      /**
       * Returns the highest point of the space diagonal.
       *
       **/
      const Point &upperBound() const ;

      /**
       * @param p any point.
       * @return 'true' if point [p] is inside this domain.
       */
      bool isInside( const Point & p ) const;

      /**
       * @return a const reference to the "IsInside" predicate.
       *
       * NB: Could have used template class DomainPredicate but, for
       * performance reason, directly used the IsWithinPointPredicate
       * which fits perfectly.
       */
      const Predicate & predicate() const;


      // ------------------------- Private Datas --------------------------------
    private:

      /**
       * Default style.
       */
      struct DefaultDrawStylePaving : public DrawableWithBoard
      {
        virtual void selfDraw(LibBoard::Board & aBoard) const
        {
          aBoard.setPenColorRGBi(160, 160, 160);
          aBoard.setFillColorRGBi(255, 255, 255);
          aBoard.setLineStyle(LibBoard::Shape::SolidStyle);
        }
      };

      /**
       * Default style.
       */
      struct DefaultDrawStyleGrid : public DrawableWithBoard
      {
        virtual void selfDraw(LibBoard::Board & aBoard) const
        {
          aBoard.setPenColorRGBi(160, 160, 160);
          aBoard.setLineStyle(LibBoard::Shape::DashStyle);
        }
      };

    // --------------- CDrawableWithBoard realization ------------------------
  public:
    
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithBoard* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;

    /**
     * Draw the object on a LibBoard board.
     * @param board the output board where the object is drawn.
     */
    void selfDraw( DGtalBoard & board ) const;


    /**
     * Draw the object (as a Grid) on a LiBoard board.
     * @param board the output board where the object is drawn.
     * @param asGrid to choose between paving vs. grid representation.
     */
    void selfDrawAsGrid( LibBoard::Board & board) const;
    
    /**
     * Draw the object (as a Grid) on a LiBoard board.
     * @param board the output board where the object is drawn.
     * @param asGrid to choose between paving vs. grid representation.
     */
    void selfDrawAsPaving( LibBoard::Board & board ) const;
    
    
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
    
    
    
    // ------------------------- Hidden services ------------------------------
    private:


      ///The lowest point of the space diagonal
      Point myLowerBound;
      ///The highest point of the space diagonal
      Point myUpperBound;

      /// "IsInside" predicate.
      Predicate myPredicate;

      /// Begin iterator
      ConstIterator myIteratorBegin;

      /// End iterator
      ConstIterator myIteratorEnd;
  }; // end of class HyperRectDomain


  /**
   * Overloads 'operator<<' for displaying objects of class 'HyperRectDomain'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'HyperRectDomain' to write.
   * @return the output stream after the writing.
   */
  template<typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out, const HyperRectDomain<TSpace> & object );


  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawDomainGrid : public DrawWithBoardModifier {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myModes[ "HyperRectDomain" ] = "Grid";
    }
  };

  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawDomainPaving : public DrawWithBoardModifier {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myModes[ "HyperRectDomain" ] = "Paving";
    }
  };


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/domains/HyperRectDomain.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_h

#undef HyperRectDomain_RECURSES
#endif // else defined(HyperRectDomain_RECURSES)
