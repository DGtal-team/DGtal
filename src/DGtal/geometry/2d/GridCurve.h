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
 * @file GridCurve.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/27
 *
 * Header file for module GridCurve.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GridCurve_RECURSES)
#error Recursive header files inclusion detected in GridCurve.h
#else // defined(GridCurve_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GridCurve_RECURSES

#if !defined GridCurve_h
/** Prevents repeated inclusion of headers. */
#define GridCurve_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>
#include <cstddef>


#include "DGtal/base/BasicTypes.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/topology/KhalimskySpaceND.h"

#include "DGtal/io/DGtalBoard.h"



//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class GridCurve
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'GridCurve' <p> Aim: describes a 4-connected oriented 
 interpixel curve, closed or open. For instance, the topological boundary of a 
simply connected digital set is a closed grid curve. This object provides 
several ranges, such as PointsRange used to get the (integer) coordinates 
of the grid points (or pointels) of the grid curve. 

   * Example :
   * @code 

   * @endcode
   */

  template <typename KSpace>
  class GridCurve
  {

  public: 
  typedef typename KSpace::Space::Point Point;
  typedef typename std::vector<Point> Storage;

    // ------------------------- static services ------------------------------
  public:

    /**
     * Outputs the chain [c] to the stream [out].
     * @param out any output stream,
     * @param c a grid curve.
     */
    static void write( std::ostream & out, const GridCurve & c )
    {
      typename Storage::const_iterator i;
      for (i =  c.myData.begin(); i != c.myData.end(); ++i) {
        Point p = *i;
        out << p[0] << " " << p[1] << endl;
      }
    }


    /**
     * Reads a chain from the stream [in] and updates [c].
     * @param in any input stream,
     * @param c (returns) the grid curve.
     */
    static void read( std::istream & in, GridCurve & c )
    {

      c.myData = PointListReader<Point>
                ::getPointsFromInputStream(in);
    };



    /**
     * Return a vector containing all the interger points of the GridCurve.
     *
     * @param c the grid curve
     * @param aVContour (returns) the vector containing all the (grid) points.
     */
    static void getData(const GridCurve& c, 
           std::vector<Point>& aVectorOfPoints)
    {
      aVectorOfPoints.clear();
      typename Storage::const_iterator i;
      for (i = c.myData.begin(); i != c.myData.end(); ++i) {
        aVectorOfPoints.push_back(*i);
      }
    }




    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GridCurve(){};


    /**
     * Constructor.
     * @param aVectorOfPoints the vector containing the sequence of grid points. 
     */
    GridCurve( const std::vector<Point> aVectorOfPoints) {
      myData = aVectorOfPoints;
    };

    /**
     * Constructor.
     * @param in any input stream,
     */
    GridCurve(std::istream & in ) {
      DGtal::GridCurve<KSpace>::read(in, *this);
    };


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GridCurve( const GridCurve & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GridCurve & operator=( const GridCurve & other );




    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[GridCurve]" << std::endl;
      typename Storage::const_iterator i;
      for (i =  myData.begin(); i != myData.end(); ++i) {
        out << *i << std::endl;
      }
      
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    };

  public:


    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithDGtalBoard* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;
    
    /**
       Draw the object on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDraw(DGtalBoard & board ) const;
    
    /**
       Draw the points on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDrawPoints(DGtalBoard & board ) const; 
    /**
       Draw the grid edges on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDrawEdges(DGtalBoard & board ) const;

     

    // ------------------------- Public Datas --------------------------------
  public:
    Storage myData; 


    // ------------------------- Internal --------------------------------
  private:

    /**
     * Default Style Functor for selfDraw methods
     *
     * @param aBoard
     */

    struct SelfDrawStyle
    {
      SelfDrawStyle(DGtalBoard & aBoard)
      {
      }
    };

    struct DefaultDrawStyle : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
      }
    };

    struct DefaultDrawStylePoints : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
      }
    };

  struct DefaultDrawStyleEdges : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
	      aBoard.setLineStyle (LibBoard::Shape::SolidStyle );
	      aBoard.setFillColor(DGtalBoard::Color::None);
      }
    };

  // ------------------------- inner classes --------------------------------

  public: 

    ///////////////////////////////////////////////////////////////////////////////
    // class PointsRange
    ///////////////////////////////////////////////////////////////////////////////


    /**
     * This class is a model of CRange and thus provides a ConstIterator to scan 
     * the grid points (pointels) 
     */

   
    class PointsRange
    {

      // ------------------------- inner type --------------------------------
        public: 
          typedef typename GridCurve::Storage Storage; 
          typedef typename GridCurve::Storage::const_iterator ConstIterator;

       /**
         * Default Constructor.
         */
	
        PointsRange()
        {
        }

       /**
         * Constructor.
         */
	
        PointsRange( const Storage& aStorage ): myData(&aStorage)
        {
        }

        /**
         * Copy constructor.
         * @param other the iterator to clone.
         */
	
        PointsRange( const PointsRange & aOther )
	      : myData( aOther.myData )
        {
        }
      
        /**
         * Assignment.
         * @param other the iterator to copy.
         * @return a reference on 'this'.
         */
	
        PointsRange& operator= ( const PointsRange & other )
        {	
	        if ( this != &other )
	          {
              myData = other.myData;
	          }
	        return *this;
        }


	
        /**
         * Destructor. Does nothing.
         */
	
        ~PointsRange()
        {
        }

      // ------------------------- private data --------------------------------
        private: 
          const typename GridCurve::Storage* myData;

      // ------------------------- iterator services --------------------------------
        public:

      /**
       * Iterator service.
       * @return begin iterator
       */
      ConstIterator begin() const {
        return myData->begin();
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      ConstIterator end() const {
        return myData->end();
      }

    };

	
    ///////////////////////////////////////////////////////////////////////////////
    // end of class PointsRange
    ///////////////////////////////////////////////////////////////////////////////
	
  /**
   * Accessor of a range of grid points
   * @return PointsRange
   */
   typename GridCurve::PointsRange getPointsRange() const {
    return PointsRange(myData);
   } 



  }; // end of class GridCurve




  /**
   * Overloads 'operator<<' for displaying objects of class 'GridCurve'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GridCurve' to write.
   * @return the output stream after the writing.
   */
  template<typename KSpace>
  std::ostream&
  operator<< ( std::ostream & out, const GridCurve<KSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/geometry/2d/GridCurve.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GridCurve_h

#undef GridCurve_RECURSES
#endif // else defined(GridCurve_RECURSES)
