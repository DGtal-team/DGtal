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
#include <utility>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/topology/KhalimskySpaceND.h"

#include "DGtal/io/boards/Board2D.h"



//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class GridCurve
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'GridCurve' <p> Aim: describes an
   * alternative sequence of signed 0-cell (pointels) and 1-cell (linels)
   * in any dimension, closed or open. For instance, the
   * topological boundary of a  simply connected digital set is a
   * closed grid curve. This object provides several ranges, such as
   * PointsRange used to get the (integer) coordinates of the pointels
   * of the grid curve. 
   *
   * Example :
   * @code 

   * @endcode
   */

  template <typename KSpace>
  class GridCurve
  {

  public: 
    typedef typename KSpace::Space::Point Point;
    typedef typename KSpace::Space::Point Vector;

    typedef typename KSpace::SCell SCell;
    typedef typename std::vector<SCell> Storage;





    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GridCurve(){};

    /**
     * Constructor.
     */
    GridCurve(const KSpace& aKSpace) : myK(aKSpace) {};

    /**
     * Default Constructor.
     */
    GridCurve(){};


    /**
     * Init.
     * @param aVectorOfPoints the vector containing the sequence of grid points. 
     */
    bool initFromVector( const std::vector<Point>& aVectorOfPoints ) throw(ConnectivityException);

    /**
     * Init.
     * @param in any input stream,
     */
    bool initFromVectorStream(std::istream & in );


    /**
     * Outputs the grid curve to the stream [out].
     * @param out any output stream,
     */
    void writeVectorToStream( std::ostream & out );

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


    /**
     * @return 'true' if grid curve is open, 'false' otherwise
     */
    bool isOpen() const;

    /**
     * @return 'true' if grid curve is closed, 'false' otherwise
     */
    bool isClosed() const;

     
    // ------------------------- private Datas --------------------------------
  private:


    // ------------------------- Public Datas --------------------------------
  public:
    KSpace myK;

    Storage my0SCells; 
    Storage my1SCells; 


    // ------------------------- Internal --------------------------------
  private:

    //conversion methods
    SCell PointTo0SCell(const Point& aPoint);
    SCell PointVectorTo1SCell(const Point& aPoint, const Vector& aVector);
    

    // ------------------------- Drawing services --------------------------------
  public: 


    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithBoard2D* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;
    
    /**
       Draw the object on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDraw(Board2D & board ) const;
    
    /**
       Draw the points on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDrawPoints(Board2D & board ) const; 
    /**
       Draw the grid edges on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDrawEdges(Board2D & board ) const;

  private: 

    /**
     * Default Style Functor for selfDraw methods
     *
     * @param aBoard
     */

    struct SelfDrawStyle
    {
      SelfDrawStyle(Board2D & aBoard)
      {
      }
    };

    struct DefaultDrawStyle : public DrawableWithBoard2D
    {
      virtual void selfDraw( Board2D & aBoard ) const
      {
      }
    };

    struct DefaultDrawStylePoints : public DrawableWithBoard2D
    {
      virtual void selfDraw( Board2D &  ) const
      {
      }
    };

    struct DefaultDrawStyleEdges : public DrawableWithBoard2D
    {
      virtual void selfDraw( Board2D & aBoard ) const
      {
        aBoard.setLineStyle (LibBoard::Shape::SolidStyle );
        aBoard.setFillColor( DGtal::Color::None);
      }
    };

    // ------------------------- inner classes --------------------------------

  public: 

    ///////////////////////////////////////////////////////////////////////////////
    // class SCellsRange
    ///////////////////////////////////////////////////////////////////////////////


    /**
     * This class is a model of CRange and thus provides a ConstIterator to scan 
     * the Khalimsky coordinates of the d-cells of a grid curve 
     */

   
    class SCellsRange
    {

      // ------------------------- inner types --------------------------------
    public: 
      typedef typename GridCurve::Storage Storage; 
      typedef typename GridCurve::Storage::const_iterator ConstIterator;
      typedef typename GridCurve::Storage::const_reverse_iterator ConstReverseIterator;

      /**
       * Default Constructor.
       */
  
      SCellsRange(){}

      /**
       * Constructor.
       */
  
      SCellsRange( const Storage& aStorage ): myData(&aStorage){}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
  
      SCellsRange( const SCellsRange & aOther )
        : myData( aOther.myData ){}
      
      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
  
      SCellsRange& operator= ( const SCellsRange & other )
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
  
      ~SCellsRange() {}

      /**
       * @return the size of the range
       */
  
      typename Storage::size_type size() const {
	return myData->size();
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

      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin() const {
        return myData->rbegin();
      }

      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const {
        return myData->rend();
      }

    };

  
    ///////////////////////////////////////////////////////////////////////////////
    // end of class SCellsRange
    ///////////////////////////////////////////////////////////////////////////////
  
    /**
     * Accessor of a range of 0-cells
     * @return SCellsRange
     */
    typename GridCurve::SCellsRange get0SCellsRange() const {
      return SCellsRange(my0SCells);
    } 

    /**
     * Accessor of a range of 1-cells
     * @return SCellsRange
     */
    typename GridCurve::SCellsRange get1SCellsRange() const {
      return SCellsRange(my1SCells);
    } 

    ///////////////////////////////////////////////////////////////////////////////
    // class PointsRange
    ///////////////////////////////////////////////////////////////////////////////


    /**
     * This class is a model of CRange and provides a ConstIterator to scan 
     * the integer coordinates of the pointels of a grid curve 
     */

   
    class PointsRange
    {

      // ------------------------- inner types --------------------------------
    public: 

      typedef typename GridCurve::Storage Storage; 
      typedef typename GridCurve::Storage::const_iterator ConstIteratorOnPointels; 

      typedef typename GridCurve::Point Point; 

      ///////////////////////////////////////////////////////////////////////////////
      // class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      /**
       * This class is a model of CConstIteratorOnPoints  
       */
        
      class ConstIterator : 
	public std::iterator<std::bidirectional_iterator_tag, 
			     Point, unsigned int, Point*, Point >
      {

	// ------------------------- data -----------------------
      private:
    
	const GridCurve* myC;
	ConstIteratorOnPointels myIt; 

	// ------------------------- Standard services -----------------------
      public:

	/**
	 * Default Constructor.
	 */
	ConstIterator() {}

	/**
	 * Constructor.
	 */
	ConstIterator(const GridCurve* aGridCurve, const ConstIteratorOnPointels it)
	  : myC(aGridCurve),myIt(it) {}

	/**
	 * Copy constructor.
	 * @param other the iterator to clone.
	 */
  
	ConstIterator( const ConstIterator & aOther )
	  : myC(aOther.myC),myIt(aOther.myIt) {}
          
	/**
	 * Assignment.
	 * @param other the iterator to copy.
	 * @return a reference on 'this'.
	 */
  
	ConstIterator& operator= ( const ConstIterator & other )
	{  
	  if ( this != &other )
	    {
	      myIt = other.myIt;
	      myC = other.myC;
	    }
	  return *this;
	}

	/**
	 * Destructor. Does nothing.
	 */
  
	~ConstIterator(){}

	// ------------------------- iteration services -------------------------
      public:

	/**
	 * @return the current coordinates.
	 */
  
	Point operator*() const
	{
	  return Point( myC->myK.sCoords(*myIt) );
	}

	/**
	 * Pre-increment.
	 */

	ConstIterator& operator++()
	{
	  ++myIt;
	  return *this;
	}
  
	/**
	 * Post-increment.
	 */
  
	ConstIterator  operator++(int)
	{
	  ConstIterator tmp(*this);
	  myIt++;
	  return tmp;
	}

	/**
	 * Pre-decrement.
	 */
  
	ConstIterator&  operator--()
	{
	  --myIt;
	  return *this;
	}

	/**
	 * Post-decrement.
	 */
  
	ConstIterator  operator--(int)
	{
	  ConstIterator tmp(*this);
	  myIt--;
	  return tmp;
	}

	/**
	 * Equality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators coincide
	 */

	bool operator == ( const ConstIterator & aOther ) const
	{
	  return myIt == aOther.myIt;
	}

	/**
	 * Less than operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators follow this relation.
	 */

	bool operator < ( const ConstIterator & aOther ) const
	{
	  return myIt < aOther.myIt;
	}

	/**
	 * Inequality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators differs.
	 */

	bool operator!= ( const ConstIterator & aOther ) const
	{
	  return myIt != aOther.myIt;
	}

      };

      ///////////////////////////////////////////////////////////////////////////////
      // end class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      typedef typename std::reverse_iterator<ConstIterator> ConstReverseIterator;

      // ------------------------- standard services --------------------------------

      /**
       * Default Constructor.
       */
  
      PointsRange(){}

      /**
       * Constructor.
       */
  
      PointsRange(const GridCurve* aGridCurve ): myC(aGridCurve){}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
  
      PointsRange( const PointsRange & aOther )
        : myC( aOther.myC ){}
      
      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
  
      PointsRange& operator= ( const PointsRange & other )
      {  
	if ( this != &other )
	  {
	    myC = other.myC;
	  }
	return *this;
      }

      /**
       * Destructor. Does nothing.
       */
  
      ~PointsRange() {}

      /**
       * @return the size of the range
       */
  
      typename Storage::size_type size() const {
	return myC->my0SCells.size();
      }

      // ------------------------- private data --------------------------------
    private: 
      const GridCurve* myC;

      // ------------------------- iterator services --------------------------------
    public:

      /**
       * Iterator service.
       * @return begin iterator
       */
      ConstIterator begin() const {
        return ConstIterator( myC, myC->my0SCells.begin() );
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      ConstIterator end() const {
        return ConstIterator( myC, myC->my0SCells.end() );
      }

      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin() const {
        return ConstReverseIterator(this->end());
      }

      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const {
        return ConstReverseIterator(this->begin());
      }

    };

    ///////////////////////////////////////////////////////////////////////////////
    // end of class PointsRange
    ///////////////////////////////////////////////////////////////////////////////

    /**
     * Accessor of the range of the integer coordinates of the pointels
     * @return PointsRange
     */
    typename GridCurve::PointsRange getPointsRange() const {
      return PointsRange(this);
    } 

    ///////////////////////////////////////////////////////////////////////////////
    // class MidPointsRange
    ///////////////////////////////////////////////////////////////////////////////


    /**
     * This class is a model of CRange and provides a ConstIterator to scan 
     * the real coordinates of the midpoints of each 1-cells 
     */

   
    class MidPointsRange
    {

      // ------------------------- inner types --------------------------------
    public: 

      typedef typename DGtal::PointVector<GridCurve::Point::dimension, double> Point; 

      typedef typename GridCurve::Storage Storage; 
      typedef typename GridCurve::Storage::const_iterator ConstIteratorOn1SCells; 

      ///////////////////////////////////////////////////////////////////////////////
      // class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      /**
       * This class is a model of CConstIteratorOnMidPoints  
       */
        
      class ConstIterator : 
	public std::iterator<std::bidirectional_iterator_tag, 
			     Point, unsigned int, Point*, Point >
      {

	// ------------------------- data -----------------------
      private:
    
	const GridCurve* myC;
	ConstIteratorOn1SCells myIt; 

	// ------------------------- Standard services -----------------------
      public:

	/**
	 * Default Constructor.
	 */
	ConstIterator() {}

	/**
	 * Constructor.
	 */
	ConstIterator(const GridCurve* aGridCurve, const ConstIteratorOn1SCells it)
	  : myC(aGridCurve),myIt(it) {}

	/**
	 * Copy constructor.
	 * @param other the iterator to clone.
	 */
  
	ConstIterator( const ConstIterator & aOther )
	  : myC(aOther.myC),myIt(aOther.myIt) {}
          
	/**
	 * Assignment.
	 * @param other the iterator to copy.
	 * @return a reference on 'this'.
	 */
  
	ConstIterator& operator= ( const ConstIterator & other )
	{  
	  if ( this != &other )
	    {
	      myIt = other.myIt;
	      myC = other.myC;
	    }
	  return *this;
	}

	/**
	 * Destructor. Does nothing.
	 */
  
	~ConstIterator(){}

	// ------------------------- iteration services -------------------------
      public:

	/**
	 * @return the current coordinates.
	 */
  
	Point operator*() const
	{
	  Point p( myC->myK.sKCoords(*myIt) );
	  p /= 2;
	  return p;
	}

	/**
	 * Pre-increment.
	 */

	ConstIterator& operator++()
	{
	  ++myIt;
	  return *this;
	}
  
	/**
	 * Post-increment.
	 */
  
	ConstIterator  operator++(int)
	{
	  ConstIterator tmp(*this);
	  myIt++;
	  return tmp;
	}

	/**
	 * Pre-decrement.
	 */
  
	ConstIterator&  operator--()
	{
	  --myIt;
	  return *this;
	}

	/**
	 * Post-decrement.
	 */
  
	ConstIterator  operator--(int)
	{
	  ConstIterator tmp(*this);
	  myIt--;
	  return tmp;
	}

	/**
	 * Equality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators coincide
	 */

	bool operator == ( const ConstIterator & aOther ) const
	{
	  return myIt == aOther.myIt;
	}

	/**
	 * Inequality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators differs.
	 */

	bool operator!= ( const ConstIterator & aOther ) const
	{
	  return myIt != aOther.myIt;
	}

      };

      ///////////////////////////////////////////////////////////////////////////////
      // end class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      typedef typename std::reverse_iterator<ConstIterator> ConstReverseIterator;

      // ------------------------- standard services --------------------------------

      /**
       * Default Constructor.
       */
  
      MidPointsRange(){}

      /**
       * Constructor.
       */
  
      MidPointsRange(const GridCurve* aGridCurve ): myC(aGridCurve){}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
  
      MidPointsRange( const MidPointsRange & aOther )
        : myC( aOther.myC ){}
      
      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
  
      MidPointsRange& operator= ( const MidPointsRange & other )
      {  
	if ( this != &other )
	  {
	    myC = other.myC;
	  }
	return *this;
      }

      /**
       * Destructor. Does nothing.
       */
  
      ~MidPointsRange() {}

      /**
       * @return the size of the range
       */
  
      typename Storage::size_type size() const {
	return myC->my1SCells.size();
      }

      // ------------------------- private data --------------------------------
    private: 
      const GridCurve* myC;

      // ------------------------- iterator services --------------------------------
    public:

      /**
       * Iterator service.
       * @return begin iterator
       */
      ConstIterator begin() const {
        return ConstIterator( myC, myC->my1SCells.begin() );
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      ConstIterator end() const {
        return ConstIterator( myC, myC->my1SCells.end() );
      }

      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin() const {
        return ConstReverseIterator(this->end());
      }

      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const {
        return ConstReverseIterator(this->begin());
      }

    };

    ///////////////////////////////////////////////////////////////////////////////
    // end of class MidPointsRange
    ///////////////////////////////////////////////////////////////////////////////

    /**
     * Accessor of the range of the (real coordinates of the) midpoints of each 1-cell
     * @return MidPointsRange
     */
    typename GridCurve::MidPointsRange getMidPointsRange() const {
      return MidPointsRange(this);
    } 



    ///////////////////////////////////////////////////////////////////////////////
    // class ArrowsRange
    ///////////////////////////////////////////////////////////////////////////////


    /**
     * This class is a model of CRange and provides a ConstIterator to scan 
     * the 1-cells and return the integer coordinates of the associated 
     * point and displacement vector
     */

   
    class ArrowsRange
    {

      // ------------------------- inner types --------------------------------
    public: 

      typedef typename GridCurve::SCell SCell;
      typedef typename GridCurve::Point Point;
      typedef typename GridCurve::Point Vector;
      typedef typename std::pair<Point,Vector> Arrow; 

      typedef typename GridCurve::Storage Storage; 
      typedef typename GridCurve::Storage::const_iterator ConstIteratorOnSCells; 

      ///////////////////////////////////////////////////////////////////////////////
      // class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      /**
       * This class is a model of CConstIteratorOnArrows 
       */
        
      class ConstIterator : 
	public std::iterator<std::bidirectional_iterator_tag, 
			     Arrow, unsigned int, Arrow*, Arrow >
      {

	// ------------------------- data -----------------------
      private:
    
	const GridCurve* myC;
	ConstIteratorOnSCells myIt;

	// ------------------------- Standard services -----------------------
      public:

	/**
	 * Default Constructor.
	 */
	ConstIterator() {}

	/**
	 * Constructor.
	 */
	ConstIterator(const GridCurve* aGridCurve, const ConstIteratorOnSCells& it)
	  : myC(aGridCurve),myIt(it) {}

	/**
	 * Copy constructor.
	 * @param other the iterator to clone.
	 */
  
	ConstIterator( const ConstIterator & aOther )
	  : myC(aOther.myC),myIt(aOther.myIt) {}
          
	/**
	 * Assignment.
	 * @param other the iterator to copy.
	 * @return a reference on 'this'.
	 */
  
	ConstIterator& operator= ( const ConstIterator & other )
	{  
	  if ( this != &other )
	    {
	      myIt = other.myIt;
	      myC = other.myC; 
	    }
	  return *this;
	}

	/**
	 * Destructor. Does nothing.
	 */
  
	~ConstIterator(){}

	// ------------------------- iteration services -------------------------
      public:

	/**
	 * @return the current coordinates.
	 */
  
	Arrow operator*() const
	{
	  ASSERT(myC);

	  //starting point of the arrow
	  SCell pointel( myC->myK.sIndirectIncident( *myIt, *myC->myK.sDirs( *myIt ) ) );
	  Point p( myC->myK.sCoords( pointel ) );   //integer coordinates

	  //displacement vector
	  Vector v( myC->myK.sKCoords( *myIt ) - myC->myK.sKCoords( pointel ) );

	  return std::pair<Point,Vector>(p,v);
	}

	/**
	 * Pre-increment.
	 */

	ConstIterator& operator++()
	{
	  ++myIt;
	  return *this;
	}
  
	/**
	 * Post-increment.
	 */
  
	ConstIterator  operator++(int)
	{
	  ConstIterator tmp(*this);
	  myIt++;
	  return tmp;
	}

	/**
	 * Pre-decrement.
	 */
  
	ConstIterator&  operator--()
	{
	  --myIt;
	  return *this;
	}

	/**
	 * Post-decrement.
	 */
  
	ConstIterator  operator--(int)
	{
	  ConstIterator tmp(*this);
	  myIt--;
	  return tmp;
	}

	/**
	 * Equality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators coincide
	 */

	bool operator == ( const ConstIterator & aOther ) const
	{
	  return myIt == aOther.myIt;
	}

	/**
	 * Inequality operator.
	 * @param aOther the iterator to compare with 
	 * @return 'true' if their intern iterators differs.
	 */

	bool operator!= ( const ConstIterator & aOther ) const
	{
	  return myIt != aOther.myIt;
	}

      };

      ///////////////////////////////////////////////////////////////////////////////
      // end class ConstIterator
      ///////////////////////////////////////////////////////////////////////////////

      typedef typename std::reverse_iterator<ConstIterator> ConstReverseIterator;

      // ------------------------- standard services --------------------------------

      /**
       * Default Constructor.
       */
  
      ArrowsRange(){}

      /**
       * Constructor.
       */
  
      ArrowsRange(const GridCurve* aGridCurve ): myC(aGridCurve){}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
  
      ArrowsRange( const ArrowsRange & aOther )
        : myC( aOther.myC ){}
      
      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
  
      ArrowsRange& operator= ( const ArrowsRange & other )
      {  
	if ( this != &other )
	  {
	    myC = other.myC;
	  }
	return *this;
      }

      /**
       * Destructor. Does nothing.
       */
  
      ~ArrowsRange() {}

      /**
       * @return the size of the range
       */
  
      typename Storage::size_type size() const {
	return myC->my1SCells.size();
      }

      // ------------------------- private data --------------------------------
    private: 
      const GridCurve* myC;

      // ------------------------- iterator services --------------------------------
    public:

      /**
       * Iterator service.
       * @return begin iterator
       */
      ConstIterator begin() const {
        return ConstIterator( myC, myC->my1SCells.begin() );
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      ConstIterator end() const {
        return ConstIterator( myC, myC->my1SCells.end() );
      }

      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin() const {
        return ConstReverseIterator(this->end());
      }

      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const {
        return ConstReverseIterator(this->begin());
      }

    };

    ///////////////////////////////////////////////////////////////////////////////
    // end of class ArrowsRange
    ///////////////////////////////////////////////////////////////////////////////

    /**
     * Range of the pair of point and displacement vector
     * (integer coordinates) associated to the 1-cells 
     * @return ArrowsRange
     */
    typename GridCurve::ArrowsRange getArrowsRange() const {
      return ArrowsRange(this);
    } 


    //TODO
    /**
     * other ranges
     - ArrowsRange operator*(): std::pair<Point,Vector> (integer coordinates of the pointel and the displacement vector associated to the following 1-cell)
     - IncidentSpelsRange operator*(): std::vector<Point> (integer coordinates of the spels incident to a given 1-cell)
     - CodesRange operator*(): {0,1,2,3} (only in 2D using SFINAE)
     - InnerPointsRange, OuterPointsRange (only in 2D using SFINAE)

     * check the iterator + add operator-> and methods for random access iterator

     * set my0SCells,my1SCells private 
     and put GridCurve as a friend class in the ConstIterator classes

     * bounding box init for myK (in nd) ?
     and/or passing myK in the constructor

     * drawing: shift pb with dgtalboard in 2d / QGLviewer in 3d

     */

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
// Includes inline functions/methods.
#include "DGtal/geometry/2d/GridCurve.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GridCurve_h

#undef GridCurve_RECURSES
#endif // else defined(GridCurve_RECURSES)
