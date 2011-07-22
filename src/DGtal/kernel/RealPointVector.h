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
 * @file RealPointVector.h
 * @author David Coeurjolly (@c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/14
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(RealPointVector_RECURSES)
#error Recursive header files inclusion detected in RealPointVector.h
#else // defined(RealPointVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RealPointVector_RECURSES

#if !defined RealPointVector_h
/** Prevents repeated inclusion of headers. */
#define RealPointVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <boost/array.hpp>

#include "DGtal/base/Common.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/boards/DGtalBoard.h"

#ifdef WITH_VISU3D_QGLVIEWER
#include "DGtal/io/viewers/Viewer3D.h"
#endif


//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
 
  /////////////////////////////////////////////////////////////////////////////
  // class RealPointVector
  /**
   * Description of class 'RealPointVector' <p>
   *
   * @brief Aim: Implements basic operations that will be used in 
   * Point and  Vector classes with double values.
   *
   * This class is a refinement of the PointVector<dim,double>> type.
   *
   * @tparam dim static dimension of the point/vector.
   *
   */
  template < DGtal::Dimension dim >
  class RealPointVector: public PointVector<dim,double>
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    typedef RealPointVector<dim> Self;
 
    typedef double Component;
    typedef double Coordinate;

    ///Copy of the dimension type
    typedef DGtal::Dimension Dimension;

    ///Copy of the static dimension of the Point/Vector.
    static const Dimension dimension = dim;

    /**
     * Constructor.
     */
    RealPointVector();

    /**
     * Constructor from array of values.
     *
     * @param ptrValues the array of values.
     */
    explicit RealPointVector( const Component* ptrValues );

    /**
     * Constructor from two values (the Dimension of the vector should
     * be at least 2). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     */
    RealPointVector( const Component & x, const Component & y );

    /**
     * Constructor from three values (the Dimension of the vector should
     * be at least 3). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     * @param z the third value.
     */
    RealPointVector( const Component & x, const Component & y, const Component & z );

    /**
     * Constructor from four values (the Dimension of the vector should
     * be at least 4). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     * @param z the third value.
     * @param t the fourth value.
     */
    RealPointVector( const Component & x, const Component & y, 
		     const Component & z, const Component & t );

#ifdef CPP0X_INITIALIZER_LIST
    /**
     * Constructor from initializer list.
     * @param the initializer list.
     */
    RealPointVector( std::initializer_list<Component> init );
#endif // CPP0X_INITIALIZER_LIST

    /** Constructor taking apoint and a functor as parameters.
     *  The new point is initialized by the result of functor f
     *  for each coordinate of apoint1 and apoint2
     */
    template<typename Functor>
    RealPointVector( const Self& apoint1, const Self& apoint2,
		     const Functor& f );


    /**
     * Constructor from PointVector.
     *
     * @param v is the PointVector used to construct the RealPointVector.
     *
     */
    template<typename AnotherComponent>
    RealPointVector( const PointVector<dim,AnotherComponent> & v );
    
    // ------------------------- Specific operators -------------------------------
    
    /**
     * Assignment operator from PointVector.
     *
     * @param v is the Point that gets divided to @a *this.
     * @return a reference on 'this'.
     */
    template<typename AnotherComponent>
    Self & operator= ( const PointVector<dim,AnotherComponent> & v );
   
    
    /**
     * Division operator with assignement.
     *
     * @param v is the Point that gets divided to @a *this.
     * @return a reference on 'this'.
     */
    Self & operator/= ( const Self & v );

     /**
     * Division operator.
     *
     * @param v is the Point that gets divided to @a *this.
     * @return the component division of *this by v.
     */
    Self  operator/ ( const Self & v ) const ;
  
    /**
     * Divides @a *this by the @a coeff scalar number.
     *
     * @param coeff is the factor @a *this get divided by.
     * @return a reference on 'this'.
     */
    Self & operator/= ( Component coeff )
    {
      for ( Dimension i = 0; i < dimension; ++i )
	this->myArray[ i ] /= coeff;
      return *this;
    }
    // ------------------------- Specific methods -------------------------------
  

    enum NormType { L_2, L_1, L_infty };
    
    /**
     * Computes the norm of a point/vector.
     * \warning This method performs a conversion
     * from the type T to double for each components to compute the
     * norms. For exact norms (restricted to L_1 and L_infinity
     * norms), please refer to RealPointVector::norm1 and RealPointVector::normInfinity. 
     *
     * @param type specifies the type of norm to consider (see @ref NormType).
     * @return the norm of the point/vector as a double.
     */
    double norm( const NormType type = L_2 ) const;

    /**
     * Computes the 1-norm of a vector.
     *
     * @return the absolute sum of the components of this vector.
     */
    double norm1() const;

    /**
     * Computes the infinity-norm of a vector.
     *
     * @return the maximum absolute value of the components of this vector.
     */
    double normInfinity() const;

    /**
     * Normalize a real vector using its Euclidean norm.
     *
     */
    void normalize()
    {
      double length=this->norm();
      for ( Dimension i = 0; i < dimension; ++i )
	this->myArray[ i ] /= length;
    }
  

    // ------------------------- Private Datas -------------------------------
  private:

    /**
     * Default styles.
     */
    struct DefaultDrawStylePaving : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
	aBoard.setPenColorRGBi(160,160,160);
	aBoard.setLineStyle( DGtalBoard::Shape::SolidStyle );
	aBoard.setFillColorRGBi(220,220,220);
	aBoard.setLineWidth(1);
      }
    };

    
    struct DefaultDrawStyleGrid : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
	aBoard.setPenColor(Color::Black);
	aBoard.setLineStyle( DGtalBoard::Shape::SolidStyle );
      }
    };



    // --------------- CDrawableWithDGtalBoard realization -------------------
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
     * Draw the object on a DGtalBoard board.
     * @param board the output board where the object is drawn.
     */
    void selfDraw( DGtalBoard & board ) const;

    
    /**
     * Draw a pixel as a unit square on a DGtalBoard board.
     * @param board the output board where the object is drawn.
     */
    
    void selfDrawAsPaving( DGtalBoard & board ) const;
    
    
    /**
     * Draw a pixel as a point on a LiBoard board
     * @param board the output board where the object is drawn.
     */
    void selfDrawAsGrid( DGtalBoard & board ) const;
    
    
#ifdef WITH_VISU3D_QGLVIEWER

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithViewer3D* defaultStyleQGL( std::string mode = "" ) const;

    /**
     * Draw the object on a DGtalBoard board.
     * @param board the output board where the object is drawn.
     */
    void selfDrawQGL ( Viewer3D & viewer ) const;
    void selfDrawQGL ( Viewer3D & viewer, const Self &startingPoint ) const;
    void selfDrawAsGridQGL( Viewer3D & viewer  ) const;
    void selfDrawAsPavingQGL( Viewer3D & viewer ) const;

#endif

    // ----------------------- Interface --------------------------------------
  public:
    /**
     * Draw the object (as a Vector from aPoint) on a DGtalBoard board
     *
     * @param board the output board where the object is drawn.
     * @param startingPoint the starting point of the vector
     * @tparam Functor a Functor to specialize the Board style
     */
    void selfDraw( DGtalBoard & board, const Self &startingPoint ) const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /// Static const for zero RealPointVector.
    static Self zero;
    
    // ------------------------- Hidden services ------------------------------
  private:
    
    ///Internal data-structure: boost/array with constant size.
    //  boost::array<Component, dimension> myArray;
    
  }; // end of class RealPointVector

  
  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawPavingRealPixel : public DrawWithBoardModifier {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myModes[ "RealPointVector" ] = "Paving";
    }
  };
  
  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawGridRealPixel : public DrawWithBoardModifier {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myModes[ "RealPointVector" ] = "Grid";
    }
  };




  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */

#ifdef WITH_VISU3D_QGLVIEWER

  struct DrawPavingRealVoxel : public DrawableWithViewer3D {
    void selfDrawQGL( Viewer3D & viewer ) const
    {
      viewer.myModes[ "RealPointVector" ] = "Paving";
    }
  };
  
  
  struct DrawGridRealVoxel : public DrawableWithViewer3D {
    void selfDrawQGL( Viewer3D & viewer ) const
    {
      viewer.myModes[ "RealPointVector" ] = "Grid";
    }
  };

  struct DefaultDrawStyleRealGrid3D : public DrawableWithViewer3D {

    virtual void selfDrawQGL( Viewer3D & viewer ) const
    {
      //aBoard.setPenColor(Color::Black);
      //aBoard.setLineStyle( DGtalBoard::Shape::SolidStyle );
    }
  };



#endif
  
  /// Operator <<
  template<DGtal::Dimension dim>
  std::ostream&
  operator<<( std::ostream & out, const RealPointVector<dim> & object );

  template< DGtal::Dimension dim>
  RealPointVector<dim>  RealPointVector<dim>::zero;
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/RealPointVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RealPointVector_h

#undef RealPointVector_RECURSES
#endif // else defined(RealPointVector_RECURSES)
