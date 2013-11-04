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

/**
 * @file testClone2.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 *
 * @date 2012/07/02
 *
 * This file is part of the DGtal library
 */

//#define TRACE_BITS

#include <cstdio>
#include <cmath>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/CowPtr.h"
#include "DGtal/base/Clone.h"
#include "DGtal/base/Alias.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/helpers/StdDefs.h"

using namespace DGtal;
using namespace std;


// New clone.
namespace DGtal {
  /**
     Performs without unnecessary duplicates "parameter -> member data"
     - A & -> A &                 // no duplication
     - A & -> A*                  // no duplication
     - CountedPtr<A> -> CountedPtr<A> // shared
     - A* -> A*                   // no duplication
     - A* -> A&                   // no duplication, exception if null
  */
  template <typename T>
  struct PAlias {
  };

  /**
     Performs without unnecessary duplicates "parameter -> member data"
     - const A & -> const A &     // no duplication
     - const A & -> const A*      // no duplication
     - const A* -> const A &      // no duplication, exception if null
     - const A* -> const A*       // no duplication
     - CountedPtr<A> -> CowPtr<A> // potential lazy duplication
     - CowPtr<A> -> CowPtr<A>     // potential lazy duplication
  */
  template <typename T>
  struct PConstAlias {
  };

  /**
     Performs without unnecessary duplicates "parameter -> member data"
     - const A & -> A             // immediate duplication 
     - const A & -> CowPtr<A>     // immediate duplication
     - CowPtr<A> -> A             // immediate duplication
     - CowPtr<A> -> CowPtr<A>     // lazy duplication
     - CountedPtr<A> -> A         // immediate duplication
     - CountedPtr<A> -> CowPtr<A> // lazy duplication
     - const A & -> A*            // immediate duplication, should be deleted at the end.            
     - CowPtr<A> -> A*            // immediate duplication, should be deleted at the end.            
     - CountedPtr<A> -> A*        // immediate duplication, should be deleted at the end.          
     - A* -> A*                   // acquired, should be deleted at the end.
     - A* -> A                    // immediate duplication, acquired and deleted.
     - A* -> CowPtr<A>            // acquired

     It uses a pair (const void*,enum), which does not use dynamic
     allocation. 

     @note (Speed) Even on a small type (here a pair<int,int>), it is
     much faster than NClone and has the advantage (wrt Clone<T>) to
     handle nicely both const T& and CowPtr<T> as input. It may be
     slightly slower than Clone (and by value or by const ref
     parameter passing) for small objects like a pair<int,int>. This
     is certainly due to the fact that it uses one more integer
     register for \a myParam data member.

     +--------+----------+----------+-----------+--------+--------+
     | Type   | Context  | value    | const ref | Clone  | PClone |
     +--------+----------+----------+-----------+--------+--------+
     | 2xint  |i7 2.4GHz |    48ms |     48ms  |   48ms |   59ms |
     |2xdouble|i7 2.4GHz |    48ms |     48ms  |   48ms |   49ms |
     | 2xint  |Xeon 2.67GHz|    54ms |     54ms  |   54ms |   54ms |
     |2xdouble|Xeon 2.67GHz|    54ms |     54ms  |   54ms | 53.5ms |
     +--------+----------+----------+-----------+--------+--------+


     @note It prevents direct assignment to CountedPtr<T> since their
     meaning is "shared_ptr". A discussion is possible for input pointer: is
     there pointer acquisition in this case ?
  */
  template <typename T> 
  class PClone
  {
    enum Parameter { CONST_LEFT_VALUE_REF, COW_PTR, COUNTED_PTR, RIGHT_VALUE_REF, CLONE_IS_ERROR }; // ACQ
  public:
    inline ~PClone() {}
    inline PClone( const PClone & ) : myParam( CLONE_IS_ERROR ), myPtr( 0 ) { ASSERT( false ); }
    inline PClone( const T & t ) 
      : myParam( CONST_LEFT_VALUE_REF ), myPtr( static_cast<const void*>( &t ) ) {}
    inline PClone( const CowPtr<T> & t ) 
      : myParam( COW_PTR ), myPtr( static_cast<const void*>( &t ) ) {}
    inline PClone( const CountedPtr<T> & t ) 
      : myParam( COUNTED_PTR ), myPtr( static_cast<const void*>( &t ) ) {}
#ifdef CPP11_AUTO
    inline PClone( T && t ) : myParam( RIGHT_VALUE_REF ), myPtr( static_cast<const void*>( &t ) ) {}
#endif

    inline operator T() const 
    {
      switch( myParam ) {
      case CONST_LEFT_VALUE_REF: return T( * static_cast< const T* >( myPtr ) );
      case COW_PTR:   return T( * static_cast< const CowPtr<T>* >( myPtr )->get() );
      case COUNTED_PTR:   return T( * static_cast< const CountedPtr<T>* >( myPtr )->get() );
#ifdef CPP11_AUTO
      case RIGHT_VALUE_REF:   return T( std::move( * const_cast<T*>( static_cast< const T* >( myPtr ) ) ) );
#endif
      default: ASSERT( false );
        return T( * static_cast< const T* >( myPtr ) );
      }
    }
    inline operator CowPtr<T>() const 
    {
      switch( myParam ) {
      case CONST_LEFT_VALUE_REF: return CowPtr<T>( new T( * static_cast< const T* >( myPtr ) ) );
      case COW_PTR:   return CowPtr<T>( * static_cast< const CowPtr<T>* >( myPtr ) );
      case COUNTED_PTR:   return CowPtr<T>( * static_cast< const CountedPtr<T>* >( myPtr ) );
#ifdef CPP11_AUTO
      case RIGHT_VALUE_REF: return CowPtr<T>( new T( std::move( * const_cast<T*>( static_cast< const T* >( myPtr ) ) ) ) );
#endif
      default: ASSERT( false );
        return CowPtr<T>( new T( * static_cast< const T* >( myPtr ) ) );
      }
    }

  private:
    const Parameter myParam;
    const void* const myPtr;
  };

  /**
     Performs without unnecessary duplicates "parameter -> member data"
     - const A & -> A 
     - const A & -> CowPtr<A> 
     - CowPtr<A> -> A
     - CowPtr<A> -> CowPtr<A>
     It uses an intermediate CowPtr<A>, which forces dynamic
     allocation. This slows down too much for small objects.
  */
  template <typename T>
  class NClone 
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor. Does nothing.
     */
    inline ~NClone() {}

    /**
      Copy constructor.
      @param other the object to clone.
    */
    inline NClone ( const NClone & ) { ASSERT( false ); }

    /**
       Constructor from an instance of T. The object is referenced in
       'this' and is generally immediately duplicated by the user to
       instantiate a data member.
       @param t any object of type T.
    */
    inline NClone( const T & t ) : myCowPtr( const_cast<T*>( &t ) ) {}

    /**
       Constructor from a pointer to a valid instance of T. The object is referenced in
       'this' and is generally immediately duplicated by the user to
       instantiate a data member.
       @param ptrT any valid pointer to a object of type T.
       @pre ptrT != 0
    */
    // NClone( const T* ptrT );

    inline NClone( CountedPtr<T> ptr ) : myCowPtr( ptr ) {}
    inline NClone( CowPtr<T> ptr ) : myCowPtr( ptr ) {}

    /**
       Cast operator to a T instance. This is only at this moment that
       the object is duplicated.  This allows things like: A a2 = a1;
       where a1 is of type NClone<A>.
    */
    inline operator T() const 
    { return myCowPtr.unique()
        ? T( *( const_cast< CowPtr<T>& >( myCowPtr ).drop() ) )
        : T( *( myCowPtr.get() ) );
    }
    // { return T( *( const_cast< CowPtr<T>& >( myCowPtr ).drop() ) ); }

    /**
       Cast operator to a CowPtr<T> instance. If the clone was initialized with an This is only at this moment that
       the object is duplicated (and only once).  This allows things like: CountedPtr<A> a2 = a1;
       where a1 is of type NClone<A>. It also allows CowPtr<A> a2 = a1;
    */
    inline operator CowPtr<T>() const 
    { return myCowPtr.unique()
        ? CowPtr<T>( new T( *( const_cast< CowPtr<T>& >( myCowPtr ).drop() ) ) )
        : myCowPtr;
    }
    //{ return myCowPtr; }

    //inline operator CountedPtr<T>() { return CountedPtr<T>( myCowPtr.get() ); }

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /// The copy-on-write pointer to the T object that is to be duplicated.
    CowPtr<T> myCowPtr;


    // ------------------------- Hidden services ------------------------------
  private:

    /**
     * Constructor.
     * Forbidden.
     */
    NClone();


    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden (otherwise the user might be tempted to use it as a member).
     */
    NClone & operator= ( const NClone & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class NClone

} // namespace DGtal


// Dummy class to test clones and aliases.
struct A1
{
private:
  A1();
public:
  ~A1()
  {
    std::cout << "  ~A1() " << std::endl;
    ++nbDeleted;
  }
  A1( int i ) : data( i )
  {
    std::cout << "  A1( int i ) " << std::endl;
    ++nbCreated;
  }
  A1( const A1 & a ) : data( a.data )
  {
    std::cout << "  A1( const A1 & a ) " << std::endl;
    ++nbCreated;
  }
#ifdef CPP11_AUTO
  A1( A1 && a ) noexcept : data( std::move( a.data ) ) 
  {
    std::cout << "  A1( A1 && a ) " << std::endl;
    ++nbCreated;
  }
#endif

  A1& operator=( const A1 & a ) 
  {
    data = a.data;
    std::cout << "  A1::operator=( const A1 & a ) " << std::endl;
    return *this;
  }
  static void reset() {
    nbCreated = 0; 
    nbDeleted = 0;
  }

  int data;

  static int nbCreated;
  static int nbDeleted;
};

int A1::nbCreated = 0;
int A1::nbDeleted = 0;

// Immediate duplication.
struct ToValueMember {
  inline ToValueMember( PClone<A1> a1 ) : myA1( a1 ) {}
  inline int value() const { return myA1.data; }
  A1 myA1;
};

// Immediate duplication.
struct ToCountedMember { // requires explicit duplication
  inline ToCountedMember( PClone<A1> a1 ) // : myA1( a1 ) {} does not compile
    : myA1( new A1( a1 ) ) {}
  inline int value() const { return myA1->data; }
  CountedPtr<A1> myA1;
};

// Immediate or lazy duplication.
struct ToCowMember {
  inline ToCowMember( PClone<A1> a1 ) : myA1( a1 ) {}
  inline int value() const { return myA1->data; }
  inline void setValue( int v ) { myA1->data = v; }
  CowPtr<A1> myA1;
};



class MyPoint {
public:
  ~MyPoint() 
  { nbDeleted++; }
  MyPoint( const MyPoint & other ) 
    : _x( other._x ), _y( other._y )
  { nbCreated++; }
  MyPoint( int x, int y ) : _x( x ), _y( y )
  { nbCreated++; }
  MyPoint operator-( const MyPoint & other ) const
  {
    return MyPoint( _x - other._x, _y - other._y );
  }
  double norm() const
  {
    double dx = (double) _x;
    double dy = (double) _y;
    return sqrt( dx * dx + dy * dy );
  }
  static void reset()
  {
    nbCreated = nbDeleted = 0;
  }
  int _x, _y;

  static int nbCreated;
  static int nbDeleted;
};

int MyPoint::nbCreated = 0;
int MyPoint::nbDeleted = 0;

class MyPointD {
public:
  ~MyPointD() 
  { nbDeleted++; }
  MyPointD( const MyPointD & other ) 
    : _x( other._x ), _y( other._y )
  { nbCreated++; }
  MyPointD( int x, int y ) : _x( x ), _y( y )
  { nbCreated++; }
  MyPointD( double x, double y ) : _x( x ), _y( y )
  { nbCreated++; }
  MyPointD operator-( const MyPointD & other ) const
  {
    return MyPointD( _x - other._x, _y - other._y );
  }
  double norm() const
  {
    double dx = (double) _x;
    double dy = (double) _y;
    return sqrt( dx * dx + dy * dy );
  }
  static void reset()
  {
    nbCreated = nbDeleted = 0;
  }
  double _x, _y;

  static int nbCreated;
  static int nbDeleted;
};

int MyPointD::nbCreated = 0;
int MyPointD::nbDeleted = 0;

//typedef Z2i::Point Point;
typedef MyPointD Point;


struct TriangleByConstReference {
  TriangleByConstReference( const Point & a, const Point & b, const Point & c )
    : _a( a ), _b( b ), _c( c ) {}
  double perimeter() const
  {
    return (_a - _b).norm() + (_b - _c).norm() + (_c - _a).norm();
  }
  Point _a, _b, _c;
};

struct TriangleByValue {
  TriangleByValue( Point a, Point b, Point c )
    : _a( a ), _b( b ), _c( c ) {}
  double perimeter() const
  {
    return (_a - _b).norm() + (_b - _c).norm() + (_c - _a).norm();
  }
  Point _a, _b, _c;
};

struct TriangleByClone {
  TriangleByClone( Clone<Point> a, Clone<Point> b, Clone<Point> c )
    : _a( a ), _b( b ), _c( c ) {}
  double perimeter() const
  {
    return (_a - _b).norm() + (_b - _c).norm() + (_c - _a).norm();
  }
  Point _a, _b, _c;
};

struct TriangleByPClone {
  TriangleByPClone( PClone<Point> a, PClone<Point> b, PClone<Point> c )
    : _a( a ), _b( b ), _c( c ) {}
  double perimeter() const
  {
    return (_a - _b).norm() + (_b - _c).norm() + (_c - _a).norm();
  }
  Point _a, _b, _c;
};

struct TriangleByPCloneAndCow {
  TriangleByPCloneAndCow( PClone<Point> a, PClone<Point> b, PClone<Point> c )
    : _a( a ), _b( b ), _c( c ) {}
  double perimeter() const
  {
    return (*_a - *_b).norm() + (*_b - *_c).norm() + (*_c - *_a).norm();
  }
  CowPtr<Point> _a, _b, _c;
};

template <typename Triangle>
double
computeTriangles( int size )
{
  double total = 0.0;
  Point A( 0, 0 );
  for ( int yb = 0; yb < size; ++yb ) 
    for ( int xb = 0; xb < size; ++xb ) 
      {
        Point B( xb, yb );
        for ( int yc = 0; yc < size; ++yc ) 
          for ( int xc = 0; xc < size; ++xc )
            {
              Point C( xc, yc );
              Triangle T( A, B, C );
              total += T.perimeter();
            }
      }
  return total;
}

template <typename Triangle>
double
computeTrianglesByCowPtr( int size )
{
  double total = 0.0;
  CowPtr<Point> A( new Point( 0, 0 ) );
  for ( int yb = 0; yb < size; ++yb ) 
    for ( int xb = 0; xb < size; ++xb ) 
      {
        CowPtr<Point> B( new Point( xb, yb ) );
        for ( int yc = 0; yc < size; ++yc ) 
          for ( int xc = 0; xc < size; ++xc )
            {
              CowPtr<Point> C( new Point( xc, yc ) );
              Triangle T( A, B, C );
              total += T.perimeter();
            }
      }
  return total;
}


int main()
{
  unsigned int nb = 0;
  unsigned int nbok = 0;
  A1 a1( 10 ); // +1/0
  CowPtr<A1> cow_a1( new A1( 5 ) ); // +1/0
  CountedPtr<A1> counted_a1( new A1( 12 ) ); // +1/0
  A1::reset();

  trace.beginBlock ( "PClone: #A1 with (const A1 &) to A1 member. Duplication (+1/0)" );
  ToValueMember c00( a1 ); // +1/0
  trace.info() << "D: d1.value() = " << c00.value() << std::endl;
  ++nb, nbok += A1::nbCreated==1 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

  trace.beginBlock ( "PClone: #A1 with (CountedPtr<A1>) to A1 member. Duplication (+1/0)" );
  ToValueMember c30( a1 ); // +1/0
  trace.info() << "D: d1.value() = " << c30.value() << std::endl;
  ++nb, nbok += A1::nbCreated==2 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

  trace.beginBlock ( "PClone: #A1 with (const A1 &) to CountedPtr<A1> member. Duplication (+1/0)" );
  ToCountedMember c01( a1 ); // +1/0
  trace.info() << "D: d1.value() = " << c01.value() << std::endl;
  ++nb, nbok += A1::nbCreated==3 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

  trace.beginBlock ( "PClone: #A1 with (const A1 &) to CowPtr<A1> member. Duplication (+1/0)" );
  ToCowMember c02( a1 ); // +1/0
  trace.info() << "D: d1.value() = " << c02.value() << std::endl;
  ++nb, nbok += A1::nbCreated==4 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

  trace.beginBlock ( "PClone: #A1 with (CowPtr<A1> &) to CowPtr<A1> member. Lazy duplication (0/0)" );
  ToCowMember c22( cow_a1 ); // +0/0
  trace.info() << "D: d1.value() = " << c22.value() << std::endl;
  ++nb, nbok += A1::nbCreated==4 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  c22.setValue( 17 );
  trace.info() << "D: d1.setValue( 17 ) -> now duplicating " << std::endl;
  trace.info() << "D: d1.value() = " << c22.value() << std::endl;
  ++nb, nbok += A1::nbCreated==5 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

  trace.beginBlock ( "PClone: #A1 with (CountedPtr<A1> &) to CowPtr<A1> member. Lazy duplication (0/0)" );
  ToCowMember c32( counted_a1 ); // +0/0
  trace.info() << "D: d1.value() = " << c32.value() << std::endl;
  ++nb, nbok += A1::nbCreated==5 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  c32.setValue( 21 );
  trace.info() << "D: d1.setValue( 21 ) -> now duplicating " << std::endl;
  trace.info() << "D: d1.value() = " << c32.value() << std::endl;
  ++nb, nbok += A1::nbCreated==6 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();

#ifdef CPP11_AUTO
  trace.beginBlock ( "PClone: #A1 with (A1 &&) to A1 member. Duplication (+1/0)" );
  ToValueMember c40( A1( -4 ) ); // +1/0
  trace.info() << "D: d1.value() = " << c40.value() << std::endl;
  ++nb, nbok += A1::nbCreated==7 ? 1 : 0;
  ++nb, nbok += A1::nbDeleted==0 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " nbCreated=" << A1::nbCreated 
               << " nbDeleted=" << A1::nbDeleted << std::endl; 
  trace.endBlock();
#endif

  int size = 40;
  trace.beginBlock ( "Total perimeter of triangles with by-value parameter passing." );
  double t1 = computeTriangles<TriangleByValue>( size );
  trace.info() << "Perimeter is " << t1 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  int nbC = Point::nbCreated;
  Point::reset();
  trace.endBlock();
  trace.beginBlock ( "Total perimeter of triangles with by-const reference parameter passing." );
  double t2 = computeTriangles<TriangleByConstReference>( size );
  trace.info() << "Perimeter is " << t2 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  ++nb, nbok += Point::nbCreated < nbC ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  Point::reset();
  trace.endBlock();
  trace.beginBlock ( "Total perimeter of triangles with by Clone parameter passing." );
  double t3 = computeTriangles<TriangleByClone>( size );
  trace.info() << "Perimeter is " << t3 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  ++nb, nbok += Point::nbCreated < nbC ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  Point::reset();
  trace.endBlock();
  trace.beginBlock ( "Total perimeter of triangles with by PClone parameter passing." );
  double t4 = computeTriangles<TriangleByPClone>( size );
  trace.info() << "Perimeter is " << t4 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  ++nb, nbok += Point::nbCreated < nbC ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  Point::reset();
  trace.endBlock();
  trace.beginBlock ( "Total perimeter of triangles with by PCloneAndCow parameter passing." );
  double t5 = computeTriangles<TriangleByPCloneAndCow>( size );
  trace.info() << "Perimeter is " << t5 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  ++nb, nbok += Point::nbCreated < nbC ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  Point::reset();
  trace.endBlock();
  trace.beginBlock ( "Total perimeter of triangles with CowPtr by PCloneAndCow parameter passing." );
  double t6 = computeTrianglesByCowPtr<TriangleByPCloneAndCow>( size );
  trace.info() << "Perimeter is " << t6 << std::endl;
  ++nb, nbok += Point::nbCreated == Point::nbDeleted ? 1 : 0;
  ++nb, nbok += Point::nbCreated < nbC ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ")"
               << " Point nbCreated=" << Point::nbCreated 
               << " nbDeleted=" << Point::nbDeleted << std::endl; 
  Point::reset();
  trace.endBlock();



  return ( nb == nbok ) ? 0 : 1;
}
/** @ingroup Tests **/
