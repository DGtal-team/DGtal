#pragma once
#if defined(Lemniscate2D_RECURSES)
#error Recursive header files inclusion detected in Lemniscate2D.h
#else // defined(Lemniscate2D_RECURSES)
 
#define Lemniscate2D_RECURSES

#if !defined Lemniscate2D_h

#define Lemniscate2D_h

// Inclusions
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/shapes/parametric/StarShaped2D.h>
#include <cmath>

//namespace DGtal
//{

 // template class Lemniscate2D
template <typename TSpace>
class Lemniscate2D : public DGtal::StarShaped2D<TSpace>
{
// ----------------------- Standard services ------------------------------
  public:

    typedef TSpace Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint2D;
    typedef typename Space::RealVector RealVector2D;
    ~Lemniscate2D();
      
   Lemniscate2D( const double a, const double b,
	      const double x, const double y);
  
      
   // ------------- Implementation of 'StarShaped' services ------------------
    public:
  
      RealPoint2D getLowerBound() const
      {
        return RealPoint2D(-myA - myCenter[0] , -myB - myCenter[1] );
      }
  
      RealPoint2D getUpperBound() const
      {
        return RealPoint2D(myA - myCenter[0] , myB - myCenter[1]);
      }
   
      RealPoint2D center() const
       {
         return myCenter;
       }
      
       double parameter( const RealPoint2D & p ) const;
   
   
       RealPoint2D x( const double t ) const;
   
       RealVector2D xp( const double t ) const;
   
       RealVector2D xpp( const double t ) const;
       
   
       // ------------------------- data ----------------------------
     private:
   
       	RealPoint2D myCenter;
       
      	double myA;
	
	double myB;
   
       // ----------------------- Interface --------------------------------------
     public:
   
       void selfDisplay ( std::ostream & out ) const;
   
       bool isValid() const;
   
   
       // ------------------------- Hidden services ------------------------------
     protected:
   
       Lemniscate2D();
   
     private:
   
       //  Lemniscate2D ( const Lemniscate2D & other );
   
       Lemniscate2D & operator= ( const Lemniscate2D & other );
   
       // ------------------------- Internals ------------------------------------
     private:
   
}; // end of class Lemniscate2D
   
   
     template <typename T>
    std::ostream&
    operator<< ( std::ostream & out, const Lemniscate2D<T> & object );
   
  // } // namespace DGtal
   
   
   // Includes inline functions.
   #include "Lemniscate2D.ih"
   
   //                                                                           //
   
   #endif // !defined Lemniscate2D_h
   
   #undef Lemniscate2D_RECURSES
  #endif // else defined(Lemniscate2D_RECURSES)
