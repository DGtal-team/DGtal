#pragma once
#if defined(Astroid2D_RECURSES)
#error Recursive header files inclusion detected in Astroid2D.h
#else // defined(Astroid2D_RECURSES)
 
#define Astroid2D_RECURSES

#if !defined Astroid2D_h

#define Astroid2D_h

// Inclusions
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/shapes/parametric/StarShaped2D.h>
#include <cmath>

//namespace DGtal
//{

 // template class Astroid2D
template <typename TSpace>
class Astroid2D : public DGtal::StarShaped2D<TSpace>
{
// ----------------------- Standard services ------------------------------
  public:

    typedef TSpace Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint2D;
    typedef typename Space::RealVector RealVector2D;
    ~Astroid2D();
      
   Astroid2D( const double a, const double b,
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
   
       Astroid2D();
   
     private:
   
       //  Astroid2D ( const Astroid2D & other );
   
       Astroid2D & operator= ( const Astroid2D & other );
   
       // ------------------------- Internals ------------------------------------
     private:
   
}; // end of class Astroid2D
   
   
     template <typename T>
    std::ostream&
    operator<< ( std::ostream & out, const Astroid2D<T> & object );
   
  // } // namespace DGtal
   
   
   // Includes inline functions.
   #include "Astroid2D.ih"
   
   //                                                                           //
   
   #endif // !defined Astroid2D_h
   
   #undef Astroid2D_RECURSES
  #endif // else defined(Astroid2D_RECURSES)
