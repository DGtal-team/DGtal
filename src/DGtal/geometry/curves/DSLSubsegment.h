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
 * @file DSLSubsegment.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/07/17
 *
 *
 */

#if defined(DSLSubsegment_RECURSES)
#error Recursive header files inclusion detected in DSLSubsegment.h
#else // defined(DSLSubsegment_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DSLSubsegment_RECURSES

#if !defined DSLSubsegment_h
/** Prevents repeated inclusion of headers. */
#define DSLSubsegment_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/arithmetic/IntegerComputer.h"

#include <iomanip> 

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class DSLSubsegment
/**
 * Description of class 'DSLSubsegment' <p>
 * \brief Aim:
 */


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
  

  template <typename TInteger, typename TNumber>
    class DSLSubsegment
    {
      // ----------------------- Standard services ------------------------------
    public:
      
      /**
       * Destructor.
     */
      ~DSLSubsegment(){};
      
    // ----------------------- Interface --------------------------------------
    public:
      
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
      
      
      typedef TNumber Number;
      typedef TInteger Integer;
      typedef long double FloatType;
      typedef DGtal::PointVector<2,Integer> Ray;
      typedef DGtal::PointVector<2,Integer> Point;
      typedef DGtal::PointVector<2,Number> PointF;

      //typedef DGtal::PointVector<2,Integer> Vector;
      typedef DGtal::PointVector<2,Integer> Vector;
      typedef DGtal::PointVector<2,Number> VectorF;
      
      // Minimal characteristics of the subsegment AB of the DSL(a,b,mu)
      Integer aa;
      Integer bb;
      Integer Nu;
      
      
      /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
      DSLSubsegment();
      
      Number myPrecision;
      
      // Constructor
      // Given the parameters of a DSL 0 <= ax -by + mu < b, and two points A and B of this DSL,
      // compute the parameters of the DSS [AB].
      //DSLSubsegment(Integer a, Integer b, Integer mu, Point A, Point B);
      DSLSubsegment(Number a, Number b, Number mu, Point A, Point B, bool *locRay);

      // Constructor
      // Given a straight line of equation y = alpha x + beta, and two
      // points A and B of the OBQ digitization of this line, compute
      // the parameters of the DSS [AB]  
      DSLSubsegment(Number alpha, Number beta, Point A, Point B, Number precision = 1e-10);
      

      DSLSubsegment(Number a, Number b, Number mu, Point A, Point B, bool flag);

    protected:
      
      class RayC
      {
      public :
	Integer x;
	Integer y;
	
	/**
	 * Default constructor.
	 * not valid
	 */
	RayC();
	
	/**
	 * Constructor with initialisation
	 * @param it an iterator on 2D points
	 * @see init
	 */
	RayC(const Integer x0, const Integer y0);
	
	RayC(const Integer p, const Integer q, const Integer r, const Integer slope);
	
	~RayC();
	
      };
      
      typedef enum Position
      {
	ABOVE,
	BELOW,
	ONTO
      } Position;
      
      
      Integer min (Integer a, Integer b);
      
      
      // Compute the intersection between the line of direction v passing through P and the vertical line x = n.
      // The intersection point is of the form P + \alpha*v and the function returns the value floor(alpha).
      Integer intersectionVertical(Point P, Vector v, Integer n);
      
      
      // Compute the intersection between the line of direction v passing through P and the line y = (aL[1]/aL[0])*x
      // The intersection point is of the form P + \alpha*v and the function returns the value floor(alpha).
      //Integer intersection(Point P, Vector v, Vector aL);
      Integer intersection(Point P, Vector v, Vector aL, Integer r);
      Integer intersection(Point P, Vector v, Number s);
      
      
      //void update(Vector u, Point A, Vector l, Vector *v);
      void update(Vector u, Point A, Vector l, Integer r, Vector *v);
      void update(Vector u, Point A, Number s, Vector *v);
      
      void filterUpperCH(Vector l, Number mu, Point sup[4]);
      void lowerConvexHull(Vector l, Integer mu, Point A, Point B, Point *prevInfL, Point *infL, Point *infR, Point *prevInfR);
      

      //void convexHullApprox(Vector l, Integer n, Point *inf, Point *sup);
      void convexHullApprox(Vector l, Integer r, Integer n, Point *inf, Point *sup);
      void convexHullHarPeled(Vector l, Integer n, Point *inf, Point *sup);
      void convexHullApprox(Number s, Integer n, Point *inf, Point *sup);
      void convexHullApproxTwoPoints(Vector l, Integer r, Integer n, Point *inf, Point *sup, Point *prevInf, Point *prevSup, bool inv);
      
      
      Point nextTermInFareySeriesEuclid(Integer fp, Integer fq, Integer n);
      


      // Compute the ray of highest slope in O(1) knowing the ray of smallest
      // slope and the order of the Farey fan
      RayC rayOfHighestSlope(Integer p, Integer q, Integer r, Integer smallestSlope, Integer n);
      
      // Compute the ceil of the slope of the line through (f=p/q,r/q) and point (a/b,mu/b)- O(1)
      //Integer slope(Integer p, Integer q, Integer r, Integer a, Integer b, Integer mu);
      Number slope(Integer p, Integer q, Integer r, Number a, Number b, Number mu);
      Number slope(Integer p, Integer q, Integer r, Number alpha, Number beta);

      
      // Compute the position of point with respect to a ray
      // Return BELOW, ABOVE or ONTO
      //Position positionWrtRay(RayC r, Integer a, Integer b, Integer mu);
      Position positionWrtRay(RayC r, Number a, Number b, Number mu);
      
      Position positionWrtRay(RayC r, Number alpha, Number beta);
      
      // Computes the ray of smallest slope emanating from the point (f=p/q,
      // r/q) using the knowledge of the next fraction g in the Farey Series.
      // Complexity O(1) 
      RayC smartRayOfSmallestSlope(Integer fp, Integer fq, Integer gp, Integer gq, Integer r); 
      
      //Integer smartFirstDichotomy(Integer fp, Integer fq, Integer gp, Integer gq, Integer a, Integer b, Integer mu, Integer n, bool *flagRayFound);
      
      
      Integer smartFirstDichotomy(Integer fp, Integer fq, Integer gp, Integer gq, Number a, Number b, Number mu, Integer n, bool *flagRayFound);
      
      Integer smartFirstDichotomy(Integer fp, Integer fq, Integer gp, Integer gq, Number alpha, Number beta, Integer n, bool *flagRayFound);
      
      
      
      // Fraction f=p/q, r, slope x0 of the ray of smallest slope passing
      // through (p/q, r/q), a point P(alpha,beta) to localize -> closest ray
      // below P passing through (p/q,r/q)
      //RayC localizeRay(Integer fp, Integer fq, Integer gp, Integer gq, Integer r, Integer a, Integer b, Integer mu,  Integer n);

      RayC localizeRay(Integer fp, Integer fq, Integer gp, Integer gq, Integer r, Number a, Number b, Number mu,  Integer n);
      RayC localizeRay(Integer fp, Integer fq, Integer gp, Integer gq, Integer r, Number alpha, Number beta, Integer n);
      
      
      // Compute the ray emenating from (f=p/q,h/q) just above r
      // Complexity O(1)
      RayC raySup(Integer fp, Integer fq, RayC r);
      
  
      // The two fractions f and g together with the ray r define a segment
      // AB. AB is part of the lower boundary of exactly one cell of the
      // FareyFan. This cell represents a DSS. findSolution computes the
      // vertex of the cell that represents the minimal parameters of the DSS. 
      // Complexity of nextTermInFareySeriesEuclid
      void findSolutionWithoutFractions(Integer fp, Integer fq, Integer gp, Integer gq, RayC r, Integer n, Integer *resAlphaP, Integer *resAlphaQ, Integer *resBetaP, bool found);  // resBetaQ = resAlphaQ  
      
      // Corresponds to the algorithm of paper "Walking in the Farey Fan to compute the characteristics of  discrete straight line subsegment" (Isabelle Sivignon, DGCI 2013, Springer LNCS 7749)
      void shortFindSolution(Integer fp, Integer fq, Integer gp, Integer gq, RayC r, Integer n, Integer *resAlphaP, Integer *resAlphaQ, Integer *resBetaP, bool found);  // resBetaQ = resAlphaQ  
      

    };
  
   
}// namespace DGtal



  //Includes inline functions.
#include "DSLSubsegment.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DSLSubsegment_h

#undef DSLSubsegment_RECURSES
#endif // else defined(DSLSubsegment_RECURSES)
