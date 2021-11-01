#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/shapes/implicit/ImplicitBall.h"

#ifdef WITH_EIGEN
#include <Eigen/Dense>
#endif

namespace DGtal
{
  namespace functions
  {
    /**
     * Returns the smallest nD ball going through two points.
     *
     * @param aPoint one point
     * @param otherPoint other point
     *
     * @return the smallest ImplicitBall that goes
     * through the 2 points
     */
    template<typename TPoint>
    ImplicitBall<SpaceND<TPoint::dimension> > ballFrom2Points(const TPoint & aPoint,
                                                              const TPoint & otherPoint)
    {
      typename SpaceND<TPoint::dimension>::RealPoint center(aPoint);
      center += otherPoint;
      double radius = (aPoint - otherPoint).norm()/2;
      return ImplicitBall<SpaceND<TPoint::dimension > >(0.5*center, radius);
    }
    
    
    /**
     * Return the smallest enclosing ball from 4 points in 3D
     */
    template<typename TPoint>
    ImplicitBall<SpaceND<TPoint::dimension> >
    getBallFrom4Point3D(const TPoint & A, const TPoint & B, const TPoint & C, const TPoint & D)
    {
      BOOST_ASSERT_MSG(TPoint::dimension == 3, "Point dimension must be 3");
      Eigen::Matrix3f M;
      Eigen::Vector3f b;
      int b1, b2, b3;
      
      b1 = -(pow(B[0], 2) - pow(A[0], 2) + pow(B[1], 2) - pow(A[1], 2) + pow(B[2], 2) - pow(A[2], 2));
      b2 = -(pow(C[0], 2) - pow(A[0], 2) + pow(C[1], 2) - pow(A[1], 2) + pow(C[2], 2) - pow(A[2], 2));
      b3 = -(pow(D[0], 2) - pow(A[0], 2) + pow(D[1], 2) - pow(A[1], 2) + pow(D[2], 2) - pow(A[2], 2));
      b << b1, b2, b3;
      
      M << 2*(A[0] - B[0]) , 2*(A[1] - B[1]) , 2*(A[2] - B[2]) ,
      2*(A[0] - C[0]) , 2*(A[1] - C[1]) , 2*(A[2] - C[2]) ,
      2*(A[0] - D[0]) , 2*(A[1] - D[1]) , 2*(A[2] - D[2]);
      
      Eigen::Vector3f centerVector = M.colPivHouseholderQr().solve(b);
      typename SpaceND<TPoint::dimension>::RealPoint center(centerVector(0), centerVector(1), centerVector(2));
      float x, y, z;
      x = center[0];
      y = center[1];
      z = center[2];
      float radius = sqrt(pow(x-A[0], 2) + pow(y-A[1], 2) + pow(z-A[2], 2));
      ImplicitBall<SpaceND<TPoint::dimension> > ball(center, radius);
      return ball;
    }
    
    
    /**
     * @param p1 one point of dimension 2
     * @param p2 second point of dimension 2
     * @param p3 third point of dimension 2
     *
     * @return the smallest ImplicitBall of dimension 2 that goes
     * through the 3 points
     */
    template<typename TPoint>
    ImplicitBall<SpaceND<TPoint::dimension> > ballFrom3Points(const TPoint & p1,
                                                              const TPoint & p2,
                                                              const TPoint & p3)
    {
      BOOST_ASSERT(TPoint::dimension == 2 || TPoint::dimension == 3);
      
      if (TPoint::dimension == 3)
      {
        return getBallFrom4Point3D(p1, p2, p3, p3);
      }
      /**
       * Formule d'après : Centre et rayon d’un cerclepassant par trois points donnés(Phm 2006/02/05)
       * https://cral-perso.univ-lyon1.fr/labo/fc/Ateliers_archives/ateliers_2005-06/cercle_3pts.pdf
       */
      double x1, x2, x3, y1, y2, y3;
      // Cas d'un côté parallèle à l'axe des abscisses pour éviter la division par 0, ici [P1, P2]
      if (!(p1[1] - p2[1])) {
        x1 = p1[0];
        y1 = p1[1];
        x3 = p2[0];
        y3 = p2[1];
        x2 = p3[0];
        y2 = p3[1];
      } else if (!(p2[1] - p3[1])) { // Ici [P2, P3]
        x1 = p2[0];
        y1 = p2[1];
        x3 = p3[0];
        y3 = p3[1];
        x2 = p1[0];
        y2 = p1[1];
      } else {
        x1 = p1[0];
        y1 = p1[1];
        x2 = p2[0];
        y2 = p2[1];
        x3 = p3[0];
        y3 = p3[1];
      }
      //Fin de gestion des cas particuliers
      
      double X1 = pow(x1, 2);
      double X2 = pow(x2, 2);
      double X3 = pow(x3, 2);
      double Y1 = pow(y1, 2);
      double Y2 = pow(y2, 2);
      double Y3 = pow(y3, 2);
      
      double centerX = -((X3-X2+Y3-Y2)/(2*(y3-y2)) - (X2-X1+Y2-Y1)/(2*(y2-y1))) / ((x2-x1)/(y2-y1) - (x3-x2)/(y3-y2));
      double centerY = -((x2-x1)/(y2-y1)) * centerX + (X2-X1+Y2-Y1)/(2*(y2-y1));
      double radius = sqrt(pow(centerX - x1, 2) + pow(centerY - y1, 2));
      typename SpaceND<TPoint::dimension>::RealPoint center(centerX, centerY);
      return ImplicitBall<SpaceND<TPoint::dimension> >(center, radius);
    }
    
   
    
    /**
     * @param R a set (DigitalSet, SmallSet etc) of size 3 or less
     *
     * @return the smallest ball with R on its border
     */
    template<typename TSet>
    ImplicitBall<typename TSet::Space> trivialCircle(TSet & R) {
      BOOST_ASSERT_MSG(R.size() < 5, "Set size must be 4 or less to find the trivial ball");
      BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<TSet> ));
      typedef typename TSet::Space::RealPoint RealPoint;
      typedef ImplicitBall<typename TSet::Space> ImplicitBall;
      double x, y, r;
      
      switch (R.size()) {
        case 1 :
          return ImplicitBall(*(R.begin()), 0);
        case 2 :
          return ballFrom2Points(*(R.begin()), *(R.begin()+1));
        case 3 :
          return ballFrom3Points(*(R.begin()), *(R.begin()+1), *(R.begin()+2));
        case 4 :
          return getBallFrom4Point3D(*(R.begin()), *(R.begin()+1), *(R.begin()+2), *(R.begin()+3));
        default :
          return ImplicitBall(RealPoint(0.5, 0.5), 0);
      }
    }
    
    /**
     * @param P the points we want to put in the
     * smallest enclosing ball. The type is necessary to use
     * random access, to generalize later...
     * @param R the points that are on the border of
     * the ball, initially empty.
     *
     * @return the smallest enclosing ImplicitBall of Point dimension
     */
    template<typename TSet>
    ImplicitBall<typename TSet::Space>  smallestEuclideanEnclosingBall(std::vector<typename TSet::Point> & P, TSet & R)
    {
      //This methods has trivialCircle code only in 2d.
      //In higher dimensions, use ........
      typedef typename TSet::Point Point;
      typedef typename TSet::Space::RealPoint RealPoint;
      typedef ImplicitBall<typename TSet::Space> ImplicitBall;
      
      BOOST_STATIC_ASSERT_MSG( Point::dimension == 2, "Point dimension must be 2");
      BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<TSet> ));
      
      double x, y, r;
      if (P.empty() || R.size() == Point::dimension) {
        ImplicitBall ball(trivialCircle(R));
        return ball;
      }
      int random = std::rand()%P.size();
      Point p = P[random];
      P.erase(P.begin()+random);
      ImplicitBall D = smallestEuclideanEnclosingBall(P, R);
      RealPoint center(D.center());
      r = D.radius();
      if ( (p-center).squaredNorm() < r*r ) {
        P.insert(P.begin(), p);
        return D;
      }
      R.insert(p);
      ImplicitBall ball(smallestEuclideanEnclosingBall(P, R));
      R.erase(p);
      P.insert(P.begin(), p);
      return ball;
    }
  }
}
