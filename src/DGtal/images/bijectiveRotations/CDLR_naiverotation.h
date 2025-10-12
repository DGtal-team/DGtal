
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
* @file CDLR_naiverotation.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07
 *
 * This file is part of the DGtal library.
 */

#if defined(CDLR_NAIVEROTATION_RECURSES)
#error Recursive header files inclusion detected in RDSL.h
#else // defined(CDLR_NAIVEROTATION_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDLR_NAIVEROTATION_RECURSES

#if !defined CDLR_NAIVEROTATION_h
/** Prevents repeated inclusion of headers. */
#define CDLR_NAIVEROTATION_h

#include "DGtal/base/Common.h"
#include <DGtal/images/RigidTransformation2D.h>
#include "PointUtils.h"
#include "DigitizedReflection.h"

namespace DGtal {


    template < typename TSpace, typename TInputValue = typename TSpace::Point, typename TOutputValue = typename TSpace::Point>
    struct CDLR_naiverotation {

        inline int X(int y, const double a, const double b, const int k) const {
            return std::ceil((2.0*k-1)/2.0 - (a/b)*(y)); // a = sin(theta) ; b=cos(theta)
        }

        TOutputValue reflect(const double angle, TOutputValue center, const TInputValue& p ) const{
            double a = std::sin(angle/2.0);
            double b = std::cos(angle/2.0);
            TOutputValue pcentered = p;

            int k = std::floor(pcentered[0] + (a/b)*pcentered[1] + 0.5);

            TOutputValue X1 = TOutputValue(X(std::ceil(a*b*k),a,b,k),std::ceil(a*b*k));
            TOutputValue X2 = TOutputValue(X(std::floor(a*b*k),a,b,k),std::floor(a*b*k));

            const double line2 = a*X1[0]-b*X1[1];
            if(line2<b/2 && line2 >(-b/2)){
                return TOutputValue(X(2*X1[1]-pcentered[1],a,b,k),2*X1[1]-pcentered[1]);
            }else{
                const double line3 = a*X2[0]-b*X2[1];
                if(line3<b/2 && line3 >(-b/2)){
                    return TOutputValue(X(2*X2[1]-pcentered[1],a,b,k),2*X2[1]-pcentered[1]);
                }
                else{
                    return TOutputValue(X(X1[1]+X2[1]-pcentered[1],a,b,k),X1[1]+X2[1]-pcentered[1]);
                }
            }


        }




        CDLR_naiverotation( double ang=0., TOutputValue ptCenter=TOutputValue(0,0), double starting_angle =0. ):my_angle(ang),my_center(ptCenter),my_startingAngle(starting_angle) {
        }

        /// @return the angle of rotation.
        inline double    angle() const{return my_angle;};

        /// @return the starting angle.
        inline double    startingAngle() const{return my_startingAngle;};

        ///set the angle of rotation and call the composition of reflections solver.
        inline void   set_angle(const double new_angle){my_angle=new_angle;}

        /// set the starting angle not the rotation angle
        inline void  set_startingAngle(const double new_startingAngle){my_startingAngle=new_startingAngle;}




        /// @return the centre of rotation
        inline TOutputValue  center() const{return my_center;};
        /// @return a reference to the centre of rotation
        inline TOutputValue& center(){return my_center;};

        /// @param p a lattice point
        /// @return the rotation of the point \a p according to the current
        /// angle and center.
        TOutputValue rotate( const TInputValue& p ) const {
            return reflect(my_startingAngle + my_angle, my_center,
                                                              reflect(
                                                                  my_startingAngle, my_center, p));
        }
        TOutputValue operator()( const TInputValue& p ) const {
            return rotate(p);
        }




    protected:
        /// The angle of rotation.
        double   my_angle;
        double my_startingAngle;

        /// The center of rotation.
        TOutputValue my_center;


    };

}



#endif //CDLR_NAIVEROTATION
#undef CDLR_NAIVEROTATION_RECURSES
#endif // else defined(CDLR_NAIVEROTATION_RECURSES)

