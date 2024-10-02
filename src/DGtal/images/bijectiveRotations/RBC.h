
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
* @file RBC.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(RBC_RECURSES)
#error Recursive header files inclusion detected in RBC.h
#else // defined(RBC_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RBC_RECURSES

#if !defined RBC_h
/** Prevents repeated inclusion of headers. */
#define RBC_h

#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "RBC_vec.h"

namespace DGtal {

    /**
     * Description of template struct RBC
     * \brief RBC : Bijective Rotation through Circles
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
    */
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
    struct RBC {
        RBC_vec<TSpace,TInputValue,TOutputValue> rot;
        typedef typename RBC_vec<TSpace,TInputValue,TOutputValue>::Circle Circle;

        /**
       * RBC Constructor.
       * @param angle  the angle given in radians.
       * @param center  the center of rotation.
       * @param aRot RBC Circles initialiser
       */
        RBC( const RBC_vec<TSpace,TInputValue,TOutputValue>& aRot, const double angle, const TOutputValue center )
          : rot( aRot ),my_angle(angle),my_center(center) {}

        /// Rotates the whole image \a Image circle by circle.
        template<typename TImage>
        TImage rotateImage( const TImage& img) const
        {
            typedef typename TImage::Domain TDomain;
            typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::RBC_vec<TSpace>> MyDomainTransformer;
            typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;

            typename TSpace::Point bottomLeft(-1,-1);
            typename TSpace::Point topRight(1,1);

            MyDomainTransformer domainTransformer ( rot );
            Bounds bounds = domainTransformer ( img.domain() );
            TDomain transformedDomain ( bounds.first+bottomLeft, bounds.second+topRight );
            TImage rotatedImage ( transformedDomain );



            // for ( auto r = 1; r < rot.size(); r++ )
            //      rotateCircle( img, rotatedImage, my_center, my_angle, r );

            for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
                rotatedImage.setValue((*this).rot.operator()(*it),img(*it));
            }
            return rotatedImage;
        }

        TOutputValue operator()( const TInputValue & aInput ) const
        {
            return (*this).rot.operator()(aInput);
        }

        std::string tostring() const {
            return {"RBC"};
        }

        void set_angle(const double newAngle) {
            my_angle=newAngle;
            rot.setAngle()= newAngle;
        }

        /// @return the centre of rotation
        inline TOutputValue  center() const{return my_center;}


        double my_angle;
        TOutputValue my_center;

    };


}

#endif //RBC

#undef RBC_RECURSES
#endif // else defined(RBC_RECURSES)