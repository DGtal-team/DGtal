
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

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "RBC_vec.h"

namespace DGtal {
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
    struct RBC {
        const RBC_vec<TSpace,TInputValue,TOutputValue>& rot;
        typedef typename RBC_vec<TSpace,TInputValue,TOutputValue>::Circle Circle;

        /// Constructor from a RotationByCircles object.
        RBC( const RBC_vec<TSpace,TInputValue,TOutputValue>& aRot, const double angle, const TOutputValue center )
          : rot( aRot ),my_angle(angle),my_center(center) {}

        /// Rotates the whole image \a Image circle by circle.
        template<typename TImage>
        TImage rotateImage( const TImage& img) const
        {
            typedef typename TImage::Domain TDomain;
            typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::RBC_vec<TSpace>> MyDomainTransformer;
            typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;


            MyDomainTransformer domainTransformer ( rot );
            Bounds bounds = domainTransformer ( img.domain() );
            TDomain transformedDomain ( bounds.first, bounds.second );
            TImage rotatedImage ( transformedDomain );

            // for ( auto r = 1; r < rot.size(); r++ )
            //     rotateCircle( img, rotatedImage, my_center, my_angle, r );

            for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
                rotatedImage.setValue((*this).rot.operator()(*it),img(*it));
            }
            return rotatedImage;
        }

        TOutputValue operator()( const TInputValue & aInput ) const
        {
            return (*this).rot.operator()(aInput);
        }




        double my_angle;
        TOutputValue my_center;

    };


}


#endif //RBC

#undef RBC_RECURSES
#endif // else defined(RBC_RECURSES)