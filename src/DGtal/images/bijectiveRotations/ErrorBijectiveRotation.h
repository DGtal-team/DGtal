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
* @file ErrorBijectiveRotation.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(ErrorBijectiveRotation_RECURSES)
#error Recursive header files inclusion detected in ErrorBijectiveRotation.h
#else // defined(ErrorBijectiveRotation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ErrorBijectiveRotation_RECURSES

#if !defined ErrorBijectiveRotation_h
/** Prevents repeated inclusion of headers. */
#define ErrorBijectiveRotation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/images/RigidTransformation2D.h>
#include "PointUtils.h"

namespace DGtal{
    // return error as a pair { loo, l2 }
    template<typename TSpace, typename TBijectiveRotation,typename TEuclideanRotation, typename TImage>
    std::pair< double, double > loo_linf_Errors(const TImage img, const TBijectiveRotation& bij_rot, const TEuclideanRotation& euclid_rot) {
        std::pair<double,double> loo_l2_errors;

        typedef typename TImage::Domain TDomain;
        typedef functors::DomainRigidTransformation2D < typename TImage::Domain, TBijectiveRotation> MyDomainTransformer;
        typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;

        std::vector<double> errors( (img.domain().myUpperBound[0]+1)*(img.domain().myUpperBound[1]+1) );
        int k=0;

        /// compute both the bijective rotation and the euclidean rotation of the input image
        for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
            typename TSpace::Point bijrotated_pt= bij_rot(*it);
            typename TSpace::RealPoint euclideanrotated_pt= euclid_rot(*it);

            errors[k++] =  distance2<typename TSpace::Point,typename TSpace::RealPoint>( bijrotated_pt, euclideanrotated_pt );
        }

        double loo = 0.0;
        double l2  = 0.0;
        for ( const auto e : errors ) {
            l2  += e;
            loo  = std::max( loo, sqrt( e ) );
        }

        return std::make_pair( loo, sqrt( l2 / errors.size() ) );;
    }
}


#endif //ErrorBijectiveRotation
#undef ErrorBijectiveRotation_RECURSES
#endif // else defined(ErrorBijectiveRotation_RECURSES)
