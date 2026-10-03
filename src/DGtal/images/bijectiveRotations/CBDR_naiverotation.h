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
* @file CBDR_naiverotation.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CBDRNAIVEROTATION_RECURSES)
#error Recursive header files inclusion detected in CBDR_naiverotation.h
#else // defined(CBDRNAIVEROTATION_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBDRNAIVEROTATION_RECURSES

#if !defined CBDRNAIVEROTATION_h
/** Prevents repeated inclusion of headers. */
#define CBDRNAIVEROTATION_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DigitizedReflection.h"
#include "GAVector.h"
namespace DGtal {
    /// vec since the parameters are the vectors of digitized reflections
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
struct CBDR_naiverotation{
        typedef Reflection<TSpace,TInputValue> DigitizedReflection;

        std::vector<DigitizedReflection> bijectiveNormalVectors;

        CBDR_naiverotation(const std::vector<DigitizedReflection>& bijectiveReflections={} ):bijectiveNormalVectors(bijectiveReflections){    }

        explicit CBDR_naiverotation(const std::vector<GAVector<TSpace>>& bijectiveGAvec ){
             bijectiveNormalVectors.resize(bijectiveGAvec.size());
             std::transform(bijectiveGAvec.begin(), bijectiveGAvec.end(), bijectiveNormalVectors.begin(),
                [](const GAVector<TSpace>& p) { return DigitizedReflection(p); });
        }

        /**
        * Operator
        * @return apply the composition of bijective digitized reflection to the point.
        */
        TOutputValue operator()( const TInputValue & aInput ) const
        {
            if(bijectiveNormalVectors.size()<=0){
                return (TOutputValue)aInput;
            }
            int i =0;
            DigitizedReflection firstReflection(bijectiveNormalVectors[i]);
            TOutputValue resultat = firstReflection(aInput);

            i+=1;
            for(; i < bijectiveNormalVectors.size(); ++i)
            {
                DigitizedReflection currentReflection(bijectiveNormalVectors[i]);
                resultat = currentReflection(resultat);
            }
            return resultat;
        }

        template<typename TImage>
        TImage rotateImage(TImage img) const {
            typedef typename TImage::Domain TDomain;
            typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::CBDR_naiverotation<TSpace>> MyDomainTransformer;
            typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;

            MyDomainTransformer domainTransformer ( *this );
            Bounds bounds = domainTransformer ( img.domain() );
            TDomain transformedDomain ( bounds.first, bounds.second );
            TImage rotatedImage ( transformedDomain );

            for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it )
            {
                rotatedImage.setValue((*this)(*it),img(*it));
            }
            return rotatedImage;
        }

    };
}


#endif //CBDRNAIVEROTATION
#undef CBDRNAIVEROTATION_RECURSES
#endif // else defined(CBDRNAIVEROTATION_RECURSES)
