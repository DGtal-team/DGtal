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
#include <DGtal/images/RigidTransformation2D.h>

namespace DGtal{
    template<typename TSpace, typename TDomain, typename TBijectiveReflections, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
    struct ErrorVectorField{

        typedef functors::ForwardRigidTransformation2D<TSpace,TInputValue,typename TSpace::RealPoint,functors::Identity> RealRotation;
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;

    protected:
        TBijectiveReflections normalVectors;
        typename TSpace::Point my_center;
        RealRotation targetRotation;
        RealRotation originCenteredRotation;

    public:
        ErrorVectorField(const TBijectiveReflections& reflections,const double theta,const typename TSpace::Point center):normalVectors(reflections),targetRotation(center,theta,{0,0}),my_center(center),originCenteredRotation({0,0},theta,{0,0})
        {}

        /// compute vector of errors for each pixel between either
        /// - each digitized reflections and the real reflection
        /// - or the the composition of digitized reflections and the final rotation
        VectorField getOutputVectorFieldFromContour(const TDomain& set2dContour, bool continuityVecField =false ){
            VectorField outVecField;

            for (typename TDomain::ConstIterator it = set2dContour.begin(); it != set2dContour.end(); ++it ) {
                std::vector<typename TSpace::RealPoint> pixelError;
                typename TSpace::Point p = *it;

                typename TSpace::Point preflections = normalVectors(p-my_center)+my_center;
                typename TSpace::RealPoint protation = targetRotation(p);

                // compute the error field
                typename TSpace::RealPoint error = protation - preflections ;
                pixelError.push_back(error);

                // compute the eucliean rotation of the neighbors of p
                if(continuityVecField) {
                    // for the 8-Neighbor, compute the rotation
                    for(int veci = -1 ; veci <2 ; ++veci) {
                        for(int vecj = -1 ; vecj<2 ; ++vecj) {
                            if(veci!=0 || vecj!=0){
                                typename TSpace::RealPoint vecij_rot = originCenteredRotation({static_cast<double>(veci),static_cast<double>(vecj)});
                                typename TSpace::RealPoint neigh_rot = protation+vecij_rot;
                                pixelError.push_back(neigh_rot-preflections);
                            }

                        }
                    }

                }


                outVecField.push_back(pixelError);
            }
            return outVecField;
        }
    };
}

#endif //ErrorBijectiveRotation
#undef ErrorBijectiveRotation_RECURSES
#endif // else defined(ErrorBijectiveRotation_RECURSES)
