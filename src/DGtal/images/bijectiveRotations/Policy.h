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
* @file Policy.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(POLICY_RECURSES)
#error Recursive header files inclusion detected in Policy.h
#else // defined(POLICY_RECURSES)
/** Prevents recursive inclusion of headers. */
#define POLICY_RECURSES

#if !defined POLICY_h
/** Prevents repeated inclusion of headers. */
#define POLICY_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions



// Base Policy class
#include <DGtal/images/RigidTransformation2D.h>
#include "ErrorVectorField.h"

namespace DGtal {
    /**
     * Description of template struct Policy
     * \brief Policy : ,
     * @tparam TSpace a 2 dimensional space.
     * @tparam BijectiveRotation either CDLR or CBDR struct
    */
    template<typename TSpace, typename TDomain, typename BijectiveRotation>
    struct Policy {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;

        virtual ~Policy() = default;
        virtual double evaluate(const TDomain& set2d, const BijectiveRotation& reflections,double my_angle, typename TSpace::Point my_center) const = 0;
    };


    // Linf Policy class
    template<typename TSpace, typename TDomain, typename TBijectiveRotation>
    struct LinfPolicy : public Policy<TSpace,TDomain,TBijectiveRotation> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef ErrorVectorField<TSpace,TDomain,TBijectiveRotation,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const TBijectiveRotation& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);

            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d);
            double outError = 0.;
            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                typename TSpace::RealPoint vecErrorRealRot = vecError[0];
                outError= std::max(outError,std::sqrt((vecErrorRealRot[0]*vecErrorRealRot[0]+vecErrorRealRot[1]*vecErrorRealRot[1])));
            }
            return outError;
        }
    };

    // Linf Policy class
    template<typename TSpace, typename TDomain, typename TBijectiveRotation>
    struct L2 : public Policy<TSpace,TDomain,TBijectiveRotation> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef ErrorVectorField<TSpace,TDomain,TBijectiveRotation,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const TBijectiveRotation& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);
            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d);
            double outError = 0.;
            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                typename TSpace::RealPoint vecErrorRealRot = vecError[0];
                outError+= (vecErrorRealRot[0]*vecErrorRealRot[0]+vecErrorRealRot[1]*vecErrorRealRot[1]);
            }
            return std::sqrt(outError);
        }
    };




    // Lcontinuity Policy struct
    template<typename TSpace, typename TDomain, typename TBijectiveRotation>
    struct LcontinuityPolicy : public Policy<TSpace,TDomain,TBijectiveRotation> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef ErrorVectorField<TSpace,TDomain,TBijectiveRotation,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const TBijectiveRotation& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);
            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d,true);
            double outError = 0.;

            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                for(int i = 1 ; i<vecError.size();++i ) {
                    //std::cout << "vec error i ="<<vecError[i]<<std::endl;
                    outError+=(1./8.)*(vecError[i][0]*vecError[i][0]+vecError[i][1]*vecError[i][1]);
                }
            }
            return std::sqrt(outError/(set2d.size()));

        }
    };

    // MixedPolicy struct combining Lcontinuity and Linf
    template<typename TSpace, typename TDomain, typename TBijectiveRotation>
    struct MixedPolicy : public Policy<TSpace,TDomain,TBijectiveRotation> {
    private:
        double my_lambda; // Weight for Linf
        double my_mu;  // Weight for Lcontinuity
    public:
        MixedPolicy(const double lambda, const double mu) : my_lambda(lambda), my_mu(mu) {}

        double evaluate(const TDomain& set2d, const TBijectiveRotation& reflections, double my_angle, typename TSpace::Point my_center) const override {
            LcontinuityPolicy<TSpace,TDomain,TBijectiveRotation> lcontinuity;
            LinfPolicy<TSpace,TDomain,TBijectiveRotation> linf;
            return my_lambda* linf.evaluate(set2d,reflections,my_angle,my_center) +  my_mu* lcontinuity.evaluate(set2d,reflections,my_angle,my_center);
        }
    };
}


#endif //POLICY
#undef POLICY_RECURSES
#endif // else defined(POLICY_RECURSES)

