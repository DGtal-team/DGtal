//
// Created by Stephane on 06/08/2024.
//

#ifndef POLICY_H
#define POLICY_H
// Base Policy class
#include <DGtal/images/RigidTransformation2D.h>
#include "CBDRErrorVectorField.h"

namespace DGtal {
    template<typename TSpace, typename TDomain>
    struct Policy {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;

        virtual ~Policy() = default;
        virtual double evaluate(const TDomain& set2d, const CBDR_vec<TSpace>& reflections,double my_angle, typename TSpace::Point my_center) const = 0;
    };


    // Linf Policy class
    template<typename TSpace, typename TDomain>
    struct LinfPolicy : public Policy<TSpace,TDomain> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef CBDRErrorVectorField<TSpace,TDomain,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const CBDR_vec<TSpace>& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);
            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d);
            double outError = 0.;
            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                typename TSpace::RealPoint vecErrorRealRot = vecError[0];
                outError= (outError<(vecErrorRealRot[0]*vecErrorRealRot[0]+vecErrorRealRot[1]*vecErrorRealRot[1]))?(vecErrorRealRot[0]*vecErrorRealRot[0]+vecErrorRealRot[1]*vecErrorRealRot[1]):outError;
            }
            return outError;
        }
    };

    // Linf Policy class
    template<typename TSpace, typename TDomain>
    struct L2 : public Policy<TSpace,TDomain> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef CBDRErrorVectorField<TSpace,TDomain,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const CBDR_vec<TSpace>& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);
            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d);
            double outError = 0.;
            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                typename TSpace::RealPoint vecErrorRealRot = vecError[0];
                outError+= (vecErrorRealRot[0]*vecErrorRealRot[0]+vecErrorRealRot[1]*vecErrorRealRot[1]);
            }
            return outError;
        }
    };




    // Lcontinuity Policy struct
    template<typename TSpace, typename TDomain>
    struct LcontinuityPolicy : public Policy<TSpace,TDomain> {
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef CBDRErrorVectorField<TSpace,TDomain,typename TSpace::RealPoint> ErrorRealVectors;

        double evaluate(const TDomain& set2d, const CBDR_vec<TSpace>& reflections,double my_angle, typename TSpace::Point my_center) const override {
            ErrorRealVectors errorsVectors(reflections,my_angle,my_center);
            VectorField errors = errorsVectors.getOutputVectorFieldFromContour(set2d,true);
            double outError = 0.;
            for(std::vector<typename TSpace::RealPoint> vecError: errors) {
                for(int i = 1 ; i<vecError.size();++i ) {
                    outError+=(1./8.)*(vecError[i][0]*vecError[i][0]+vecError[i][1]*vecError[i][1]);
                }
            }
            return outError;

        }
    };

    // MixedPolicy struct combining Lcontinuity and Linf
    template<typename TSpace, typename TDomain>
    struct MixedPolicy : public Policy<TSpace,TDomain> {
    private:
        double my_lambda; // Weight for Linf
        double my_mu;  // Weight for Lcontinuity
    public:
        MixedPolicy(double lambda, double mu) : my_lambda(lambda), my_mu(mu) {}

        double evaluate(const TDomain& set2d, const CBDR_vec<TSpace>& reflections,double my_angle, typename TSpace::Point my_center) const override {
            LcontinuityPolicy<TSpace,TDomain> lcontinuity;
            LinfPolicy<TSpace,TDomain> linf;
            return my_mu * linf.evaluate(reflections,my_angle,my_center) + my_lambda * lcontinuity.evaluate(reflections,my_angle,my_center);
        }
    };
}

#endif //POLICY_H
