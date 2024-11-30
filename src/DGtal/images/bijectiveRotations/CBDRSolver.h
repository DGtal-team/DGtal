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
* @file CBDRSolver.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CBDRSOLVER_RECURSES)
#error Recursive header files inclusion detected in CBDRSolver.h
#else // defined(CBDRSOLVER_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBDRSOLVER_RECURSES

#if !defined CBDRSOLVER_h
/** Prevents repeated inclusion of headers. */
#define CBDRSOLVER_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <DGtal/images/RigidTransformation2D.h>
#include "Policy.h"
#include "NBijectiveReflectionGenerator.h"
namespace DGtal{
    /**
     * Description of template struct 'CBDR Solver'
     * \brief CBDR solver, use a policy to choose the composition of digitized reflections that minimises an error
     * @tparam TSpace a 2 dimensional space.
     * @tparam TDomain a 2 dimensional domain.
     */
    template<typename TSpace,typename TDomain>
    class CBDRSolver_GAvec{
    public:
        typedef CBDR_naiverotation<TSpace,typename TSpace::RealPoint> BijectiveReflections;
        typedef functors::ForwardRigidTransformation2D<TSpace,typename TSpace::RealPoint,typename TSpace::RealPoint,functors::Identity> RealRotation;
        typedef std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> bijectiveReflect;
        typedef ErrorVectorField<TSpace,TDomain,typename TSpace::RealPoint> ErrorRealVectors;
        typedef Reflection<TSpace> DigitizedReflection;
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;


        CBDRSolver_GAvec(const size_t km, const double rotAngle, const typename TDomain::Point center):nBijectiveGenerator(km),my_angle(rotAngle),my_center(center){}

        /// find the composition of bijective reflections that minimize either
        /// Linf, L2 or Lcontinuity of error vector field given by the policy
        /// @return {vector that minimizes an error given by policy, the error}
        std::pair<std::vector<GAVector<TSpace>>,double> outputCompositionReflection(const TDomain& set2d,
                                                            typename bijectiveReflect::iterator& lowerAngle,
                                                            typename bijectiveReflect::iterator& upperAngle,
                                                            const Policy<TSpace,TDomain,BijectiveReflections>& policy){

            // first error
            auto it = lowerAngle;

            typename bijectiveReflect::iterator itMinError;

            std::vector<DigitizedReflection> bijectiveReflections;
            std::vector<GAVector<TSpace>> firstReflectionsIndex=(*lowerAngle).first;

            for(size_t i = 0 ;i<firstReflectionsIndex.size();++i){
                bijectiveReflections.push_back(DigitizedReflection(firstReflectionsIndex[i]));//firstReflectionsIndex.size()-1-
            }

            CBDR_naiverotation<TSpace> reflections(bijectiveReflections);

            // finally get the average error
            double minError = policy.evaluate(set2d, reflections,my_angle, my_center);  //outputError(firsterrors,policy); // should now be policy

            itMinError = lowerAngle;

            for(it = lowerAngle+1 ; it != upperAngle ; ++it ){
                std::vector<DigitizedReflection> currentbijectiveReflections;
                std::vector<GAVector<TSpace>> currentReflectionsIndex=(*it).first;
                for(size_t i = 0 ;i<currentReflectionsIndex.size();++i){
                    currentbijectiveReflections.push_back(DigitizedReflection(currentReflectionsIndex[i]));//nBijectiveGenerator.BijectiveVectors[currentReflectionsIndex[i]] // currentReflectionsIndex.size()-1-
                }
                CBDR_naiverotation<TSpace> currentreflections(currentbijectiveReflections);

                // // finally get the error
                double error =  policy.evaluate(set2d, currentreflections,my_angle, my_center);
                if(fabs(error)<fabs(minError)){
                    itMinError = it;
                    minError = error;
                }
            }
            return {(*itMinError).first,minError};
        }
        NBijectiveGenerator<TSpace,typename TSpace::RealPoint> nBijectiveGenerator;
        double my_angle;
        typename TDomain::Point my_center;
    };


    /**
     * Description of template struct 'CBDR Solver'
     * \brief Aim: CBDR solver, use a policy to choose the composition of digitized reflections that minimises an error
     * @tparam TSpace a 2 dimensional space.
     * @tparam TDomain a 2 dimensional domain.
     */
    template<typename TSpace,typename TDomain>
    struct CBDRSolver {
        typedef CBDR_naiverotation<TSpace> BijectiveReflections;
        typedef functors::ForwardRigidTransformation2D<TSpace,typename TSpace::RealPoint,typename TSpace::RealPoint,functors::Identity> RealRotation;
        typedef std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> bijectiveReflect;
        typedef ErrorVectorField<TSpace,TDomain,BijectiveReflections> ErrorRealVectors;
        typedef Reflection<TSpace> DigitizedReflection;
        typedef std::vector<std::vector<typename TSpace::RealPoint>> VectorField;
        typedef std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> BijectiveSearchTree;

        CBDRSolver(const double rotAngle, const typename TDomain::Point center,const int km, const int NSamples):my_angle(rotAngle),my_center(center),kmax(km),N(NSamples){}

        void setPolicy(const Policy<TSpace,TDomain,BijectiveReflections> customPolicy){my_policy(customPolicy);}

        std::vector<Reflection<TSpace>> solve(const TDomain& points,NBijectiveGenerator<TSpace>& nbijectiveVectors, std::vector<BijectiveSearchTree>& vecBijectiveSearchTree,const Policy<TSpace,TDomain,BijectiveReflections>& policy){
            /// assume the sorted table was already generated either from a file or through one of the function of NbictiveGenerator
            std::vector<GAVector<TSpace>> bestParam;
            double errorMin = points.myUpperBound[0];
            for(size_t nReflection = 0 ; nReflection < vecBijectiveSearchTree.size();++nReflection){
                typename std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>>::iterator lowerAngle;
                typename std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>>::iterator upperAngle;
                int numberOfCompositions=N;

                nbijectiveVectors.getKNearestBijectiveComposition(
                vecBijectiveSearchTree[nReflection],
                lowerAngle,
                upperAngle,
                numberOfCompositions,
                my_angle);


                CBDRSolver_GAvec<TSpace,TDomain> rotationSolver(kmax,my_angle,my_center);
                std::pair<std::vector<GAVector<TSpace>>,double> bestParam_Error =rotationSolver.outputCompositionReflection(points,lowerAngle,upperAngle,policy);
                if(bestParam_Error.second < errorMin) {
                    bestParam = bestParam_Error.first;
                    errorMin = bestParam_Error.second;
                }
            }
            std::vector<DigitizedReflection> bestGAVectors;
            for(GAVector<TSpace> indexB:bestParam){
                bestGAVectors.push_back(DigitizedReflection(indexB));
            }
            return bestGAVectors;
        }


    protected:
        int kmax;
        int N;/// number of sample rotation angle
        double my_angle;
        typename TDomain::Point my_center;

    };
}


#endif //CBDRSOLVER
#undef CBDRSOLVER_RECURSES
#endif // else defined(CBDRSOLVER_RECURSES)

