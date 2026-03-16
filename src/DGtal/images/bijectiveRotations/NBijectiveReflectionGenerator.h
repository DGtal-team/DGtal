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
* @file NBijectiveReflectionGenerator.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(NBIJECTIVEREFLECTIONGENERATOR_RECURSES)
#error Recursive header files inclusion detected in NBijectiveReflectionGenerator.h
#else // defined(CBDRFASTSOLVER_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NBIJECTIVEREFLECTIONGENERATOR_RECURSES

#if !defined NBIJECTIVEREFLECTIONGENERATOR_h
/** Prevents repeated inclusion of headers. */
#define NBIJECTIVEREFLECTIONGENERATOR_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <sstream>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include "GAVector.h"
#include "DigitizedReflection.h"
#include "CBDR_naiverotation.h"

namespace DGtal{
    template<typename TSpace,typename TInputValue=typename TSpace::RealPoint>
    struct NBijectiveGenerator{

        explicit NBijectiveGenerator(const size_t km):kmax(km){
            for(int k=0; k<kmax; k++){
                int x_kpp_k = (k+1);
                int x_k_kpp = (k);
                int x_1_2kpp = (1);
                int x_2kpp_1 = (2*k+1);


                int y_kpp_k = (k);
                int y_k_kpp = (k+1);
                int y_1_2kpp = (2*k+1);
                int y_2kpp_1 = (1);


                int x_kpp_k_m = (k+1);
                int x_k_kpp_m = (k);
                int x_1_2kpp_m = (1);
                int x_2kpp_1_m = (2*k+1);


                int y_kpp_k_m = -(k);
                int y_k_kpp_m = -(k+1);
                int y_1_2kpp_m = -(2*k+1);
                int y_2kpp_1_m = -(1);

                BijectiveVectors.push_back(GAVector<TSpace>({x_kpp_k,y_kpp_k}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_k_kpp,y_k_kpp}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_1_2kpp,y_1_2kpp}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_2kpp_1,y_2kpp_1}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_kpp_k_m,y_kpp_k_m}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_k_kpp_m,y_k_kpp_m}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_1_2kpp_m,y_1_2kpp_m}));
                BijectiveVectors.push_back(GAVector<TSpace>({x_2kpp_1_m,y_2kpp_1_m}));
            }
        }



        /// @brief  compose bijective digitized reflections
        /// @param Avector the composition of bijective reflection
        /// @return vector of indices of bijective digitized reflections
        std::vector<std::pair<std::vector<int>,GAVector<TSpace>>> composeBijectiveReflections(
            std::vector<std::pair<std::vector<int>,GAVector<TSpace>>>&  Avector ){
            std::vector<std::pair<std::vector<int>,GAVector<TSpace>>> result;
            result.reserve(Avector.size()*BijectiveVectors.size());

            for(std::pair<std::vector<int>,GAVector<TSpace>> normalVectorsAndResult : Avector){
                std::pair<std::vector<int>,GAVector<TSpace>> resultatCourant = normalVectorsAndResult;
                for(size_t indexB = 0 ; indexB<BijectiveVectors.size();++indexB){
                    GAVector<TSpace> outVec = normalVectorsAndResult.second*BijectiveVectors[indexB];
                    if(outVec.my_gavec[0] <0){
                        // resultatCourant.first.insert(resultatCourant.first.begin(),indexB);
                        // resultatCourant.second = outVec*-1;
                    }else{
                        resultatCourant.first.push_back(indexB);
                        resultatCourant.second = outVec;
                        result.push_back(resultatCourant);
                        resultatCourant=normalVectorsAndResult;
                    }

                }
            }
            // unique
            std::sort(result.begin(), result.end(), [](const std::pair<std::vector<int>,GAVector<TSpace>>& b1, const std::pair<std::vector<int>,GAVector<TSpace>>& b2) {
                    return b1.second < b2.second;});


            return result;
        }

        /// @brief  predicate to compare two composition of bijective reflections
        /// @param b1 first pair of composition of bijective reflections
        /// @param b2 second pair of composition of bijective reflections
        /// @return composition leads to same error?
        bool sameBijectiveComposition(const std::pair<std::vector<int>,GAVector<TSpace>>& b1, const std::pair<std::vector<int>,GAVector<TSpace>>& b2) const{
            // both reflections thanks to reflection composer
            if(fabs(b1.second.angleToXAxis()-b1.second.angleToXAxis())>1e-4){
                return false;
            }
            std::vector<Reflection<TSpace,TInputValue>> normals1 = getGAVectorFromIndex(b1.first);
            std::vector<Reflection<TSpace,TInputValue>> normals2 = getGAVectorFromIndex(b2.first);
            CBDR_naiverotation<TSpace,TInputValue> reflectionsCompose1(normals1);
            CBDR_naiverotation<TSpace,TInputValue> reflectionsCompose2(normals2);

            typename TSpace::Point zeroPt(0,0);

            for(int i =0 ; i<100;++i){
                for(int j =0 ; j<100;++j){
                    double x=i-50;
                    double y=j-50;
                    typename TSpace::RealPoint pointCourant = {x,y};
                    typename TSpace::Point out1 = reflectionsCompose1(pointCourant);
                    typename TSpace::Point out2 = reflectionsCompose2(pointCourant);
                    if((out1-out2)!= zeroPt  ){
                        return false;
                    }
                }
            }
            return true;
        }


        std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> vecBijNormals_index_2_GAVector(const std::vector<std::pair<std::vector<int>,GAVector<TSpace>>>& vecBijNormalsIndex) {
            std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> vecBijNormalsIndex_GAVector;
            for(std::pair<std::vector<int>,GAVector<TSpace>> pairIndexGAVector : vecBijNormalsIndex) {
                std::vector<GAVector<TSpace>> resGavec;

                for(int index : pairIndexGAVector.first) {
                    resGavec.push_back(BijectiveVectors[index]);
                }
                vecBijNormalsIndex_GAVector.push_back({resGavec,pairIndexGAVector.second});
            }
            return vecBijNormalsIndex_GAVector;

        }

        /// @brief generate all n =2,4 reflection composition
        /// @param n number of reflection to compose
        std::vector<std::pair<std::vector<int>,GAVector<TSpace>>> n_bijectiveReflections_get_NormalVectorsAngles(size_t n)
        {
            std::vector<std::pair<std::vector<int>,GAVector<TSpace>>> vecBijNormals;
            for(int i =0 ; i< BijectiveVectors.size() ; ++i){
                vecBijNormals.push_back({{i},BijectiveVectors[i]});
            }
            if(n>3) {
                --n;
            }

            for ( size_t j = 1; j < n; ++j )
            {
                vecBijNormals = composeBijectiveReflections( vecBijNormals);

                // remove duplicates with predicate defined below
                auto last = std::unique(vecBijNormals.begin(),vecBijNormals.end(),[this](const std::pair<std::vector<int>,GAVector<TSpace>>& b1, const std::pair<std::vector<int>,GAVector<TSpace>>& b2) {
                return sameBijectiveComposition(b1, b2);});
                vecBijNormals.erase(last,vecBijNormals.end());
            }
            if(n==3){
                /// interpret as the composition of 4 vectors : 3 bijective and 1 trivial bijective reflection
                for(auto& pairIndicesGAVec : vecBijNormals)
                    pairIndicesGAVec.first.insert(pairIndicesGAVec.first.begin(),0);
            }

            std::sort(vecBijNormals.begin(), vecBijNormals.end(), [](const std::pair<std::vector<int>,GAVector<TSpace>>& b1, const std::pair<std::vector<int>,GAVector<TSpace>>& b2) {
                      return b1.second.angleToXAxis() < b2.second.angleToXAxis();});


            return vecBijNormals;
        }

        void displayNormalVectorsAndAngles(const std::vector<std::pair<std::vector<int>,GAVector<TSpace>>>& vecBijNormals){
            for(size_t i =0;i<vecBijNormals.size();++i){
                auto currentIndicesNormalVector=vecBijNormals[i].first;
                GAVector<TSpace> finalNormalVec = vecBijNormals[i].second;

                if(2.0*finalNormalVec.angleToXAxis() < M_PI_4/8){
                std::cout << "m=[";
                for(int indexBijective : currentIndicesNormalVector){
                    std::cout <<"("<<BijectiveVectors[indexBijective].x<<","<<BijectiveVectors[indexBijective].y<<"),";
                }
                std::cout << "], angle="<<2.0*finalNormalVec.angleToXAxis()<<std::endl;
                }
            }
        }

        /// getter function used in predicate
        std::vector<Reflection<TSpace>> getGAVectorFromIndex(const std::vector<int>& indices) const{
            std::vector<Reflection<TSpace>> normals;
            for(auto i : indices){
                Reflection<TSpace> refi(BijectiveVectors[i]);
                normals.push_back(refi);
            }
            return normals;
        }

        void writeBijectiveVectors(const std::string& fileName, const std::vector<std::pair<std::vector<int>,GAVector<TSpace>>>& vecBijNormals){
            std::ofstream file(fileName);

            file << "kmax="<<kmax<<std::endl;

            for(size_t i = 0;i<vecBijNormals.size();++i){
                std::vector<int> gaNormalVecs = vecBijNormals[i].first;
                for(auto index :gaNormalVecs)
                    file << "("<< BijectiveVectors[index].my_gavec[0] << ","<<BijectiveVectors[index].my_gavec[1]<<"),";
                file  << "("<<vecBijNormals[i].second.my_gavec[0]<<"," <<vecBijNormals[i].second.my_gavec[1]<<")"<<std::endl;
            }
            file.close();
        }

        void getKNearestBijectiveComposition(
            std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>>& vecBijNormals,
            typename std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>>::iterator& lowerBound,
            typename std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>>::iterator& upperBound,
            const int K,
            const double targetAngle)
        {
            // find the index of the closest angle
            int i = std::lower_bound(vecBijNormals.begin(), vecBijNormals.end(), targetAngle,[](const std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>& b1, const double b2) {
                        return 2.0*b1.second.angleToXAxis() < b2;
                    }) - vecBijNormals.begin();

            int leftCompo=i-1;
            int rightCompo = i;

            // assume that K<vecBijNormals.size()
            int numberElements=0;
            while(numberElements<K){
                if (leftCompo < 0 || (rightCompo < vecBijNormals.size() &&
                    fabs(2*vecBijNormals[leftCompo].second.angleToXAxis() - targetAngle) > fabs(2*vecBijNormals[rightCompo].second.angleToXAxis() - targetAngle))) {
                    rightCompo++;
                }
                else {
                    leftCompo--;
                }
                numberElements++;
            }
            lowerBound=vecBijNormals.begin()+leftCompo;
            upperBound=vecBijNormals.begin()+rightCompo;

        }

    public:
        size_t kmax;
        std::vector<GAVector<TSpace>> BijectiveVectors;
    };

}



#endif //NBIJECTIVEREFLECTIONGENERATOR
#undef NBIJECTIVEREFLECTIONGENERATOR_RECURSES
#endif // else defined(NBIJECTIVEREFLECTIONGENERATOR_RECURSES)