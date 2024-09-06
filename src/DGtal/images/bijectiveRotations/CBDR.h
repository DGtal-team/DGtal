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
* @file CBDR.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CBDR_RECURSES)
#error Recursive header files inclusion detected in CBDR.h
#else // defined(CBDR_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBDR_RECURSES

#if !defined CBDR_h
/** Prevents repeated inclusion of headers. */
#define CBDR_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "GAVector.h"
#include "CBDRSolver.h"
#include "CBDRFastSolver.h"

#include "Rotationtables.h"
#include "NBijectiveReflectionGenerator.h"


namespace DGtal {
    /**
         * Description of template struct CBDR
         * \brief CBDR : Composition of Bijective Digitized Reflections,
         * @tparam TSpace a 2 dimensional space.
         * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
         * @tparam TOutputValue type of the output point e.g., TSpace::Point
         */
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
struct CBDR {
        ///Checking concepts
        BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
        BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
        BOOST_STATIC_ASSERT(( TOutputValue::dimension == 2 ));
        BOOST_STATIC_ASSERT(( TInputValue::dimension == 2 ));


        typedef Reflection<TSpace,TInputValue> DigitizedReflection;
        typedef std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> BijectiveSearchTree;


        /**
       * CBDR Constructor.
       * @param theta  the angle given in radians.
       * @param center  the center of rotation.
       * @param nbreflect the number of composition of bjijective reflections applied (2,4)
       * @param km conditions the number of bijective reflection normal vectors
       * @param policy either Linf, L2, Lcontinuity, see Policy
       * @param precompute use precomputed table of the sorted bijective composition of bijective digitized reflections
       * @param fast use the table that stores for each angle the composition that minimised the Linf metric distorsion
       */
        CBDR(const double theta,const typename TSpace::Point center,const size_t nbreflect,const size_t km, std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CBDR_naiverotation<TSpace>>> policy,
            const bool precompute=true, const bool fast =true ):nbijectiveGen(km),my_domain(typename TSpace::Point(0,0),typename TSpace::Point(100,100)),my_angle(theta),my_center(center),nbReflections(nbreflect),kmax(km),my_policy(policy),
                        usePrecomputedTable(precompute),useFastTable(fast),N(100),my_cbdr(initCBDRVec()) {
        }




        // whenever fast is set, load the table that stores for each angle the optimised set of bijective reflections
        std::shared_ptr<CBDR_naiverotation<TSpace>> initCBDRVec() {
            if(useFastTable) {
                std::cout << "use fast table"<<std::endl;
                std::string fastTableName=("CBDROptimisedTable_"+std::to_string(nbReflections)+"_"+std::to_string(kmax)+".txt");
                struct stat buffer;
                fastTableFound = (stat (fastTableName.c_str(), &buffer) == 0);
                if(fastTableFound) {
                        fastCBDRTable=loadFastOptimisedTable(fastTableName);
                }else {
                        initcbdr_loadBijectiveReflectionSearchTree(my_domain);
                        fastCBDRTable=initFastPrecomputationTable(my_domain,nbijectiveGen,vecBijectiveSearchTree);
                }
                /// solver part
                CBDRFastSolver<TSpace,HyperRectDomain<TSpace>> cbdrFastSolver(fastCBDRTable,my_angle,my_center,kmax);
                return std::make_shared<CBDR_naiverotation<TSpace>>(cbdrFastSolver.solve());
            }else{
                initcbdr_loadBijectiveReflectionSearchTree(my_domain);
                /// solver part
                CBDRSolver<TSpace,HyperRectDomain<TSpace>>  cbdrSolver(my_angle, my_center,kmax,N);
                return std::make_shared<CBDR_naiverotation<TSpace>>(cbdrSolver.solve(my_domain,nbijectiveGen, vecBijectiveSearchTree,*my_policy));
            }
        }

        void set_angle(const double newAngle) {
            my_angle = newAngle;
            if(fastTableFound) {
                // does not use the policy since the table is already stored.
                CBDRFastSolver<TSpace,HyperRectDomain<TSpace>> cbdrFastSolver(fastCBDRTable,my_angle,my_center,kmax);
                my_cbdr=std::make_shared<CBDR_naiverotation<TSpace>>(cbdrFastSolver.solve());
            }else {
                CBDRSolver<TSpace,HyperRectDomain<TSpace>>  cbdrSolver(my_angle, my_center,kmax,N);
                my_cbdr=std::make_shared<CBDR_naiverotation<TSpace>>(cbdrSolver.solve(my_domain,nbijectiveGen, vecBijectiveSearchTree,*my_policy));
            }
        }

        void setPolicy(const std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CBDR_naiverotation<TSpace>>>& newPolicy) {
            // need to search again for the precomputed tables
            my_policy = newPolicy;
            my_cbdr = initCBDRVec();
        }


        TOutputValue operator()( const TInputValue & aInput ) const
        {
            return my_cbdr->operator()(aInput-my_center)+my_center;
        }

        template<typename TImage>
        TImage rotateImage(TImage img) const {
            return my_cbdr->rotateImage(img);
        }

        std::string tostring() const {
            return {"CBDR"};
        }

        /// @return the centre of rotation
        inline TOutputValue  center() const{return my_center;}

    public:
        NBijectiveGenerator<TSpace,TInputValue> nbijectiveGen;
        std::vector<BijectiveSearchTree> vecBijectiveSearchTree;
        HyperRectDomain<TSpace> my_domain;

        size_t kmax;
        size_t nbReflections;
        bool usePrecomputedTable;
        bool useFastTable;
        int N;/// number of sample rotation angle
        double my_angle;
        TOutputValue my_center;
        std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CBDR_naiverotation<TSpace>>> my_policy;
        std::vector<CBDR_naiverotation<TSpace,TInputValue>> fastCBDRTable;
        std::shared_ptr<CBDR_naiverotation<TSpace>> my_cbdr;


    private:
        bool fastTableFound;

        /// load fast optimised table in case the table is found
        std::vector<CBDR_naiverotation<TSpace, TInputValue>> loadFastOptimisedTable(const std::string& fastTableName);
        void initcbdr_loadBijectiveReflectionSearchTree(const HyperRectDomain<TSpace>& my_domain);
        std::vector<CBDR_naiverotation<TSpace, TInputValue>> initFastPrecomputationTable(
                                         const HyperRectDomain<TSpace>& points,
                                         NBijectiveGenerator<TSpace>& nbijectiveVectors,
                                         std::vector<BijectiveSearchTree>& vecBijectiveSearchTree);
    };


    template<typename TSpace, typename TInputValue, typename TOutputValue>
    std::vector<CBDR_naiverotation<TSpace, TInputValue>> CBDR<TSpace, TInputValue, TOutputValue>::loadFastOptimisedTable(const std::string &fastTableName) {
        auto vecTable = DGtal::functions::loadFastCBDRTable<DGtal::SpaceND< 2, DGtal::int32_t >>(fastTableName);
        std::vector<CBDR_naiverotation<TSpace, TInputValue>> fastCBDRtab(vecTable.size());
        std::transform(vecTable.begin(),vecTable.end(), fastCBDRtab.begin(), [](std::tuple<std::vector<DGtal::GAVector<DGtal::SpaceND< 2, DGtal::int32_t >>>,double,double>& x) {
             return DGtal::CBDR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>(std::get<0>(x));
        });
        return fastCBDRtab;
    }

    template<typename TSpace, typename TInputValue, typename TOutputValue>
    void CBDR<TSpace, TInputValue, TOutputValue>::initcbdr_loadBijectiveReflectionSearchTree(const HyperRectDomain<TSpace>& my_domain) {

        for(size_t nbReflec = 2 ; nbReflec <= nbReflections ; nbReflec+=2) {
            BijectiveSearchTree vecBijNormals;
            std::string tableName("CBDRTable_"+std::to_string(nbReflec)+"_"+std::to_string(kmax)+".txt");
            // check that the file exists when usePrecomputedTable is set to true
            if(usePrecomputedTable ) {
                struct stat buffer;
                bool exists = (stat (tableName.c_str(), &buffer) == 0);
                if(!exists) {
                    usePrecomputedTable=false;
                }
            }

            if(usePrecomputedTable){
                std::cout << "loading precomputed table ..."<<std::endl;
                vecBijNormals = functions::loadBijectiveRotationTable<TSpace,TInputValue>(tableName,nbReflec,kmax);
            } else {
                std::cout << "does not use precomputed table"<<std::endl;
                auto tableIntVecBijNormals = nbijectiveGen.n_bijectiveReflections_get_NormalVectorsAngles(nbReflec);
                nbijectiveGen.writeBijectiveVectors("CBDRTable_"+std::to_string(nbReflec)+"_"+std::to_string(kmax)+".txt",tableIntVecBijNormals );
                vecBijNormals = nbijectiveGen.vecBijNormals_index_2_GAVector(tableIntVecBijNormals);
            }
            vecBijectiveSearchTree.push_back(vecBijNormals);
        }

    }

    template<typename TSpace, typename TInputValue, typename TOutputValue>
    std::vector<CBDR_naiverotation<TSpace, TInputValue>> CBDR<TSpace, TInputValue, TOutputValue>::
    initFastPrecomputationTable(const HyperRectDomain<TSpace> &points, NBijectiveGenerator<TSpace> &nbijectiveVectors,
        std::vector<BijectiveSearchTree> &vecBijectiveSearchTree) {
        std::vector<CBDR_naiverotation<TSpace, TInputValue>> fastCBDRtab;
        auto Linf = std::make_shared<DGtal::LinfPolicy<TSpace,DGtal::HyperRectDomain<TSpace>,DGtal::CBDR_naiverotation<TSpace>>>();
        auto LContinuity = std::make_shared<DGtal::LcontinuityPolicy<TSpace,DGtal::HyperRectDomain<TSpace>,DGtal::CBDR_naiverotation<TSpace>>>();

        std::string tablefilename ="CBDROptimisedTable_"+std::to_string(nbReflections)+"_"+std::to_string(kmax)+".txt";
        std::ofstream file(tablefilename);
        for(int alpha = 0 ; alpha < 91; ++alpha) {
            double currentAngle = (alpha * M_PI) / 180.0;
            CBDRSolver<TSpace,HyperRectDomain<TSpace>>  cbdrsolv(currentAngle, my_center,kmax,N);
            std::vector<Reflection<TSpace>> bestReflections =cbdrsolv.solve(points,nbijectiveVectors,
                vecBijectiveSearchTree,*my_policy);

            CBDR_naiverotation<TSpace> currentRotation(bestReflections);
            fastCBDRtab.push_back(currentRotation);

            // display vector of reflections
            for(size_t i = 0 ; i<bestReflections.size() ; ++i) {
                if(i<bestReflections.size()-1)
                    file << bestReflections[i].normalVector.my_gavec << ",";
                else
                    file << bestReflections[i].normalVector.my_gavec;
            }
            file << ";"<<Linf->evaluate(points,currentRotation,currentAngle,my_center);
            file << ";"<<LContinuity->evaluate(points,currentRotation,currentAngle,my_center);
            file << std::endl;

        }
        file.close();
        std::cout << "table written !"<<std::endl;
        fastTableFound=true;
        return fastCBDRtab;
    }


}




#endif //CBDR
#undef CBDR_RECURSES
#endif // else defined(CBDR_RECURSES)
