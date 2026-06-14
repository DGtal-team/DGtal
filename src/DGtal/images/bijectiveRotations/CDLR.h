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
* @file CDLR.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(CDLR_RECURSES)
#error Recursive header files inclusion detected in CDLR.h
#else // defined(CDLR_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDLR_RECURSES

#if !defined CDLR_h
/** Prevents repeated inclusion of headers. */
#define CDLR_h

#include "DGtal/base/Common.h"
#include <DGtal/images/RigidTransformation2D.h>
#include "CDLR_naiverotation.h"
#include "DigitizedReflection.h"
#include "Policy.h"
namespace DGtal {

    /**
     * Description of template struct CDLR
     * \brief CDLR : Rotation with Discrete Line Reflections,
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
    */
    template < typename TSpace, typename TInputValue = typename TSpace::Point, typename TOutputValue = typename TSpace::Point>
    class CDLR {
    public:
        /**
       * CDLR Constructor.
       * @param ang  the angle given in radians.
       * @param ptCenter  the center of rotation.
       * @param policy either Linf, L2, Lcontinuity, see Policy
       */
        CDLR( double ang, TOutputValue ptCenter, std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CDLR_naiverotation<TSpace>>> policy  );

        /// @return the angle of rotation.
        inline double    angle() const{return my_angle;};
        /// @return set the angle of rotation and call the composition of reflections solver.
        inline void   set_angle(const double new_angle) {
            my_angle=new_angle;
            my_naive_rdlr_rotation.set_angle(new_angle);
            dslSolver(new_angle,my_center);
        }

        /// initialisation function to find the composition of Discrete Line Reflection that minimises the sum of
        /// linf and lcontinuity
        void dslSolver(double ang, TOutputValue ptCenter);



        /// @return the centre of rotation
        inline TOutputValue  center() const{return my_center;};
        /// @return a reference to the centre of rotation
        inline TOutputValue& center(){return my_center;};

        /// @param p a lattice point
        /// @return the rotation of the point \a p according to the current
        /// angle and center.
        TOutputValue rotate( const TInputValue& p ) const;
        TOutputValue operator()( const TInputValue& p ) const;

        template<typename Img>
        Img rotateImage(Img img) const;

        std::string tostring() const {
            return {"CDLR"};
        }

    protected:
        /// The angle of rotation
        double   my_angle;
        /// The center of rotation
        TOutputValue my_center;



        CDLR_naiverotation<TSpace,TInputValue,TOutputValue> my_naive_rdlr_rotation;
        std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CDLR_naiverotation<TSpace>>> my_policy;

    private:
        /// DSL specific variables, see Andres paper
        double a;
        double b;
        double my_minAngle;
    };

    template<typename TSpace, typename TInputValue, typename TOutputValue>
    CDLR<TSpace,TInputValue,TOutputValue>::CDLR( const double ang, const TOutputValue ptCenter, std::shared_ptr<Policy<TSpace,HyperRectDomain< TSpace>,CDLR_naiverotation<TSpace>>> policy ):my_angle(ang),my_center(ptCenter),my_naive_rdlr_rotation(ang,ptCenter,0.),my_policy(policy){
        dslSolver(ang,ptCenter);
    }

    template<typename TSpace, typename TInputValue, typename TOutputValue>
    void CDLR<TSpace, TInputValue, TOutputValue>::dslSolver(double ang, TOutputValue ptCenter) {

        a = std::sin(ang/2.0);
        b = std::cos(ang/2.0);

        TInputValue origin(0,0);
        DGtal::functors::ForwardRigidTransformation2D<TSpace,TInputValue,DGtal::SpaceND< 2, DGtal::int32_t >::RealPoint,DGtal::functors::Identity> euclideanRot(my_center,my_angle,origin);

        double errorMinAlpha = 200.*200;
        int N = 200;

        // create the domain
        TOutputValue A(0,0);
        TOutputValue B(1000,1000);
        HyperRectDomain<TSpace> my_domain(A,B);


        /// look for the pair of reflections that minimizes the Linf and Lcontinuity norm
        std::vector<double> mix_errors;
        std::vector<double> angles;
        for(int k=0;k<N;++k){
            double alphaCourant = -M_PI_2 + (k*M_PI_2)/N;
            angles.push_back(alphaCourant);

            // compute both reflections and get both the Linf and Lcontinuity errors
            std::vector<double> linf_per_image;
            my_naive_rdlr_rotation.set_startingAngle(alphaCourant);
            double error = my_policy->evaluate(my_domain,my_naive_rdlr_rotation,my_angle,my_center);
            mix_errors.push_back(error);
        }
        for(int idxImages = 0 ; idxImages < mix_errors.size() ; idxImages++) {
            if(mix_errors[idxImages] < errorMinAlpha) {
                errorMinAlpha = mix_errors[idxImages];
                my_minAngle = angles[idxImages];
            }
        }
    }


    template<typename TSpace, typename TInputValue, typename TOutputValue>
    TOutputValue CDLR<TSpace,TInputValue,TOutputValue>::rotate( const TInputValue& p ) const{
        return this->operator()(p);
    }







    template<typename TSpace, typename TInputValue, typename TOutputValue>
    TOutputValue CDLR<TSpace,TInputValue,TOutputValue>::operator()( const TInputValue& p ) const{
        return my_naive_rdlr_rotation.reflect(my_minAngle+(my_angle),my_center,my_naive_rdlr_rotation.reflect(my_minAngle,my_center,p-my_center))+my_center;
    }


    template<typename TSpace, typename TInputValue, typename TOutputValue>
    template<typename TImage>
    TImage CDLR<TSpace,TInputValue,TOutputValue>::rotateImage( const TImage img ) const{
        typedef typename TImage::Domain TDomain;
        typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::CDLR<TSpace>> MyDomainTransformer;
        typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;

        typename TSpace::Point bottomLeft(-2,-2);
        typename TSpace::Point topRight(2,2);
        MyDomainTransformer domainTransformer ( *this );
        Bounds bounds = domainTransformer ( img.domain() );
        TDomain transformedDomain ( bounds.first+bottomLeft, bounds.second+topRight );
        TImage rotatedImage ( transformedDomain );

        for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it )
        {
            rotatedImage.setValue((*this)(*it),img(*it));
        }
        return rotatedImage;
    }


}


#endif //CDLR
#undef CDLR_RECURSES
#endif // else defined(CDLR_RECURSES)
