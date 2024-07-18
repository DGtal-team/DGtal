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
* @file RDSL.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(RDSL_RECURSES)
#error Recursive header files inclusion detected in RDSL.h
#else // defined(RDSL_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RDSL_RECURSES

#if !defined RDSL_h
/** Prevents repeated inclusion of headers. */
#define RDSL_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions

#include "DGtal/base/Common.h"
#include <DGtal/io/readers/GenericReader.h>
#include <DGtal/images/RigidTransformation2D.h>
namespace DGtal {
    template < typename TSpace, typename TInputValue = typename TSpace::Point, typename TOutputValue = typename TSpace::Point>
    struct RotationDSL {

        /// \brief DSL reflection function, see Andres et al.
        inline int X(int y, const double a, const double b, const int k) const{
            return std::ceil((2.0*k-1)/2.0 - (a/b)*(y)); // a = sin(theta) ; b=cos(theta)
        }



        RotationDSL( double ang, TOutputValue ptCenter  );

        /// @return the angle of rotation.
        inline double    angle() const{return my_angle;};
        /// @return a reference to the angle of rotation.
        inline void   set_angle(const double new_angle){my_angle=new_angle; a = std::sin(my_angle);b = std::cos(my_angle);};
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

    private:
        TOutputValue reflect( const TInputValue& p ) const;


    protected:
        /// The angle of rotation.
        double   my_angle;
        /// The center of rotation.
        TOutputValue my_center;

    private:
        /// DSL specific variables, see Andres paper
        double a;
        double b;


    };

    template<typename TSpace, typename TInputValue, typename TOutputValue>
    RotationDSL<TSpace,TInputValue,TOutputValue>::RotationDSL( const double ang, const TOutputValue ptCenter  ):my_angle(ang),my_center(ptCenter){
        a = std::sin(my_angle/2.0);
        b = std::cos(my_angle/2.0);
    }



    template<typename TSpace, typename TInputValue, typename TOutputValue>
    TOutputValue RotationDSL<TSpace,TInputValue,TOutputValue>::rotate( const TInputValue& p ) const{
        return this->operator()(p);
    }


    template<typename TSpace, typename TInputValue, typename TOutputValue>
    TOutputValue RotationDSL<TSpace,TInputValue,TOutputValue>::reflect( const TInputValue& p ) const{
        TOutputValue pcentered = p-my_center;


        int k = std::floor(pcentered[0] + (a/b)*pcentered[1] + 0.5);

        TOutputValue X1 = TOutputValue(X(std::ceil(a*b*k),a,b,k),std::ceil(a*b*k));
        TOutputValue X2 = TOutputValue(X(std::floor(a*b*k),a,b,k),std::floor(a*b*k));

        const double line2 = a*X1[0]-b*X1[1];
        if(line2<b/2 && line2 >(-b/2)){
            return TOutputValue(X(2*X1[1]-pcentered[1],a,b,k),2*X1[1]-pcentered[1])+ my_center;
        }else{
            const double line3 = a*X2[0]-b*X2[1];
            if(line3<b/2 && line3 >(-b/2)){
                return TOutputValue(X(2*X2[1]-pcentered[1],a,b,k),2*X2[1]-pcentered[1])+ my_center;
            }
            else{
                return TOutputValue(X(X1[1]+X2[1]-pcentered[1],a,b,k),X1[1]+X2[1]-pcentered[1]) + my_center;
            }
        }


    }




    template<typename TSpace, typename TInputValue, typename TOutputValue>
    TOutputValue RotationDSL<TSpace,TInputValue,TOutputValue>::operator()( const TInputValue& p ) const{
        return this->reflect(RotationDSL(0.,my_center).reflect(p));
    }


    template<typename TSpace, typename TInputValue, typename TOutputValue>
    template<typename TImage>
    TImage RotationDSL<TSpace,TInputValue,TOutputValue>::rotateImage( const TImage img ) const{
        typedef typename TImage::Domain TDomain;
        typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::RotationDSL<TSpace>> MyDomainTransformer;
        typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;

        TInputValue origin(0,0);

        MyDomainTransformer domainTransformer ( *this );
        Bounds bounds = domainTransformer ( img.domain() );
        TDomain transformedDomain ( bounds.first, bounds.second );
        TImage rotatedImage ( transformedDomain );

        DGtal::functors::ForwardRigidTransformation2D<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::SpaceND< 2, DGtal::int32_t >::Point,DGtal::SpaceND< 2, DGtal::int32_t >::RealPoint,DGtal::functors::Identity> euclideanRot(my_center,my_angle,origin);

        int indiceAlpha = 0;
        double alphaMin = 0.0;
        double errorMinAlpha = 200.*200;
        int N = 100;

        /// look for the pair of reflections that minimizes the Linf norm
        for(int k=0;k<N;++k){
            double alphaCourant = -M_PI_4 + (k*M_PI_4)/N;

            // compute both reflections and get the Linf error
            double currentLinf=0.0;
            for(int i =0 ; i<201;++i){
                for(int j =0 ; j<201;++j){
                    int x=i-100;
                    int y=j-100;

                    DGtal::Z2i::Point xref1 = RotationDSL<TSpace>(alphaCourant,DGtal::Z2i::Point(i-100,j-100)).reflect(DGtal::Z2i::Point(x,y));
                    DGtal::Z2i::Point xrot2 = RotationDSL<TSpace>(alphaCourant+(my_angle),DGtal::Z2i::Point(i-100,j-100)).reflect(xref1);//reflect(xref1, alphaCourant+(my_angle));

                    DGtal::Z2i::RealPoint xrotReal = euclideanRot({i-100,j-100});

                    double pointError = std::max(fabs(xrotReal[0]-xrot2[0]),(fabs(xrotReal[1]-xrot2[1])));
                    if(pointError>currentLinf){
                        currentLinf=pointError;
                    }
                }
            }

            if(currentLinf<errorMinAlpha){
                errorMinAlpha=currentLinf;
                alphaMin = alphaCourant;
                indiceAlpha=k;
            }

        }



        for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it )
        {
            rotatedImage.setValue(RotationDSL<TSpace>(alphaMin+(my_angle),my_center).reflect(RotationDSL<TSpace>(alphaMin,my_center).reflect((*it))),img(*it));//(*this)(*it)
        }

        return rotatedImage;
    }
}


#endif //RDSL
#undef RDSL_RECURSES
#endif // else defined(RDSL_RECURSES)
