

#ifndef RBC_H
#define RBC_H
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "RBC_vec.h"

namespace DGtal {
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
typename TFunctor = DGtal::functors::VectorRounding < TInputValue, TOutputValue >>
    struct RBC {
        const RBC_vec<TSpace,TInputValue,TOutputValue,TFunctor>& rot;
        typedef typename RBC_vec<TSpace,TInputValue,TOutputValue,TFunctor>::Circle Circle;

        /// Constructor from a RotationByCircles object.
        RBC( const RBC_vec<TSpace,TInputValue,TOutputValue,TFunctor>& aRot, const double angle, const TOutputValue center )
          : rot( aRot ),my_angle(angle),my_center(center) {}

        /// Rotates the whole image \a Image circle by circle.
        template<typename TImage>
        TImage rotateImage( const TImage& img) const
        {
            typedef typename TImage::Domain TDomain;
            typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::RBC_vec<TSpace>> MyDomainTransformer;
            typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;


            MyDomainTransformer domainTransformer ( rot );
            Bounds bounds = domainTransformer ( img.domain() );
            TDomain transformedDomain ( bounds.first, bounds.second );
            TImage rotatedImage ( transformedDomain );

            // for ( auto r = 1; r < rot.size(); r++ )
            //     rotateCircle( img, rotatedImage, my_center, my_angle, r );

            for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
                rotatedImage.setValue((*this).rot.operator()(*it),img(*it));
            }
            return rotatedImage;
        }

        TOutputValue operator()( const TInputValue & aInput ) const
        {
            return (*this).rot.operator()(aInput);
        }


        /// Rotates the \a r-th circle around center in the image \a Image.
        template<typename TImage>
        TImage rotateCircle( TImage img,TImage& rotatedImage,
             TOutputValue center,
             double angle,
             int r ) const
        {
            if ( r >= rot.size() ) return rotatedImage;



            return rotatedImage;

            // const Circle& C = rot.circle( r ); // \todo change above algorithm to this one
            // std::vector< unsigned char > save( C.size() );
            // for ( int i = 0; i < C.size(); i++ )
            //     save[ i ] = getValue( Image, center + C[ i ] );
            // int shift = int( round( -angle * C.size() / ( 2.0 * M_PI ) ) );
            // for ( int i = 0; i < C.size(); i++ ) {
            //     int j = ( C.size() + i + shift ) % C.size();
            //     setValue( Image, center + C[ j ], save[ i ] );
            // }
        }

        double my_angle;
        TOutputValue my_center;

    };


}



#endif //RBC_H
