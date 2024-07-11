
#ifndef QSH_H
#define QSH_H
#include "DGtal/base/Common.h"
#include <DGtal/io/readers/GenericReader.h>
#include <DGtal/images/RigidTransformation2D.h>

namespace DGtal {
//////////////////////////////////////////////////////////////////////////////
/// Represent a bijective rotation shears .
/// Provides a `rotate` method (if `angle`and `center` are set.
/// typedef not outside of class



template < typename TSpace, typename TInputValue = typename TSpace::Point, typename TOutputValue = typename TSpace::Point>
struct RotationShears {
        ///Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
    BOOST_STATIC_ASSERT(( TOutputValue::dimension == 2 ));
    BOOST_STATIC_ASSERT(( TInputValue::dimension == 2 ));




    inline TOutputValue hqs(const TOutputValue X, const double a, const double b, const double c) const{
        return TOutputValue{X[0] + static_cast<int>(round((a*X[1]+c)/b)),X[1]};
    }
    inline TOutputValue vqs(const TOutputValue X, const double a, const double b, const double c) const{
        return TOutputValue{X[0], X[1] + static_cast<int>((round((a*X[0]+c)/b)))};
    }

    inline TOutputValue hqs_p(const TOutputValue X, const double ap, const double bp, const double omega) const{
        return hqs(X, -ap, bp, 0.0);
    }
    inline TOutputValue vqs_p(const TOutputValue X, const double a, const double bp, const double omega) const{
        return vqs(X,a, omega, round(omega/2.0));
    }

    template<typename Img>
    Img rotateImage(Img img) const;

    RotationShears(double ang, TOutputValue ptCenter  );


    /// @return the angle of rotation.
    inline double    angle() const{return my_angle;};



    /// @return the centre of rotation
    inline TOutputValue  center() const{return my_center;};
    /// @return a reference to the centre of rotation
    inline TOutputValue& center(){return my_center;};

/// table à charger dans une autre classe :

  /// @param p a lattice point
  /// @return the rotation of the point \a p according to the current
  /// angle and center.
    TOutputValue rotate( const TInputValue& p ) const;
    TOutputValue operator()( const TInputValue& p ) const;



  /// SBJL rotate image with the image as a template parameter

protected:
    /// The angle of rotation.
    double   my_angle;
    /// The center of rotation.
    TOutputValue my_center;


private:
    /// shears variables
    const double omega=1<<15;
    double a;
    double aprime;
    double bprime;


};

template<typename TSpace, typename TInputValue, typename TOutputValue>
RotationShears<TSpace,TInputValue,TOutputValue>::RotationShears( const double ang, const TOutputValue ptCenter  ):my_angle(ang),my_center(ptCenter){
    a = round(omega*std::sin(my_angle));
    aprime = round(omega*std::sin(my_angle/2.0));
    bprime = round(omega*std::cos(my_angle/2.0));
}



template<typename TSpace, typename TInputValue, typename TOutputValue>
TOutputValue RotationShears<TSpace,TInputValue,TOutputValue>::rotate( const TInputValue& p ) const{
    return this->operator()(p);
}

template<typename TSpace, typename TInputValue, typename TOutputValue>
TOutputValue RotationShears<TSpace,TInputValue,TOutputValue>::operator()( const TInputValue& p ) const{

    TOutputValue pcentered = p-my_center;

    TOutputValue y1 = hqs_p(pcentered, aprime, bprime, omega);
    TOutputValue y2 = vqs_p(y1, a, bprime, omega);
    TOutputValue y = hqs_p(y2, aprime, bprime, omega);
    return y+my_center;
}



template<typename TSpace, typename TInputValue, typename TOutputValue>
template<typename TImage>
TImage RotationShears<TSpace,TInputValue,TOutputValue>::rotateImage( const TImage img ) const{
    typedef typename TImage::Domain TDomain;
    typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::RotationShears<TSpace>> MyDomainTransformer;
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



}
#endif //QSH_H
