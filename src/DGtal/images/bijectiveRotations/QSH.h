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
* @file QSH.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(QSH_RECURSES)
#error Recursive header files inclusion detected in QSH.h
#else // defined(QSH_RECURSES)
/** Prevents recursive inclusion of headers. */
#define QSH_RECURSES

#if !defined QSH_h
/** Prevents repeated inclusion of headers. */
#define QSH_h
#include <DGtal/base/Common.h>
#include <DGtal/images/RigidTransformation2D.h>

namespace DGtal {
/**
 * Description of template struct CDLR
 * \brief QSH : Quasi Shears represents a bijective rotation through shears
 * @tparam TSpace a 2 dimensional space.
 * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
 * @tparam TOutputValue type of the output point e.g., TSpace::Point
*/

template < typename TSpace, typename TInputValue = typename TSpace::Point, typename TOutputValue = typename TSpace::Point>
struct QSH {
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

    /**
   * QSH Constructor.
   * @param ang  the angle given in radians.
   * @param ptCenter  the center of rotation.
   */
    QSH(double ang, TOutputValue ptCenter  );

    /// init a QSH rotation using Andres' parameters
    void initQSHRotation(){
            a = round(omega*std::sin(my_angle));
            aprime = round(omega*std::sin(my_angle/2.0));
            bprime = round(omega*std::cos(my_angle/2.0));
    }

    /// @return the angle of rotation.
    inline double    angle() const{return my_angle;};

    /// set rotation angle
    inline void  set_angle(const double new_angle){my_angle=new_angle;initQSHRotation();}


    /// @return the centre of rotation
    inline TOutputValue  center() const{return my_center;};
    /// @return a reference to the centre of rotation
    inline TOutputValue& center(){return my_center;};


  /// @param p a lattice point
  /// @return the rotation of the point \a p according to the current
  /// angle and center.
    TOutputValue rotate( const TInputValue& p ) const;
    TOutputValue operator()( const TInputValue& p ) const;

    std::string tostring() const {
        return {"QSH"};
    }

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
QSH<TSpace,TInputValue,TOutputValue>::QSH( const double ang, const TOutputValue ptCenter  ):my_angle(ang),my_center(ptCenter){
    initQSHRotation();
}



template<typename TSpace, typename TInputValue, typename TOutputValue>
TOutputValue QSH<TSpace,TInputValue,TOutputValue>::rotate( const TInputValue& p ) const{
    return this->operator()(p);
}

template<typename TSpace, typename TInputValue, typename TOutputValue>
TOutputValue QSH<TSpace,TInputValue,TOutputValue>::operator()( const TInputValue& p ) const{

    TOutputValue pcentered = p-my_center;

    TOutputValue y1 = hqs_p(pcentered, aprime, bprime, omega);
    TOutputValue y2 = vqs_p(y1, a, bprime, omega);
    TOutputValue y = hqs_p(y2, aprime, bprime, omega);
    return y+my_center;
}



template<typename TSpace, typename TInputValue, typename TOutputValue>
template<typename TImage>
TImage QSH<TSpace,TInputValue,TOutputValue>::rotateImage( const TImage img ) const{
    typedef typename TImage::Domain TDomain;
    typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::QSH<TSpace>> MyDomainTransformer;
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


#endif //QSH

#undef QSH_RECURSES
#endif // else defined(QSH_RECURSES)