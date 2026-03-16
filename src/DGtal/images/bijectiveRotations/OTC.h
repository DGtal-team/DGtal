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
 * @file OTC.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(OTC_RECURSES)
#error Recursive header files inclusion detected in GAVector.h
#else // defined(OTC_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OTC_RECURSES

#if !defined OTC_h
/** Prevents repeated inclusion of headers. */
#define OTC_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "RBC.h"

namespace DGtal {
  /**
   * Description of template struct OTC
   * \brief OTC : Optimal Transport through Circle bijective rotation
   * @tparam TSpace a 2 dimensional space.
   * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
   * @tparam TOutputValue type of the output point e.g., TSpace::Point
  */
  template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
struct OTC {
    const std::vector< std::vector< int > >& _table;
    int _dr;
    TOutputValue          my_center;
    int               max_radius;
    RBC_vec<TSpace,TInputValue,TOutputValue> rbc;
    int angle;
    int _W;
    int _H;
    int _outS;
    std::vector< int > _offset; // precomputes the number of points in circles < k

    /**
   * OTC Constructor.
   * @param table precomputed table thanks to the OT implementation
   * @param dr  width of each ring (OTC-2, OTC3)
   * @param c center of rotation
   * @param W,H size of the image
   */
    OTC( const std::vector< std::vector< int > >& table,
          int dr,
          TOutputValue c, int W, int H )
      : _table( table ), rbc( 0 )
    {
      _W     = W;
      _H     = H;
      _dr    = dr;
      TOutputValue corner00(   0,   0 );
      TOutputValue cornerW0( W-1,   0 );
      TOutputValue corner0H(   0, H-1 );
      TOutputValue cornerWH( W-1, H-1 );
      int max_radius = int( ceil( sqrt( distance2( c, corner00 ) ) ) );

      max_radius  = std::max( max_radius, int( ceil( sqrt( distance2( c, cornerW0 ) ) ) ) );
      max_radius  = std::max( max_radius, int( ceil( sqrt( distance2( c, corner0H ) ) ) ) );
      max_radius  = std::max( max_radius, int( ceil( sqrt( distance2( c, cornerWH ) ) ) ) );
      rbc.init( max_radius, false );
      my_center = c;
      rbc.center() = c;
      _outS = 2*max_radius+1;

      // Precompute offset table
      _offset.resize( max_radius );
      int n = 0;
      for ( auto r = 0; r < max_radius; r++ )
      {
        _offset[ r ] = n;
        n += rbc.circle( r ).size();
      }
    }


    /// @return the centre of rotation
    inline TOutputValue  center() const{return my_center;}


    void set_angle( double alpha )
    {
      angle       = std::round((alpha*180.0)/M_PI);
      rbc.setAngle() = alpha;
      std::cout <<"OTC angle="<<rbc.angle()<<std::endl;
    }

    int outSize() const { return _outS; }

    TOutputValue rotatePoint( TInputValue p ) const
    {

      // We must find the correct ring.
      auto  pc0 = static_cast<TOutputValue>(p - my_center);

      //std::cout <<"pc0="<<pc0<<std::endl;
      //TOutputValue  pc( pc0[ 0 ], -pc0[ 1 ] );// sb comment
      TOutputValue  pc( pc0[ 0 ], pc0[ 1 ] );
      //std::cout <<"pc="<<pc<<std::endl;
      // Table is for angles in [0°,90°], rotate input points to take
      // care of higher angles.
      TOutputValue  rpc = ( angle < 90 ) ? pc
        : ( angle < 180 ) ? TOutputValue( -pc[ 1 ],  pc[ 0 ] )
        : ( angle < 270 ) ? TOutputValue( -pc[ 0 ], -pc[ 1 ] )
        : TOutputValue( pc[ 1 ], -pc[ 0 ] );

      auto       it = rbc.point2circle.find( rpc );
      if ( it == rbc.point2circle.end() ) return static_cast<TOutputValue>(p);
      int      in_r = it->second[ 0 ];
      int      in_i = it->second[ 1 ];



      int   in_ring = in_r <= 0 ? 0 : 1 + ( (in_r - 1) / _dr ) * _dr;
      int     in_ri = in_i;
      int    offset = _offset[ in_ring ];
      for ( auto k = in_ring; k < in_r; k++ ) in_ri += rbc.circle( k ).size();
      const auto& I = _table[ angle % 90 ];
      int    out_ri = I[ offset + in_ri ];
      auto    out_r = in_ring;
      int     out_i = out_ri;
      while ( out_i >= rbc.circle( out_r ).size() )
      {
        out_i -= rbc.circle( out_r ).size();
        out_r += 1;
      }
      TOutputValue q = rbc.circle( out_r )[ out_i ];
      TOutputValue r = TOutputValue( q[ 0 ] + outSize()/2, -q[ 1 ] + outSize()/2 );
      //unsigned char* pInput  = &input[ 0 ]  + 3*( p[ 1 ] * _W    + p[ 0 ] );
      return TOutputValue( q[ 0 ] + my_center[0] , q[ 1 ] + my_center[1]);

    }

    TOutputValue operator()( const TInputValue & aInput ) const
    {
      return this->rotatePoint(aInput);
    }

    void rotateInput()
    {
      for ( int y = 0; y < _H; y++ )
        for ( int x = 0; x < _W; x++ )
        {
          rotatePoint( TOutputValue( x, y ) );
        }
    }

    template<typename TImage>
    TImage rotateImage( const TImage& img) const
    {
      typedef typename TImage::Domain TDomain;
      typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::OTC<TSpace>> MyDomainTransformer;
      typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;
      typename TSpace::Point bottomLeft(-1,-1);
      typename TSpace::Point topRight(1,1);



      MyDomainTransformer domainTransformer ( *this );
      Bounds bounds = domainTransformer ( img.domain() );
      TDomain transformedDomain ( bounds.first+bottomLeft, bounds.second+topRight );
      TImage rotatedImage ( transformedDomain );

      for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
        rotatedImage.setValue((*this).operator()(*it),img(*it));
      }
      return rotatedImage;



      return img;
    }

    std::string tostring() const {
      return {"OTC"};
    }

  };
}


#undef OTC_RECURSES
#endif // else defined(OTC_RECURSES)

