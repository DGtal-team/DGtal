

#ifndef OTC_H
#define OTC_H
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include "RBC.h"

namespace DGtal {
  template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
typename TFunctor = DGtal::functors::VectorRounding < TInputValue, TOutputValue >>
struct OTC {
    const std::vector< std::vector< int > >& _table;
    int _dr;
    TOutputValue          center;
    int               max_radius;
    RBC_vec<TSpace,TInputValue,TOutputValue,TFunctor> rbc;
    //std::vector< unsigned char > input; // size 3*W*H
    //std::vector< unsigned char > output;// size 3*W*H
    int angle;
    int _W;
    int _H;
    int _outS;
    std::vector< int > _offset; // precomputes the number of points in circles < k

    OTC( const std::vector< std::vector< int > >& table,
          int dr,
          TOutputValue c, int W, int H, bool white = true )
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
      center = c;
      rbc.center() = c;
      _outS = 2*max_radius+1;
      //output.resize( _outS * _outS * 3 ); // by sb
      //std::fill( output.begin(), output.end(), white ? 255 : 0 ); // by sb
      // Precompute offset table
      _offset.resize( max_radius );
      int n = 0;
      for ( auto r = 0; r < max_radius; r++ )
      {
        _offset[ r ] = n;
        n += rbc.circle( r ).size();
      }
    }

    void setAngle( int alpha )
    {
      angle       = alpha;
      rbc.setAngle() = alpha * M_PI / 180.0;
      std::cout <<"OTC angle="<<rbc.angle()<<std::endl;
    }

    int outSize() const { return _outS; }

    TOutputValue rotatePoint( TInputValue p ) const
    {

      // We must find the correct ring.
      auto  pc0 = static_cast<TOutputValue>(p - center);

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

      //std::cout <<"rpc="<<rpc<<std::endl;
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
      return TOutputValue( q[ 0 ] + center[0] , q[ 1 ] + center[1]);

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

    /// \todo rotate dgtal image
    ///
    template<typename TImage>
    TImage rotateImage( const TImage& img) const
    {
      typedef typename TImage::Domain TDomain;
      typedef DGtal::functors::DomainRigidTransformation2D < typename TImage::Domain, DGtal::OTC<TSpace>> MyDomainTransformer;
      typedef std::pair < typename TSpace::Point, typename TSpace::Point > Bounds;


      MyDomainTransformer domainTransformer ( *this );
      Bounds bounds = domainTransformer ( img.domain() );
      TDomain transformedDomain ( bounds.first, bounds.second );
      TImage rotatedImage ( transformedDomain );

      for (typename TDomain::ConstIterator it = img.domain().begin(); it != img.domain().end(); ++it ) {
        rotatedImage.setValue((*this).operator()(*it),img(*it));
      }
      return rotatedImage;



      return img;
    }


  };
}

#endif //OTC_H
