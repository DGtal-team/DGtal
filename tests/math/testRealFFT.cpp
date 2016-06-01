#include <cmath>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/math/RealFFT.h"
#include "DGtal/io/writers/PGMWriter.h"
//#include "VTKWriter.h"

int main()
{
  constexpr typename DGtal::Dimension N = 2;
  using real = float;
  using integer = int;
  using Space = DGtal::SpaceND<N, integer>;
  using Point = typename Space::Point;
  using Domain = DGtal::HyperRectDomain<Space>;
  using Image = DGtal::ImageContainerBySTLVector<Domain, real>;
  using FFT = DGtal::RealFFT<Domain, real>;

  real dT = 5; // Diffusion coefficient

  // Image initialization
  Domain domain{ Point::diagonal(-64), Point::diagonal(63) };
  Image image{ domain };
  for ( auto const& pt : domain )
    image.setValue( pt, pt.norm1() <= 30 ? 255 : 0 );

  FFT fft(image.domain());

  // Copy data
  auto spatial_image = fft.getSpatialImage();
  std::copy( image.cbegin(), image.cend(), spatial_image.begin() );

  // Forward transformation
  fft.forwardFFT(FFTW_ESTIMATE);

  // Convolution
  auto const extent = fft.getSpatialExtent();
  auto freq_image = fft.getFreqImage();
  for ( auto it = freq_image.begin(); it != freq_image.end(); ++it )
    {
      /*
      auto const& point = it.getPoint();

      real norm2 = 0;
      for ( size_t j = 0; j < Image::dimension; ++j)
        {
          real coord = static_cast<real>(point[j]) / extent[j];
          if ( coord >= 0.5 ) coord -= 1.;
          test[j] = coord;
          norm2 += coord*coord;
        }
      */

      const auto freq = fft.calcScaledFreqCoords( it.getPoint() );
      const auto norm2 = freq.dot(freq);
      const real c = std::exp( -4*M_PI*M_PI*dT*norm2 );

      // New value
      auto const v = *it;
      *it = { c*std::real(v), c*std::imag(v) };
    }

  // Back in spatial space
  fft.backwardFFT(FFTW_ESTIMATE, false);

  // Store the result
  const size_t n = fft.getSpatialDomain().size();
  std::transform(
      spatial_image.cbegin(),
      spatial_image.cend(),
      image.begin(),
      [n] (real x) { return x/n; }
  );
  //std::copy( spatial_image.cbegin(), spatial_image.cend(), image.begin() );

  DGtal::PGMWriter<Image, DGtal::functors::Cast<unsigned char> >::exportPGM( "testRealFFT.pgm", image );
  /**
  // Export
  if ( N == 2 || N == 3 )
    {
      DGtal::VTKWriter<Domain>( "fft_test", image.domain() ) << "data" << image;
    }
  **/

  return 0;
}

