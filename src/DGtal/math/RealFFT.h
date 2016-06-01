#pragma once

#include <cstddef>    // std::size_t
#include <stdexcept>  // Exceptions
#include <new>        // std::bad_alloc exception

#include <complex>    // To be included before fftw: see http://www.fftw.org/doc/Complex-numbers.html#Complex-numbers
#include <fftw3.h>

#include <DGtal/kernel/domains/HyperRectDomain.h>
#include <DGtal/images/ArrayImageAdapter.h>

namespace DGtal
{

namespace
{

/// Facility to cast to complex type used by fftw
template <typename TFFTW>
struct FFTWComplexCast
  {
    static inline
    typename TFFTW::complex* apply( typename TFFTW::complex* ptr ) { return ptr; }

    static inline
    typename TFFTW::complex* apply( std::complex<typename TFFTW::real>* ptr ) { return reinterpret_cast<typename TFFTW::complex*>(ptr); }
  };


/** Ugly macro used to call fftw functions depending on the value type (fftw_*, fftwf_* & fftwl_*)
 *
 * @see http://www.fftw.org/doc/Multi_002dDimensional-DFTs-of-Real-Data.html
 * @see http://www.fftw.org/doc/Real_002ddata-DFTs.html
 * @see http://www.fftw.org/doc/Precision.html#Precision
 */
#define FFTW_WRAPPER_GEN(suffix)                                                                                         \
    using size_t  = std::size_t;                                                                                         \
    using complex = fftw ## suffix ## _complex;                                                                          \
    using plan = fftw ## suffix ## _plan;                                                                                \
    using self = FFTWWrapper<real>;                                                                                      \
                                                                                                                         \
    static inline void*   malloc( size_t n )      noexcept { return fftw ## suffix ## _malloc(n); }                      \
    static inline void    free( void* p )         noexcept { fftw ## suffix ## _free(p); }                               \
    static inline void    execute( const plan p ) noexcept { fftw ## suffix ## _execute(p); }                            \
    static inline void    destroy_plan( plan p )  noexcept { fftw ## suffix ## _destroy_plan(p); }                       \
                                                                                                                         \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    plan plan_dft_r2c( int rank, const int* n, real* in, C* out, unsigned flags ) noexcept                               \
      {                                                                                                                  \
        return fftw ## suffix ## _plan_dft_r2c(rank, n, in, FFTWComplexCast<self>::apply(out), flags);                   \
      }                                                                                                                  \
                                                                                                                         \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    plan plan_dft_c2r( int rank, const int* n, C* in, real* out, unsigned flags ) noexcept                               \
      {                                                                                                                  \
        return fftw ## suffix ## _plan_dft_c2r(rank, n, FFTWComplexCast<self>::apply(in), out, flags);                   \
      }                                                                                                                  \
                                                                                                                         \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    void execute_dft_r2c( const plan p, real* in, C* out ) noexcept                                                      \
      {                                                                                                                  \
        fftw ## suffix ## _execute_dft_r2c(p, in, FFTWComplexCast<self>::apply(out));                                    \
      }                                                                                                                  \
                                                                                                                         \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    void execute_dft_c2r( const plan p, C* in, real* out ) noexcept                                                      \
      {                                                                                                                  \
        fftw ## suffix ## _execute_dft_c2r(p, FFTWComplexCast<self>::apply(in), out);                                    \
      }                                                                                                                  \
                                                                                                                         \
    /** Plan creation with fft way specifier.                                                                            \
     *                                                                                                                   \
     * @param way FFTW_FORWARD for real->complex FFT, FFTW_BACKWARD for the reverse way.                                 \
     */                                                                                                                  \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    plan plan_dft( int rank, const int* n, real* in, C* out, int way, unsigned flags ) noexcept                          \
      {                                                                                                                  \
        if ( way == FFTW_FORWARD )                                                                                       \
          return plan_dft_r2c( rank, n, in, out, flags );                                                                \
        else                                                                                                             \
          return plan_dft_c2r( rank, n, out, in, flags );                                                                \
      }                                                                                                                  \
                                                                                                                         \
    /** Plan execution with fft way specifier.                                                                           \
     *                                                                                                                   \
     * @param way FFTW_FORWARD for real->complex FFT, FFTW_BACKWARD for the reverse way.                                 \
     */                                                                                                                  \
    template < typename C >                                                                                              \
    static inline                                                                                                        \
    void execute_dft( const plan p, real* in, C* out, int way ) noexcept                                                 \
      {                                                                                                                  \
        if ( way == FFTW_FORWARD )                                                                                       \
          execute_dft_r2c( p, in, out );                                                                                 \
        else                                                                                                             \
          execute_dft_c2r( p, out, in );                                                                                 \
      }                                                                                                                  \

/// Wrapper to fftw functions depending on value type.
template <typename Real = double>
struct FFTWWrapper;

/** Wrapper implementations to fftw functions for double values.
 * @warning Remember to link against fftw3 library.
 */
template <>
struct FFTWWrapper<double>
  {
    using real = double;
    FFTW_WRAPPER_GEN()
  };

/** Wrapper implementations to fftw functions for float values.
 * @warning Remember to link against fftw3f library.
 */
template <>
struct FFTWWrapper<float>
  {
    using real = float;
    FFTW_WRAPPER_GEN(f)
  };

/** Wrapper implementations to fftw functions for long double values.
 * @warning Remember to link against fftw3l library.
 */
template <>
struct FFTWWrapper<long double>
  {
    using real = long double;
    FFTW_WRAPPER_GEN(l)
  };

} // anonymous namespace


/** Generic real-complex backward and forward Fast Fourier Transform.
 * @tparam  TDomain Type of the domain over which the FFT will be performed.
 * @tparam  T       Values type.
 *
 * @see http://www.fftw.org/doc/index.html
 */
template <
  class TDomain, 
  typename T = double
>
class RealFFT;

/** Specialization for FFT over HyperRectDomain.
 * @tparam  TSpace  Type of the space.
 * @tparam  T       Values type.
 */
template <typename TSpace, typename T>
class RealFFT< HyperRectDomain<TSpace>, T >
  {
  private:
    using FFTW = FFTWWrapper<T>;

  public:
    using Space   = TSpace;
    using Domain  = HyperRectDomain<Space>;
    using Point   = typename Domain::Point;
    using Dimension = typename Domain::Dimension;
    using Real = T;
    using Complex = std::complex<Real>;
    static const Dimension dimension = Domain::dimension;

    /** Constructor.
     * @param aDomain The domain over which the transform will be performed.
     */
    RealFFT( Domain const& aDomain ) noexcept
        : mySpatialDomain{ aDomain }
        , mySpatialExtent{ mySpatialDomain.upperBound() - mySpatialDomain.lowerBound() + Point::diagonal(1) }
        , myFreqExtent{ mySpatialExtent / (Point::diagonal(1) + Point::base(0)) + Point::base(0) }
        , myFreqDomain{ Point::diagonal(0), myFreqExtent - Point::diagonal(1) }
        , myStorage( FFTW::malloc( sizeof(Complex) * myFreqDomain.size() ) )
      {}

    /// Destructor
    ~RealFFT()
      {
        FFTW::free( myStorage );
      }

    /** Checks if storage is valid.
     * @return true if there is an allocated storage, false otherwise.
     */
    bool isValid() const noexcept
      {
        return myStorage != nullptr;
      }

    /** Padding when using real datas. 
     *
     * @return the number of real values used as padding along the last dimension.
     *
     *  \see http://www.fftw.org/doc/Multi_002dDimensional-DFTs-of-Real-Data.html#Multi_002dDimensional-DFTs-of-Real-Data 
     */
    inline  
    size_t getPadding() const noexcept
      {
        return 2*myFreqExtent[0] - mySpatialExtent[0];
      }

    /** Get mutable spatial storage.
     * @warning There is a padding at the end of the first dimension. \see getPadding
     */
    inline
    Real* getSpatialStorage() noexcept
      {
        return reinterpret_cast<Real*>(myStorage);
      }

    /** Get non-mutable spatial storage.
     * @warning There is a padding at the end of the first dimension. \see getPadding
     */
    inline
    Real const* getSpatialStorage() const noexcept
      {
        return reinterpret_cast<Real const*>(myStorage);
      }

    /// Get mutable spatial image.
    inline
    ArrayImageAdapter<Real*, Domain> getSpatialImage() noexcept
      {
        const Domain full_domain { mySpatialDomain.lowerBound(), mySpatialDomain.upperBound() + Point::base(0, getPadding()) };
        return { getSpatialStorage(), full_domain, mySpatialDomain };
      }
    
    /// Get non-mutable spatial image.
    inline
    ArrayImageAdapter<const Real*, Domain> getSpatialImage() const noexcept
      {
        const Domain full_domain { mySpatialDomain.lowerBound(), mySpatialDomain.upperBound() + Point::base(0, getPadding()) };
        return { getSpatialStorage(), full_domain, mySpatialDomain };
      }

    /// Get mutable frequential storage.
    inline
    Complex* getFreqStorage() noexcept
      {
        return reinterpret_cast<Complex*>(myStorage);
      }

    /// Get non-mutable frequential storage.
    inline
    Complex const* getFreqStorage() const noexcept
      {
        return reinterpret_cast<Complex const*>(myStorage);
      }
   
    /// Get mutable frequential image.
    inline
    ArrayImageAdapter<Complex*, Domain> getFreqImage() noexcept
      {
        return { getFreqStorage(), getFreqDomain() };
      }
    
    /// Get non-mutable frequential image.
    inline
    ArrayImageAdapter<Complex*, Domain> getFreqImage() const noexcept
      {
        return { getFreqStorage(), getFreqDomain() };
      }
    
    /// Get spatial domain.
    inline Domain const& getSpatialDomain() const noexcept { return mySpatialDomain; }

    /// Get frequential domain.
    inline Domain const& getFreqDomain()    const noexcept { return myFreqDomain; }

    /// Get spatial domain extent.
    inline Point  const& getSpatialExtent() const noexcept { return mySpatialExtent; }
        
    /// Get frequential domain extent.
    inline Point  const& getFreqExtent()    const noexcept { return myFreqExtent; }

    /** Fast Fourier Transformation.
     *
     * @param flags Planner flags. \see http://www.fftw.org/fftw3_doc/Planner-Flags.html#Planner-Flags 
     * @param way   The direction of the transform: FFTW_FORWARD for real->complex, FFTW_BACKWARD for complex->real.
     */
    void doFFT( unsigned flags = FFTW_MEASURE, int way = FFTW_FORWARD )
      {
        typename FFTW::plan p;

        // Transform dimensions
        int n[dimension];
        for (size_t i = 0; i < dimension; ++i)
          n[dimension-i-1] = mySpatialExtent[i];

        // Create the plan for this transformation
        // Only FFTW_ESTIMATE flag preserves input.
        if ( flags & FFTW_ESTIMATE )
          {
            p = FFTW::plan_dft( dimension, n, getSpatialStorage(), getFreqStorage(), way, FFTW_ESTIMATE );
          }
        else
          {
            // Strategy to preserve input datas while creating DFT plan:
            // - Firstly, checks if a plan already exists for this dimensions.
            p = FFTW::plan_dft( dimension, n, getSpatialStorage(), getFreqStorage(), way, FFTW_WISDOM_ONLY | flags );

            // - Otherwise, create fake input to create the plan.
            if ( p == NULL )
              {
                void* tmp = FFTW::malloc( sizeof(Complex) * myFreqDomain.size() );
                if ( tmp == nullptr )  throw std::bad_alloc{};
                p = FFTW::plan_dft( dimension, n, reinterpret_cast<Real*>(tmp), reinterpret_cast<Complex*>(tmp), way, flags );
                FFTW::free(tmp);
              }
          }

        // We must have a valid plan now ...
        if ( p == NULL ) throw std::runtime_error("No valid DFT plan founded.");

        // Gogogo !
        FFTW::execute_dft( p, getSpatialStorage(), getFreqStorage(), way );

        // Destroying plan
        FFTW::destroy_plan( p );
      }

    /** Forward transformation (spatial -> frequential)
     *
     * @param flags Planner flags. \see http://www.fftw.org/fftw3_doc/Planner-Flags.html#Planner-Flags 
     */
    inline
    void forwardFFT( unsigned flags = FFTW_MEASURE )
      {
        doFFT( flags, FFTW_FORWARD );
      }
    
    /** Backward transformation (frequential -> spatial)
     *
     * @param flags Planner flags. \see http://www.fftw.org/fftw3_doc/Planner-Flags.html#Planner-Flags 
     */
    inline
    void backwardFFT( unsigned flags = FFTW_MEASURE )
      {
        doFFT( flags, FFTW_BACKWARD );
      }

  private:
    const Domain  mySpatialDomain;  ///< Spatial domain (real).
    const Point   mySpatialExtent;  ///< Extent of the spatial domain.
    const Point   myFreqExtent;     ///< Extent of the frequential domain.
    const Domain  myFreqDomain;     ///< Frequential domain (complex).
          void*   myStorage;        ///< Storage.
    
  };

} // namespace DGtal

/* GNU coding style */
/* vim: set ts=2 sw=2 expandtab cindent cinoptions=>4,n-2,{2,^-2,:2,=2,g0,h2,p5,t0,+2,(0,u0,w1,m1 : */
