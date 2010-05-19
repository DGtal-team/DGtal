/** 
 * @file Clock.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2009/12/11
 * 
 * Header file for module Clock.cpp
 *
 * This file is part of the DGtal library (backported from Imagene)
 */

#if defined(Clock_RECURSES)
#error Recursive header files inclusion detected in Clock.h
#else // defined(Clock_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Clock_RECURSES

#if !defined Clock_h
/** Prevents repeated inclusion of headers. */
#define Clock_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cstdlib>
#if ( (defined(UNIX)||defined(unix)||defined(linux)) )
#include <sys/time.h>
#elif ( (defined(WIN32)) )
#include <time.h>
#else
#include <sys/time.h>
#endif

//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{

  /////////////////////////////////////////////////////////////////////////////
  // class Clock
  /** 
   * Description of class 'Clock' <p>
   * Aim: To provide functions to start and stop a timer. Is useful to get
   * performance of algorithms.
   *
   * \see test_clock.cpp
   */
  class Clock
  {
    // ----------------------- Standard services ------------------------------
    // -------------------------- timing services -------------------------------
  public:
    /**
    * Starts a clock.
    */
    void startClock();
    /**
    * @return the time (in ms) since the last 'startClock()'.
    */
    long stopClock();

    /**
     * Constructor.
     *
     */
    Clock();

    /**
     * Destructor. 
     */
    ~Clock();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Datas --------------------------------
  private:

    ///internal timer object;
#if ( (defined(UNIX)||defined(unix)||defined(linux)) )
    struct itimerval myTimerInit;
#elif ( (defined(WIN32)) )
    clock_t myFirstTick;
#else
    struct itimerval myTimerInit;
#endif

    // ------------------------- Hidden services ------------------------------
  protected:

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Clock( const Clock & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Clock & operator=( const Clock & other );
  
    // ------------------------- Internals ------------------------------------
  private:

  
  }; // end of class Clock


  /**
   * Overloads 'operator<<' for displaying objects of class 'Clock'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Clock' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<<( std::ostream & out, const Clock & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/utils/Clock.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Clock_h

#undef Clock_RECURSES
#endif // else defined(Clock_RECURSES)
