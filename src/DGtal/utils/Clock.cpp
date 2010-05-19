/** 
 * @file Clock.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2009/12/11
 * 
 * Implementation of methods defined in Clock.h 
 *
 * This file is part of the DGtal library (backported from Imagene)
 */

///////////////////////////////////////////////////////////////////////////////


#include "DGtal/utils/Clock.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/utils/Clock.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class Clock
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

///////////////////////////////////////////////////////////////////////////////
// -------------------------- timing services -------------------------------


namespace DGtal {


 /**
   * Constructor.
   */
 Clock::Clock()
 {
 }


//- Starts a clock.
void Clock::startClock()
{
#if ( (defined(UNIX)||defined(unix)||defined(linux)) )
  struct itimerval old_timer;
  myTimerInit.it_interval.tv_sec=0;
  myTimerInit.it_interval.tv_usec=0;
  myTimerInit.it_value.tv_sec=100000;
  myTimerInit.it_value.tv_usec=0;
  if (setitimer(ITIMER_VIRTUAL, &myTimerInit, &old_timer)!=0)
    {
      cerr << "[Clock::startClock] Erreur sur 'setitimer()'." << endl;
    }
#elif ( (defined(WIN32)) )
  myFirstTick = clock();
  if (myFirstTick == (clock_t) -1)
    {
      cerr << "[Clock::startClock] Erreur sur 'clock()'." << endl;
    }
#else
  struct itimerval old_timer;
  myTimerInit.it_interval.tv_sec=0;
  myTimerInit.it_interval.tv_usec=0;
  myTimerInit.it_value.tv_sec=100000;
  myTimerInit.it_value.tv_usec=0;
  if (setitimer(ITIMER_VIRTUAL, &myTimerInit, &old_timer)!=0)
    {
      cerr << "[Clock::startClock] Erreur sur 'setitimer()'." << endl;
    }
#endif
}

//- @return the time (in ms) since the last 'startClock()'.
long Clock::stopClock()
{
#if ( (defined(UNIX)||defined(unix)||defined(linux)) )
  long d;
  struct itimerval timer_current;

  if (getitimer(ITIMER_VIRTUAL, &timer_current)!=0)
    {
      cerr << "[Clock::stopClock] Erreur sur 'getitimer()'." << endl;
    }
  d=(myTimerInit.it_value.tv_sec-timer_current.it_value.tv_sec)*1000-
    (timer_current.it_value.tv_usec)/1000;

  //Minimal tick must be positive
  if (d<0)
    return 0;
  else
    return(d);

#elif ( (defined(WIN32)) )
  clock_t last_tick = clock();
  if (last_tick == (clock_t) -1)
    {
      cerr << "[Clock::stopClock] Erreur sur 'clock()'." << endl;
    }
  return (long) ((float) 1000.0 * (float)(last_tick - myFirstTick)
     / (float) CLOCKS_PER_SEC);
#else
  long d;
  struct itimerval timer_current;

  if (getitimer(ITIMER_VIRTUAL, &timer_current)!=0)
    {
      cerr << "[Clock::stopClock] Erreur sur 'getitimer()'." << endl;
    }
  d=(myTimerInit.it_value.tv_sec-timer_current.it_value.tv_sec)*1000-
    (timer_current.it_value.tv_usec)/1000;

  //Minimal tick must be positive
  if (d<0)
    return 0;
  else
    return(d);
#endif
}



/**
 * Destructor. 
 */
DGtal::Clock::~Clock()
{
}



///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void 
DGtal::Clock::selfDisplay( std::ostream & out ) const
{
  out << "[Clock]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool 
DGtal::Clock::isValid() const
{
  return true;
}


///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
}
