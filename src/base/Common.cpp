/** 
 * @file Common.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2009/12/10
 * 
 * Implementation of methods defined in Common.h 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/Common.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/base/Common.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

#include <iostream>

using namespace std;



///////////////////////////////////////////////////////////////////////////////
// class Common
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor. 
 */
DGtal::Common::~Common()
{
}



///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void 
DGtal::Common::selfDisplay( std::ostream & out ) const
{
  out << "[Common]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool 
DGtal::Common::isValid() const
{
  return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
