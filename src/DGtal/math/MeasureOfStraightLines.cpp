/** 
 * @file MeasureOfStraightLines.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/03/04
 * 
 * Implementation of methods defined in MeasureOfStraightLines.h 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/math/MeasureOfStraightLines.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/math/MeasureOfStraightLines.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class MeasureOfStraightLines
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :




///////////////////////////////////////////////////////////////////////////////
// Interface - public :


/**
 * Constructor.
 */
DGtal::MeasureOfStraightLines::MeasureOfStraightLines()
{
   ///Default value
   myEpsilon = 0.0005;
 }


 /**
   * Destructor.
   */
DGtal::MeasureOfStraightLines:: ~MeasureOfStraightLines()
 {
 }

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void 
DGtal::MeasureOfStraightLines::selfDisplay( std::ostream & out ) const
{
  out << "[MeasureOfStraightLines]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool 
DGtal::MeasureOfStraightLines::isValid() const
{
  return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
