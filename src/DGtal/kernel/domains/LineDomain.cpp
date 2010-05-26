/**
 * @file LineDomain.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/26
 *
 * Implementation of methods defined in LineDomain.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/kernel/domains/LineDomain.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/kernel/domains/LineDomain.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class LineDomain
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
template<class TSpace>
DGtal::LineDomain<TSpace>::~LineDomain()
{
}

/**
* Constructor.
*/
template<class TSpace>
DGtal::LineDomain<TSpace>::LineDomain()
{
}

///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
template<class TSpace>
void
DGtal::LineDomain<TSpace>::selfDisplay ( std::ostream & out ) const
{
    out << "[LineDomain]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
template<class TSpace>
bool
DGtal::LineDomain<TSpace>::isValid() const
{
    return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
