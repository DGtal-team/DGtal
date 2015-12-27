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
 * @file MeaningfulScaleAnalysis.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/12/27
 *
 * Header file for module MeaningfulScaleAnalysis.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MeaningfulScaleAnalysis_RECURSES)
#error Recursive header files inclusion detected in MeaningfulScaleAnalysis.h
#else // defined(MeaningfulScaleAnalysis_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeaningfulScaleAnalysis_RECURSES

#if !defined MeaningfulScaleAnalysis_h
/** Prevents repeated inclusion of headers. */
#define MeaningfulScaleAnalysis_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class MeaningfulScaleAnalysis
/**
 * Description of class 'MeaningfulScaleAnalysis' <p>
 * \brief Aim:
 */
class MeaningfulScaleAnalysis
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    ~MeaningfulScaleAnalysis();

    // ----------------------- Interface --------------------------------------
public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    MeaningfulScaleAnalysis();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    MeaningfulScaleAnalysis ( const MeaningfulScaleAnalysis & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    MeaningfulScaleAnalysis & operator= ( const MeaningfulScaleAnalysis & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class MeaningfulScaleAnalysis


/**
 * Overloads 'operator<<' for displaying objects of class 'MeaningfulScaleAnalysis'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'MeaningfulScaleAnalysis' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const MeaningfulScaleAnalysis & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/math/MeaningfulScaleAnalysis.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeaningfulScaleAnalysis_h

#undef MeaningfulScaleAnalysis_RECURSES
#endif // else defined(MeaningfulScaleAnalysis_RECURSES)
