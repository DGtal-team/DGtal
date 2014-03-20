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
 * @file SimpleMatrixLinearAlgebra.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/20
 *
 * Header file for module SimpleMatrixLinearAlgebra.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleMatrixLinearAlgebra_RECURSES)
#error Recursive header files inclusion detected in SimpleMatrixLinearAlgebra.h
#else // defined(SimpleMatrixLinearAlgebra_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleMatrixLinearAlgebra_RECURSES

#if !defined SimpleMatrixLinearAlgebra_h
/** Prevents repeated inclusion of headers. */
#define SimpleMatrixLinearAlgebra_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class SimpleMatrixLinearAlgebra
/**
 * Description of class 'SimpleMatrixLinearAlgebra' <p>
 * \brief Aim:
 */
class SimpleMatrixLinearAlgebra
{
    // ----------------------- Standard services ------------------------------
public:

    // ----------------------- Interface --------------------------------------
public:
    typedef Dimension Index;
    typedef double Scalar;
    typedef SimpleMatrix<Scalar, 3, 3> Matrix;
    typedef Matrix::ColumnVector Vector;

		struct Solver
		{
				Solver& compute(const Matrix& _matrix)
				{
						return *this;
				}

				Vector solve(const Vector& _vector) const
				{
						FATAL_ERROR_MSG(false, "solve not implemented for simple matrix");
						return _vector;
				}
		};

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
    SimpleMatrixLinearAlgebra();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    SimpleMatrixLinearAlgebra ( const SimpleMatrixLinearAlgebra & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    SimpleMatrixLinearAlgebra & operator= ( const SimpleMatrixLinearAlgebra & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class SimpleMatrixLinearAlgebra


} // namespace DGtal

#endif // !defined SimpleMatrixLinearAlgebra_h

#undef SimpleMatrixLinearAlgebra_RECURSES
#endif // else defined(SimpleMatrixLinearAlgebra_RECURSES)
