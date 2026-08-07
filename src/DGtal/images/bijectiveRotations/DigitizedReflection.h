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
 * @file DigitizedReflection.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitizedReflection_RECURSES)
#error Recursive header files inclusion detected in DigitizedReflection.h
#else // defined(DigitizedReflection_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitizedReflection_RECURSES

#if !defined DigitizedReflection_h
/** Prevents repeated inclusion of headers. */
#define DigitizedReflection_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "GAVector.h"

namespace DGtal {
  template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
  struct Reflection
  {
    GAVector<TSpace> normalVector;

    explicit Reflection ( const GAVector<TSpace> & m=GAVector<TSpace>())
        :normalVector(m){}

    /**
   * Operator
   * @return the reflected and digitized point.
   */
    inline
    TOutputValue operator()( const TInputValue & aInput ) const
    {
      DGtal::functors::VectorRounding < TInputValue, TOutputValue > roundingOpe;
      Z2i::RealPoint m_r = Z2i::RealPoint(normalVector.my_gavec[0],normalVector.my_gavec[1]);
      Z2i::RealPoint x_r = Z2i::RealPoint(aInput[0],aInput[1]);
      Z2i::RealPoint p=x_r - 2.0*((x_r[0]*m_r[0] + x_r[1]*m_r[1])/(m_r[0]*m_r[0] + m_r[1]*m_r[1]))*m_r;
      return  roundingOpe(p);
    }

  };
}
#endif //DigitizedReflection

#undef DigitizedReflection_RECURSES
#endif // else defined(DigitizedReflection_RECURSES)