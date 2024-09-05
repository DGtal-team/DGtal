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
* @file CBDRFastSolver.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CBDRFASTSOLVER_RECURSES)
#error Recursive header files inclusion detected in CBDRFastSolver.h
#else // defined(CBDRFASTSOLVER_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBDRFASTSOLVER_RECURSES

#if !defined CBDRFASTSOLVER_h
/** Prevents repeated inclusion of headers. */
#define CBDRFASTSOLVER_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "NBijectiveReflectionGenerator.h"
namespace DGtal {
  template<typename TSpace,typename TDomain>
  class CBDRFastSolver {
    typedef CBDR_naiverotation<TSpace> BijectiveReflections;
    typedef Reflection<TSpace> DigitizedReflection;
    public:
    CBDRFastSolver(const std::vector<BijectiveReflections>& vecOptimisedReflections,const double rotAngle,
                    const typename TDomain::Point center,
                    const int km):my_vecOptimisedReflections(vecOptimisedReflections),my_angle(rotAngle),my_center(center),kmax(km){}

    BijectiveReflections solve() {
      double angleInDegrees = my_angle * 180.0 / M_PI;
      angleInDegrees = fmod(angleInDegrees, 360.0);
      if (angleInDegrees < 0) {
        angleInDegrees += 360.0;
      }
      return my_vecOptimisedReflections[static_cast<int>(std::round(angleInDegrees))];
    }
    protected:
    std::vector<BijectiveReflections> my_vecOptimisedReflections;
    double my_angle;
    typename TDomain::Point my_center;
    int kmax;
  };
}



#endif //CBDRFASTSOLVER
#undef CBDRFASTSOLVER_RECURSES
#endif // else defined(CBDRFASTSOLVER_RECURSES)