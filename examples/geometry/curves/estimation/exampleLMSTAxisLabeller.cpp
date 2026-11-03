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

/**
 * @file geometry/curves/estimation/exampleLMSTAxisLabeller.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2026/10/28
 *
 * An example file named exampleLMSTAxisLabeller.
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to estimate tangents along a whole 3D digital curve
   whose main axis changes, with LambdaMST3DBy2D run by LambdaMSTAxisLabeller,
   and how to set the threshold of the filtered LambdaMST3D.

@see \ref moduleArithDSSReco

\example geometry/curves/estimation/exampleLMSTAxisLabeller.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iterator>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/parametric/EllipticHelix.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"
//! [LMSTAxisLabellerHeader]
#include "DGtal/geometry/curves/estimation/LambdaMSTAxisLabeller.h"
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.h"
#include "DGtal/geometry/curves/estimation/LambdaMST3D.h"
//! [LMSTAxisLabellerHeader]
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  typedef EllipticHelix < Space > Helix;
  typedef NaiveParametricCurveDigitizer3D < Helix > Digitizer;
  typedef Digitizer::DigitalCurve Curve;
  typedef Curve::const_iterator ConstIterator;

  // A digitized elliptic helix: its main axis changes along the curve.
  Helix helix ( 60, 40, 10 );
  Digitizer digitizer;
  digitizer.attach ( &helix );
  digitizer.init ( 0., Helix::getPeriod ( ), 0.0001 );
  Curve curve;
  digitizer.digitize ( std::back_inserter ( curve ) );

  //! [LMSTAxisLabellerInit]
  LambdaMSTAxisLabeller < ConstIterator > labeller;
  labeller.init ( curve.cbegin ( ), curve.cend ( ) );
  //! [LMSTAxisLabellerInit]
  trace.info ( ) << labeller << std::endl;

  //! [LMSTAxisLabellerBy2D]
  LambdaMST3DBy2D < ConstIterator > lmstBy2D;
  std::vector < RealVector > tangentsBy2D;
  labeller.evalByRuns ( lmstBy2D, std::back_inserter ( tangentsBy2D ) );
  //! [LMSTAxisLabellerBy2D]

  //! [LMSTAxisLabellerFiltered]
  typedef Naive3DDSSComputer < ConstIterator, int, 8 > SegmentComputer;
  typedef SaturatedSegmentation < SegmentComputer > Segmentation;
  LambdaMST3D < Segmentation, functors::Lambda64Function, DSSLengthLessEqualFilter < SegmentComputer > > lmst;
  lmst.getDSSFilter ( ).init ( labeller.shortest2DDSSLength ( ) / 2. );
  Segmentation segmentation ( curve.cbegin ( ), curve.cend ( ), SegmentComputer ( ) );
  lmst.attach ( segmentation );
  lmst.init ( curve.cbegin ( ), curve.cend ( ) );
  std::vector < RealVector > tangentsFiltered;
  lmst.eval ( curve.cbegin ( ), curve.cend ( ), std::back_inserter ( tangentsFiltered ) );
  //! [LMSTAxisLabellerFiltered]

  trace.info ( ) << curve.size ( ) << " points, " << tangentsBy2D.size ( ) << " tangents by 2D projections, "
                 << tangentsFiltered.size ( ) << " filtered 3D tangents" << std::endl;
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
