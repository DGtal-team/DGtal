#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Circulator.h"

#include "DGtal/geometry/curves/ArithmeticalDSSComputerOnSurfels.h"

using namespace std;
using namespace DGtal;

//////////////////////////////////////////////////////////////
int main(void)
{
  typedef KhalimskySpaceND<3, int> KSpace;
  typedef KSpace::Point Point;
  KSpace kspace;

  typedef std::vector<KSpace::SCell> Container;
  Container contour;

  KSpace::SCell surfel1 = kspace.sCell(Point(2*(-1)+1, 2*0+1,2*1), kspace.POS);
  contour.push_back(surfel1);
  KSpace::SCell surfel2 = kspace.sCell(Point(2*(-1)+1, 2*1, 2*0+1), kspace.POS);
  contour.push_back(surfel2);
  KSpace::SCell surfel3 = kspace.sCell(Point(2*(-1)+1, 2*1+1,2*0), kspace.POS);
  contour.push_back(surfel3);

  Dimension dim1 = 1, dim2 = 2; // The two dimensions to project the surfels into
  ArithmeticalDSSComputerOnSurfels<KSpace,Container::const_iterator, KSpace::Space::Integer, 4> dss(kspace, dim1, dim2);

  // Add points while it is possible
  dss.init( contour.begin() );
  while ( ( dss.end() != contour.end() ) &&
          ( dss.extendFront() ) ) {}
  cout << dss << endl;

  return 0;
}
