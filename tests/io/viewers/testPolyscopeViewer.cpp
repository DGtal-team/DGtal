#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer3D.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

typedef PolyscopeViewer3D<> MyViewer;

int main( int argc, char** argv )
{
    MyViewer viewer;

    std::vector<Z3i::RealPoint> polyg1;

    polyg1.push_back(Z3i::RealPoint(0,0,0));
    polyg1.push_back(Z3i::RealPoint(0,1,0));
    polyg1.push_back(Z3i::RealPoint(1,1,0));

    viewer.addPolygon(polyg1);

    viewer.createNewPolygonList("hop");
    
    std::vector<Z3i::RealPoint> polyg2;  
    
    polyg2.push_back(Z3i::RealPoint(0,10,0));
    polyg2.push_back(Z3i::RealPoint(0,11,0));
    polyg2.push_back(Z3i::RealPoint(11,11,0));
    
    viewer.addPolygon(polyg2);
    
    viewer << MyViewer::updateDisplay;
    viewer.show();
    return 0;
}