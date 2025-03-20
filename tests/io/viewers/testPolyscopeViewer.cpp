#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer3D.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

int main( int argc, char** argv )
{
    typedef PolyscopeViewer3D<> MyViewer;
    MyViewer viewer;
    {
        Point p1( 0, 0, 0 );
        Point p2( 20, 20, 20 );
        Domain domain(p1, p2);
        DigitalSet shape_set( domain );

        Shapes<Domain>::addNorm2Ball( shape_set, Point( 10, 10, 10 ), 7 );

        viewer << SetMode3D( shape_set.className(), "Both" );
        viewer << shape_set;
        viewer << CustomColors3D(Color(250, 200,0, 100),Color(250, 200,0, 20));
        viewer <<  SetMode3D( p1.className(), "Paving" );

        viewer << ClippingPlane(1,0,0,-4.9);
        viewer << ClippingPlane(0,1,0.3,-10);

        viewer << MyViewer::updateDisplay;
    }
    // viewer << MyViewer::updateDisplay;
    viewer.show();
    return 0;
}