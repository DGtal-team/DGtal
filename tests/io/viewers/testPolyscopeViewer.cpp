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
        Point p2( 10, 10 , 10 );
        Domain domain( p1, p2 );

        DigitalSet shape_set( domain );
        Shapes<Domain>::addNorm1Ball( shape_set, Point( 5, 5, 5 ), 2 );
        Shapes<Domain>::addNorm2Ball( shape_set, Point( 3, 3, 3 ), 2 );
        viewer <<  CustomColors3D(Color(250, 200,0, 100),Color(250, 200,0, 25));
        viewer << shape_set;

        Object6_18 shape( dt6_18, shape_set );
        viewer << SetMode3D( shape.className(), "DrawAdjacencies" );
        viewer << shape;

        Object18_6 shape2( dt18_6, shape_set );
        viewer << SetMode3D( shape2.className(), "DrawAdjacencies" );
        viewer << shape2;
    }
    viewer << MyViewer::updateDisplay;
    viewer.show();
    return 0;
}