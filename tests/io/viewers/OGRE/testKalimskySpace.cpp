/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

/**
* @file testKalimskySpace.cpp
* @ingroup Tests
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* INSA Lyon
*
* @date 2012/06/13
*
* Functions for testing class testKalimskySpace.
*
* This file is part of the DGtal library.
*/

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/viewers/OGRE/ViewerOgre3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include <iostream>
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ViewerOgre3D.
///////////////////////////////////////////////////////////////////////////////
/**
* Example of a test. To be completed.
*
*/

using namespace DGtal;

bool testKalimskySpace()
{
    new ViewerOgre3D();
    DGtal::ViewerOgre3D & View = DGtal::ViewerOgre3D::getSingleton();
    DGtal::Z3i::KSpace K;
    
    
    DGtal::Z3i::Point plow(0,0,0);  
    DGtal::Z3i::Point pup(3,3,2);
    DGtal::Z3i::Domain domain( plow, pup );
    K.init( plow, pup, true );
    
    DGtal::Z3i::SCell ptlow = K.sPointel( plow ); // pointel (0*2,0*2, 0*2)
    DGtal::Z3i::SCell ptup1 = K.sPointel( pup );  // pointel (3*2,3*2, 2*2)
    DGtal::Z3i::SCell ptup2 = K.sTranslation( ptup1,  DGtal::Z3i::Point::diagonal() ); // pointel (4*2, 4*2, 3*2)

   View << ptlow << ptup1 << ptup2; 
    // drawing cells of dimension 0
    DGtal::Z3i::SCell p1= K.sCell(DGtal::Z3i::Point(0,0,2),false);  // pointel (0*2,0*2,2*2)  
    DGtal::Z3i::SCell p2= K.sCell(DGtal::Z3i::Point(0,2,2));  // ...
    DGtal::Z3i::SCell p3= K.sCell(DGtal::Z3i::Point(2,2,2),false);
    DGtal::Z3i::SCell p4= K.sCell(DGtal::Z3i::Point(2,0,2));
    DGtal::Z3i::SCell p5= K.sCell(DGtal::Z3i::Point(0,0,4),false);
    DGtal::Z3i::SCell p6= K.sCell(DGtal::Z3i::Point(0,2,4));
    DGtal::Z3i::SCell p7= K.sCell(DGtal::Z3i::Point(2,2,4), false);
    DGtal::Z3i::SCell p8= K.sCell(DGtal::Z3i::Point(2,0,4));
    View << p1 << p2 << p3 << p4 << p5 << p6 << p7 << p8;
    
    
    // drawing Cells of dimension 1
    DGtal::Z3i::SCell linel0 = K.sCell( DGtal::Z3i::Point( 1, 0, 2 ) );  // linel (2*1+1, 0, 2*2)
    DGtal::Z3i::SCell linel1 = K.sCell( DGtal::Z3i::Point( 1, 2, 2 ) );  // ...
    DGtal::Z3i::SCell linel2 = K.sCell( DGtal::Z3i::Point( 0, 1, 2 ) ); 
    DGtal::Z3i::SCell linel3 = K.sCell( DGtal::Z3i::Point( 2, 1, 2 ) ); 
    
    DGtal::Z3i::SCell linel4 = K.sCell( DGtal::Z3i::Point( 1, 0, 4 ) );
    DGtal::Z3i::SCell linel5 = K.sCell( DGtal::Z3i::Point( 1, 2, 4 ) );
    DGtal::Z3i::SCell linel6 = K.sCell( DGtal::Z3i::Point( 0, 1, 4 ) );
    DGtal::Z3i::SCell linel7 = K.sCell( DGtal::Z3i::Point( 2, 1, 4 ) );

    DGtal::Z3i::SCell linel8 = K.sCell( DGtal::Z3i::Point( 0, 0, 3 ) );
    DGtal::Z3i::SCell linel9 = K.sCell( DGtal::Z3i::Point( 0, 2, 3 ) );
    DGtal::Z3i::SCell linel10 = K.sCell( DGtal::Z3i::Point( 2, 0, 3 ) );
    DGtal::Z3i::SCell linel11 = K.sCell( DGtal::Z3i::Point( 2, 2, 3 ) );

    
    DGtal::Z3i::SCell linel12 = K.sCell( DGtal::Z3i::Point( 3, 2, 2 ) );
  
    View << linel0<< linel1<< linel2 << linel3 ;
    View << linel4<< linel5<< linel6 << linel7 ;
    View << linel8<< linel9<< linel10 << linel11 << linel12;
    

    DGtal::Z3i::Cell surfelA = K.uCell( DGtal::Z3i::Point( 2, 1, 3 ) ); // surfel (2*2,2*1+1,2*3+1)
    DGtal::Z3i::Cell surfelB = K.uCell( DGtal::Z3i::Point( 1, 0, 1 ) ); // surfel (2*1,2*0,2*1+1)
    DGtal::Z3i::Cell surfelC = K.uCell( DGtal::Z3i::Point( 2, 1, 1 ) ); // surfel (2*2,2*1+1,2*1+1)
    View<< surfelB << surfelC << surfelA;
  
    
  
      
    // drawing cells of dimension 3  
    DGtal::Z3i::SCell vox1 = K.sCell( DGtal::Z3i::Point( 3, 3, 3 ) ); // voxel (2*3+1,2*3+1,2*3+1)
    DGtal::Z3i::SCell vox2 = K.sCell( DGtal::Z3i::Point( 1, 1,  3 ) ,false ); // voxel (2*1+1,2*1+1,2*3+1)  
    View << vox1 << vox2;


  
  
  View.start();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing adding DGtalSet  for ViewerOgre3D" );
  bool res = testKalimskySpace(); 
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
