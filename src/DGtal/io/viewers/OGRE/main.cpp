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
* @file main.cpp
* @ingroup Tests
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/26
*
* Functions for testing class ViewerOgre3D.
*
* This file is part of the DGtal library.
*/

///////////////////////////////////////////////////////////////////////////////
#include "ViewerOgre3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include <iostream>
///////////////////////////////////////////////////////////////////////////////



void test3( )
{


  DGtal::ViewerOgre3D View;

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );

  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );

  View.manipulate ( shape_set1, setModify  , 1, 150, 5 );

}


void test1()
{
  DGtal::ViewerOgre3D View;

  DGtal::Z3i::Point p1 ( 0, 0, 0 );
  DGtal::Z3i::Point p2 ( 5, 5 , 5 );
  DGtal::Z3i::Point p3 ( 2, 3, 4 );

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );
  
  
  View << DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),"unselected.jpg");
  View << DGtal::SetViewerMode3D(p1.className(),"Paving");
  View << p1;
  View << p2;
  View << p3;
  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );
  View << DGtal::SetViewerMode3D(shape_set1.className(),"Grid");
  View << DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),"unselected.jpg");
  View << shape_set1;


  DGtal::Z3i::DigitalSet shape_set2 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set2, DGtal::Z3i::Point ( -10, -10, -10 ), 2 );
  View << shape_set2;
  View.start();
}

void test2()
{
  DGtal::ViewerOgre3D View;

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );

  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );
  View << shape_set1;

  DGtal::Z3i::Point p10 ( 30, 30 , 30 );
  View << p10;

  /*
  View << DGtal::ViewerClippingPlane ( 1, 0, 0, -4.9 );
  View << DGtal::ViewerClippingPlane ( 0, 1, 0.3, -10 );
  View << DGtal::ViewerCameraPosition ( 2.500000, 2.500000, 16.078199 )
  << DGtal::ViewerCameraDirection ( 0.000000, 0.000000, -1.000000 )
  << DGtal::ViewerCameraUpVector ( 0.000000, 1.000000, 0.000000 );
  View << DGtal::ViewerCameraZNearFar ( 0.1, 200 );
  */
  DGtal::Z3i::Point p1 ( 0, 0, 0 );
  DGtal::Z3i::Point p2 ( 0, 0 , 0 );


  /*
  cout<<" Begin Scene display "<<endl;
  View.sceneDisplay();
  cout<<" End scene display "<<endl;
  */


  View >> p1;

  View >> p2;

  shape_set1.erase ( p1 );
  shape_set1.erase ( p2 );
  View.clearScene();

// On met le nouvel objet
  View << shape_set1;
  View << p10;
  View.start();


}

 void test4()
 {
   
    DGtal::ViewerOgre3D View;
    DGtal::Z3i::KSpace K;
    
    
    DGtal::Z3i::Point plow(0,0,0);  
    DGtal::Z3i::Point pup(3,3,2);
    DGtal::Z3i::Domain domain( plow, pup );
    K.init( plow, pup, true );
    

  
    DGtal::Z3i::SCell ptlow = K.sPointel( plow ); // pointel (0*2,0*2, 0*2)
    DGtal::Z3i::SCell ptup1 = K.sPointel( pup );  // pointel (3*2,3*2, 2*2)
    DGtal::Z3i::SCell ptup2 = K.sTranslation( ptup1,  DGtal::Z3i::Point::diagonal() ); // pointel (4*2, 4*2, 3*2)

//    View << ptlow << ptup1 << ptup2; 
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
 }

 void test5()
 {
    DGtal::Z3i::Point p4 ( 3, 3 , 3 );
    DGtal::Z3i::Point p5 ( -3, -3 , -3 );
    DGtal::Z3i::Domain domain ( p4, p5 );
    DGtal::ViewerOgre3D View;
    View << domain;
    View.start();	
 }
 
 
 void test6()
 {
   
      DGtal::Z3i::Point p4 ( 30, 30 , 30 );
      DGtal::Z3i::Point p5 ( -30, -30 , -30 );
      DGtal::Z3i::Domain domain ( p4, p5 );
      DGtal::ViewerOgre3D View;
      DGtal::Z3i::DigitalSet shape_set1 ( domain );
      
      DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );
//      View << DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),"unselected.jpg");
//      View << DGtal::CustomViewerStyle3D( shape_set1.className(), new DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),"unselected.jpg") );
      View << shape_set1;
      View.start();
       
 }
int main ( int argc, char** argv )
{
//  test1();
//  test2();
//    test3();
//  test4(); 
//test5();
    test6();
  return 0;
}