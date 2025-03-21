#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer3D.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

typedef PolyscopeViewer3D<> MyViewer;

struct BigDataCells
{
  KSpace K;
  std::map< DGtal::int32_t, Z3i::SCell > cells;
};

struct BigDataVoxels
{
  std::map< DGtal::int32_t, Z3i::Point > voxels;
};

int reaction1( void* viewer, DGtal::int32_t name, void* data )
{
  BigDataCells* bg = (BigDataCells*) data;
  stringstream ssMessage;
  ssMessage << "Reaction1 with name " << name << " cell " << bg->K.sKCoords( bg->cells[ name ] )  ;
  ((MyViewer *) viewer)->displayMessage(std::string(ssMessage.str().c_str()));
  trace.info() <<  ssMessage.str() << std::endl;
  return 0;
}
int reaction23( void* viewer, DGtal::int32_t name, void* data )
{
  BigDataCells* bg = (BigDataCells*) data;
  stringstream ssMessage;
  ssMessage <<  "Reaction23 with name " << name << " cell " << bg->K.sKCoords( bg->cells[ name ] );
  ((MyViewer *) viewer)->displayMessage(std::string(ssMessage.str().c_str()));
  trace.info() << ssMessage.str() << std::endl;
  return 0;
}
int reaction4( void* viewer, DGtal::int32_t name, void* data )
{
  BigDataVoxels* bg = static_cast<BigDataVoxels*>(data);
  for (const auto& p : bg->voxels)
    std::cout << p.first << ": " << p.second << std::endl;
  stringstream ssMessage;

  ssMessage <<  "Reaction4 with name " << name << " Voxel " << bg->voxels[name] ;
  ((MyViewer *) viewer)->displayMessage(std::string(ssMessage.str().c_str()));
  trace.info() << ssMessage.str() << std::endl;
  return 0;
}

class Ext : public MyViewer::Extension
{
public:
    virtual void UICallback()
    {

    }

    virtual void OnSelect(
        MyViewer* viewer,

        polyscope::Structure* structure, 
        uint32_t structureIndex, 
        DGtal::int32_t voxelName
    )
    {
        std::cout << voxelName << ": selected" << std::endl;
    }
private:
};

int main( int argc, char** argv )
{
    MyViewer viewer;
        viewer.setExtension(new Ext);
    
        BigDataCells data;
        BigDataVoxels dataV;
        Point p1( 0, 0, 0 );
        Point p2( 5, 5 ,5 );
        Point p3( 2, 3, 4 );

        KSpace & K = data.K;
        K.init( p1, p2, true );
        Point v1 = Z3i::Point(10, 10,10);
        Point v2 = Z3i::Point(9, 9, 9);
        Point v3 = Z3i::Point(11, 11,11);

        dataV.voxels[4001] = v1;
        dataV.voxels[4002] = v2;
        dataV.voxels[4003] = v3;

        viewer.displayMessage(std::string("You can click on surfels or voxel to interact ..."));
        Z3i::SCell surfel1 = K.sCell( Point( 1, 1, 2 ), KSpace::POS );
        Z3i::SCell surfel2 = K.sCell( Point( 3, 3, 4 ), KSpace::NEG );
        Z3i::SCell surfel3 = K.sCell( Point( 5, 6, 5 ), KSpace::POS );
        data.cells[ 10001 ] = surfel1;
        data.cells[ 10002 ] = surfel2;
        data.cells[ 10003 ] = surfel3;
        viewer << SetMode3D( surfel1.className(), "Basic" );
        viewer << SetName3D( 10001, "Toto 1" ) << CustomColors3D( Color::Red, Color::Red ) << surfel1;
        viewer << SetName3D( 10002, "Toto 2" ) << CustomColors3D( Color::Green, Color::Green ) << surfel2;
        viewer << SetName3D( 10003, "Toto 3" ) << CustomColors3D( Color::Blue, Color::Blue ) << surfel3;
        viewer << SetSelectCallback3D( reaction1,  &data, 10001, 10001 );
        viewer << SetSelectCallback3D( reaction23, &data, 10002, 10003 );

        // example by using voxel interaction:
        viewer << SetName3D( 4001 ) << v1;
        viewer << SetName3D( 4002 ) << v2;
        viewer << SetName3D( 4003 ) << v3;
        viewer << SetSelectCallback3D( reaction4, &dataV, 4001,4003 );
        viewer<< MyViewer::updateDisplay;
    
    // viewer << MyViewer::updateDisplay;
    viewer.show();
    return 0;
}