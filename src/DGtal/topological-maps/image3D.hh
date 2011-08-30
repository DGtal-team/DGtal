//******************************************************************************
#ifndef IMAGE3D_HH
#define IMAGE3D_HH
//******************************************************************************
#include <cassert>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <stdint.h>
#include <iostream>
#include "voxel.hh"
//******************************************************************************
namespace Map3d
{
  /** Definition of the TColor type used to represent the color parameter of
   * voxels in raw images.
   */
  typedef uint16_t TColor;

  /** Type used to deal with color sum or square color sum. Should be enough 
   * until images are larger than 2^36 voxels or until the color type change.
   */
  typedef uint64_t TSumColor;
  
  /** Color used to fill the image on creation.
   */
  const TColor NULL_COLOR = (TColor) 0 ;
  
  /** Image class representing a 3D matrix of voxel with TColor values.
   *
   * @author Guillaume Damiand, Alexandre Dupas
   */
  class CImage3D
  {
  public:
    enum TByteOrder {
      L_ENDIAN = 0,
      B_ENDIAN
    };

  public:


bool CImage3D::isOk() const
{ return FIsOk; }
//------------------------------------------------------------------------------

uint32_t CImage3D::getXSize() const
{ return FSizeX; }
//------------------------------------------------------------------------------

uint32_t CImage3D::getYSize() const
{ return FSizeY; }
//------------------------------------------------------------------------------

uint32_t CImage3D::getZSize() const
{ return FSizeZ; }
//------------------------------------------------------------------------------

uint32_t CImage3D::getNbVoxels() const
{ return FSizeX*FSizeY*FSizeZ; }
//------------------------------------------------------------------------------

long int CImage3D::getNumberOfByte() const
{ return sizeof(CImage3D)+(sizeof(TColor)*FSizeX*FSizeY*FSizeZ); }
//******************************************************************************

void CImage3D::setXSize( uint32_t Ax )
{ FSizeX = Ax; }
//------------------------------------------------------------------------------

void CImage3D::setYSize( uint32_t Ay )
{ FSizeY = Ay; }
//------------------------------------------------------------------------------

void CImage3D::setZSize( uint32_t Az )
{ FSizeZ = Az; }
//******************************************************************************

unsigned long CImage3D::indice( uint32_t Ax, uint32_t Ay, uint32_t Az ) const
{
  assert (isOk());
  assert (Ax>=0 && Ax<getXSize());
  assert (Ay>=0 && Ay<getYSize());
  assert (Az>=0 && Az<getZSize());

  return (Az*FSizeX*FSizeY)+(Ay*FSizeX)+Ax;
}
//------------------------------------------------------------------------------

unsigned long CImage3D::indiceFirstVoxel() const
{ return 0; }
//------------------------------------------------------------------------------

unsigned long CImage3D::indiceLastVoxel() const
{ return indice( getXSize()-1, getYSize()-1, getZSize()-1 ); }
//------------------------------------------------------------------------------

TColor CImage3D::getVoxel(uint32_t Ax, uint32_t Ay, uint32_t Az) const
{ return FImage[indice(Ax,Ay,Az)]; }
//------------------------------------------------------------------------------

TColor CImage3D::getVoxel(const CVoxel& AVoxel) const
{ return getVoxel(AVoxel.getX(), AVoxel.getY(), AVoxel.getZ()); }
//------------------------------------------------------------------------------

void CImage3D::setVoxel( uint32_t Ax, uint32_t Ay, uint32_t Az, TColor AValue )
{ FImage[indice(Ax,Ay,Az)] = AValue; }
//------------------------------------------------------------------------------

void CImage3D::setVoxel(const CVoxel& AVoxel, TColor AValue )
{ setVoxel(AVoxel.getX(), AVoxel.getY(), AVoxel.getZ(), AValue); }
//------------------------------------------------------------------------------

bool CImage3D::isVoxelInInfiniteRegion( uint32_t Ax, uint32_t Ay, 
					uint32_t Az ) const
{
  return ( Ax>=getXSize() || Ay>=getYSize() || Az>=getZSize() );
}
//------------------------------------------------------------------------------

bool CImage3D::isVoxelInInfiniteRegion(const CVoxel & AVoxel) const
{
  return isVoxelInInfiniteRegion(AVoxel.getX(), AVoxel.getY(), AVoxel.getZ());
}
//------------------------------------------------------------------------------
inline
bool CImage3D::isVoxelInImage( uint32_t Ax, uint32_t Ay, uint32_t Az ) const
{
  return
    ( Ax>=0 ) &&
    ( Ay>=0 ) &&
    ( Az>=0 ) &&
    ( Ax<getXSize() ) &&
    ( Ay<getYSize() ) &&
    ( Az<getZSize() );
}
//------------------------------------------------------------------------------
inline
bool CImage3D::isVoxelInImage(const CVoxel & AVoxel) const
{
  return isVoxelInImage(AVoxel.getX(), AVoxel.getY(), AVoxel.getZ());
}
//------------------------------------------------------------------------------

bool CImage3D::existNeighboorVoxel( uint32_t Ax, uint32_t Ay, uint32_t Az, 
				    uint32_t ADir) const
{
  assert( isVoxelInImage(Ax,Ay,Az) );
  assert( ADir>=FIRST_DIR && ADir<LAST_DIR_18CONNECTED );
  return isVoxelInImage(CVoxel(Ax,Ay,Az).neighboor(ADir));
}
//------------------------------------------------------------------------------

bool CImage3D::existNeighboorVoxel(const CVoxel& AVoxel, unsigned int ADir) const
{
  assert( isVoxelInImage(AVoxel) );
  assert( ADir>=FIRST_DIR && ADir<LAST_DIR_18CONNECTED );
  return isVoxelInImage(AVoxel.neighboor(ADir));
}
//------------------------------------------------------------------------------

void CImage3D::fillImage( TColor AValue )
{
  for( uint32_t i = indiceFirstVoxel() ; i <= indiceLastVoxel() ; ++i )
    FImage[i] = AValue;
}
//------------------------------------------------------------------------------

void CImage3D::emptyImage()
{ fillImage(); }
//******************************************************************************

bool CImage3D::sameVoxel(uint32_t Ax1, uint32_t Ay1, uint32_t Az1,
			 uint32_t Ax2, uint32_t Ay2, uint32_t Az2) const
{ return getVoxel(Ax1,Ay1,Az1) == getVoxel(Ax2,Ay2,Az2); }
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelActuLeft( uint32_t Ax, uint32_t Ay,
				  uint32_t Az ) const
{
  if ( Ay==getYSize() || Az==getZSize() ) return true;
  if ( Ax==0 || Ax==getXSize() ) return false;

  return sameVoxel( Ax,Ay,Az, Ax-1,Ay,Az );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelActuBehind( uint32_t Ax, uint32_t Ay,
				    uint32_t Az ) const
{
  if ( Ax==getXSize() || Az==getZSize() ) return true;
  if ( Ay==0 || Ay==getYSize() ) return false;

  return sameVoxel( Ax,Ay,Az, Ax,Ay-1,Az );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelActuUp( uint32_t Ax, uint32_t Ay,
				uint32_t Az ) const
{
  if ( Ax==getXSize() || Ay==getYSize() ) return true;
  if ( Az==0 || Az==getZSize() ) return false;

  return sameVoxel( Ax,Ay,Az, Ax,Ay,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelLeftBehind( uint32_t Ax, uint32_t Ay,
				    uint32_t Az ) const
{
  if ( Ax==0 || Az==getZSize() ) return true;
  if ( Ay==0 || Ay==getYSize() ) return false;

  return sameVoxel( Ax-1,Ay,Az, Ax-1,Ay-1,Az );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelLeftUp( uint32_t Ax, uint32_t Ay,
				uint32_t Az ) const
{
  if ( Ax==0 || Ay==getYSize() ) return true;
  if ( Az==0 || Az==getZSize() ) return false;

  return sameVoxel( Ax-1,Ay,Az, Ax-1,Ay,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelBehindLeft( uint32_t Ax, uint32_t Ay,
				    uint32_t Az ) const
{
  if ( Ay==0 || Az==getZSize() ) return true;
  if ( Ax==0 || Ax==getXSize() ) return false;

  return sameVoxel( Ax,Ay-1,Az, Ax-1,Ay-1,Az );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelBehindUp( uint32_t Ax, uint32_t Ay,
				  uint32_t Az ) const
{
  if ( Ax==getXSize() || Ay==0 ) return true;
  if ( Az==0 || Az==getZSize() ) return false;

  return sameVoxel( Ax,Ay-1,Az, Ax,Ay-1,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelUpLeft( uint32_t Ax, uint32_t Ay,
				uint32_t Az ) const
{
  if ( Ay==getYSize() || Az==0 ) return true;
  if ( Ax==0 || Ax==getXSize() ) return false;

  return sameVoxel( Ax,Ay,Az-1, Ax-1,Ay,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelUpBehind( uint32_t Ax, uint32_t Ay,
				  uint32_t Az ) const
{
  if ( Ax==getXSize() || Az==0 ) return true;
  if ( Ay==0 || Ay==getYSize() ) return false;

  return sameVoxel( Ax,Ay,Az-1, Ax,Ay-1,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelUpbehindLeft( uint32_t Ax, uint32_t Ay,
				      uint32_t Az ) const
{
  if ( Ay==0 || Az==0 ) return true;
  if ( Ax==0 || Ax==getXSize() ) return false;

  return sameVoxel( Ax,Ay-1,Az-1, Ax-1,Ay-1,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelLeftbehindUp( uint32_t Ax, uint32_t Ay,
				      uint32_t Az ) const
{
  if ( Ax==0 || Ay==0 ) return true;
  if ( Az==0 || Az==getZSize() ) return false;

  return sameVoxel( Ax-1,Ay-1,Az, Ax-1,Ay-1,Az-1 );
}
//------------------------------------------------------------------------------

bool CImage3D::sameVoxelLeftupBehind( uint32_t Ax, uint32_t Ay,
				      uint32_t Az ) const
{
  if ( Ax==0 || Az==0 ) return true;
  if ( Ay==0 || Ay==getYSize() ) return false;

  return sameVoxel( Ax-1,Ay,Az-1, Ax-1,Ay-1,Az-1 );
}
//******************************************************************************

bool CImage3D::isVoxelMarked( uint32_t Ax, uint32_t Ay,
			      uint32_t Az ) const
{ return FVoxelsMark[indice(Ax, Ay, Az)]; }
//------------------------------------------------------------------------------

void CImage3D::setVoxelMark( uint32_t Ax, uint32_t Ay, uint32_t Az,
			     bool AOn )
{ FVoxelsMark[indice(Ax, Ay, Az)] = AOn; }
//------------------------------------------------------------------------------

void CImage3D::markVoxel( uint32_t Ax, uint32_t Ay, uint32_t Az )
{ setVoxelMark(Ax, Ay, Az, true); }
//------------------------------------------------------------------------------

void CImage3D::unmarkVoxel( uint32_t Ax, uint32_t Ay, uint32_t Az )
{ setVoxelMark(Ax, Ay, Az, false); }
//------------------------------------------------------------------------------

void CImage3D::unmarkAllVoxels()
{
  for( uint32_t i = indiceFirstVoxel() ; i <= indiceLastVoxel() ; ++i )
    FVoxelsMark[i] = false;
}
//******************************************************************************

void CImage3D::medianFilter()
{
  std::vector<TColor> mask(27);

  uint32_t ind = 0;

  CImage3D res( *this );

  for( uint32_t i = 1 ; i < getXSize() ; i++ )
    for( uint32_t j = 1 ; j < getYSize() ; j++ )
      for( uint32_t k = 1 ; k < getZSize() ; k++ )
	{
	  for( int x = -1 ; x <= 1 ; x++ )
	    for( int y = -1 ; y <= 1 ; y++ )
	      for( int z = -1 ; z <= 1 ; z++ )
		{
		  mask[ind++] = getVoxel( i-x, j-y, k-z );
		}
	  std::sort<std::vector<TColor>::iterator>(mask.begin(),mask.end());
	  res.setVoxel( i, j, k, mask[13] );
	  ind=0;
	}

  delete [] FImage;
  FImage = res.FImage;
  res.FImage = NULL;
}

//******************************************************************************
CImage3D::CImage3D() :
  FSizeX(0), FSizeY(0), FSizeZ(0), FIsOk(false), FImage(NULL), FVoxelsMark(NULL)
{}
//------------------------------------------------------------------------------
CImage3D::CImage3D(uint32_t Ax, uint32_t Ay, uint32_t Az) :
  FSizeX(Ax), FSizeY(Ay), FSizeZ(Az)
{
  FImage      = new TColor[FSizeX*FSizeY*FSizeZ];
  FVoxelsMark = new bool[FSizeX*FSizeY*FSizeZ];
  FIsOk  = ( FImage!=NULL && FVoxelsMark!=NULL );

  if ( FIsOk )
    {
      emptyImage();
      unmarkAllVoxels();
    }
  else
    {
      delete []FImage;       FImage=NULL;
      delete []FVoxelsMark;  FVoxelsMark=NULL;
    }
}
//------------------------------------------------------------------------------
CImage3D::CImage3D( const CImage3D & ACimage3D ) :
  FSizeX(ACimage3D.FSizeX), FSizeY(ACimage3D.FSizeY), FSizeZ(ACimage3D.FSizeZ),
  FIsOk(ACimage3D.FIsOk), FImage(NULL), FVoxelsMark(NULL)
{
  if ( FIsOk )
    {
      FImage      = new TColor[FSizeX*FSizeY*FSizeZ];
      FVoxelsMark = new bool[FSizeX*FSizeY*FSizeZ];
      FIsOk = ( FImage!=NULL && FVoxelsMark!=NULL );
      if ( FIsOk )
	{
	  for (unsigned long i=0; i<FSizeX*FSizeY*FSizeZ; i++)
	    {
	      FImage[i]      = ACimage3D.FImage[i];
	      FVoxelsMark[i] = ACimage3D.FVoxelsMark[i];
	    }
	}
      else
	{
	  delete []FImage;       FImage=NULL;
	  delete []FVoxelsMark;  FVoxelsMark=NULL;
	}
    }
}
//------------------------------------------------------------------------------
CImage3D::CImage3D( const char* AFilename ) :
  FSizeX(0), FSizeY(0), FSizeZ(0), FIsOk(false), FImage(NULL), FVoxelsMark(NULL)
{
  loadRaw3D(AFilename);
}
//------------------------------------------------------------------------------
CImage3D::CImage3D( const char* AFilename,
		    uint32_t ALargX, uint32_t ALargY,
		    uint32_t AFirstPlane, int ANbPlaneToRead,
		    uint32_t ALg ) :
  FSizeX(0), FSizeY(0), FSizeZ(0), FIsOk(false), FImage(NULL), FVoxelsMark(NULL)
{
  loadRaw16(AFilename, ALargX, ALargY, AFirstPlane, ANbPlaneToRead, ALg);
}
//******************************************************************************
CImage3D::~CImage3D()
{
  delete []FImage;
  delete []FVoxelsMark;
}
//*****************************************************************************
int CImage3D::computeZLarg( const char * AFileName, uint32_t AFirstPlane,
			    int ANbPlaneToRead, uint32_t ALg ) const
{
  ifstream is;
  char File[ strlen(AFileName) + ALg + 1 ];
  int  FNbPlaneToRead;
  uint32_t  FSizeZ = 0;
  
// 1) On initialise FNbPlaneToRead
  if (ANbPlaneToRead == 0)
    if (strchr( AFileName, '%' )==NULL) // Si le nom du fichier ne contient pas %
      FNbPlaneToRead = 1;               // Alors on lit un seul fichier.
    else
      FNbPlaneToRead = -1;              // Sinon on les lit tous
  else
    FNbPlaneToRead = ANbPlaneToRead; // Sinon on essaye de lire le nombre demandé

// 2) On compte le nombre de plan qu'il va nous falloir lire
//    (c'est la hauteur de l'image)
  sprintf(File,AFileName,AFirstPlane);
  is.open(File);
  if (!is.is_open())
    {
      cout<<"Erreur durant le chargement de l'image 3d "<<AFileName<<endl;
      return -1;
    }

  while(is.is_open() && FNbPlaneToRead!=0)
    {
      ++FSizeZ;
      if (FNbPlaneToRead>0) --FNbPlaneToRead;
      is.close();
      sprintf(File,AFileName,AFirstPlane+FSizeZ);
      is.open(File);
    }

  return FSizeZ;
}
//******************************************************************************
bool CImage3D::loadPgm3D( const char * AFilename )
{
// 0) On détruit l'ancienne image
  delete []FImage; FImage = NULL;
  delete []FVoxelsMark; FVoxelsMark = NULL;
  FSizeX = FSizeY = FSizeZ = 0;
  FIsOk = false;

  std::ifstream AStream(AFilename);
  char tmp[256];
  uint16_t val16;

  AStream>>tmp;
  if ( AStream.bad() || strcmp(tmp, "PGM3D") )
    {
      cerr<<"Echec de lecture de l'image "<<AFilename
	  <<" au format loadPgm3D."<<endl;
      return false;
    }
    
  AStream>>FSizeX>>FSizeY>>FSizeZ;
  AStream>>val16;

  FImage = new TColor[FSizeX*FSizeY*FSizeZ];
  FVoxelsMark = new bool[FSizeX*FSizeY*FSizeZ];
  FIsOk = ( FImage!=NULL && FVoxelsMark!=NULL );

  if ( !FIsOk )
    {
      cerr<<"Echec d'allocation mémoire pour la lecture de l'image "<<AFilename
	  <<" au format loadPgm3D."<<endl;
      delete []FImage; FImage = NULL;
      delete []FVoxelsMark; FVoxelsMark = NULL;
      return false;
    }
  
  for (uint32_t z=0; z<getZSize(); ++z)
    for (uint32_t y=0; y<getYSize(); ++y)
      for (uint32_t x=0; x<getXSize(); ++x)
	{
	  AStream>>val16;
	  if ( AStream.eof() )
	    {
	      cerr<<"Echec de lecture de l'image "<<AFilename
		  <<" au format loadPgm3D."<<endl;
	      delete []FImage; FImage=NULL;
	      delete []FVoxelsMark; FVoxelsMark=NULL;
	      FIsOk=false;
	      return false;
	    }
	  setVoxel    (x,y,z,val16);
	  setVoxelMark(x,y,z,false);
	}
  
  AStream.close();
  return true;
}
//******************************************************************************
bool CImage3D::loadRaw3D( const char * AFilename )
{
  std::ifstream AStream(AFilename);
  char tmp[256];
  uint32_t SizeX=0, SizeY=0, SizeZ=0;
  uint32_t FirstPlane=0;
  int NbPlaneToRead=0;
  uint32_t NBC=3;
  char FileName[256];
  char FileNameWithPath[512];
  uint32_t order = 0;
  TByteOrder ByteOrder = L_ENDIAN;

  while (!AStream.eof ())
    {
      AStream>>tmp;
      if ( !strcmp(tmp, "FileName:") )            AStream>>FileName;
      else if ( !strcmp(tmp, "SizeX:") )          AStream>>SizeX;
      else if ( !strcmp(tmp, "SizeY:") )          AStream>>SizeY;
      else if ( !strcmp(tmp, "SizeZ:") )          AStream>>SizeZ;
      else if ( !strcmp(tmp, "ByteOrder:") )      AStream>>order;
      else if ( !strcmp(tmp, "FirstPlane:") )     AStream>>FirstPlane;
      else if ( !strcmp(tmp, "NombreChiffres:") ) AStream>>NBC;
      else if ( !strcmp(tmp, "NbPlaneToRead:" ) ) AStream>>NbPlaneToRead;
    }
  AStream.close();

  if( order == 1 )
    {
      ByteOrder = B_ENDIAN;
    }

  const char *chemin=strrchr(AFilename,'/');
  if ( chemin==NULL )
    FileNameWithPath[0]=0;
  else
    {
      strncpy(FileNameWithPath,AFilename,1+chemin-AFilename);
      FileNameWithPath[1+chemin-AFilename]=0;
    }
  strcat(FileNameWithPath,FileName);
  
  if( SizeZ == 0 )
    return loadRaw16( FileNameWithPath,
		      SizeX, SizeY, 
		      FirstPlane, NbPlaneToRead, NBC, 
		      ByteOrder);
  else
    return loadRaw16File( FileNameWithPath,
			  SizeX, SizeY, SizeZ, 
			  FirstPlane, NbPlaneToRead, 
			  ByteOrder );
}
//******************************************************************************
bool CImage3D::loadRaw16File( const char* AFilename, 
			      uint32_t ALargX, uint32_t ALargY, uint32_t ALargZ, 
			      uint32_t AFirstPlane, int ANbPlaneToRead,
			      TByteOrder AOrder )
{
  // On détruit l'ancienne image
  delete []FImage; FImage = NULL;
  delete []FVoxelsMark; FVoxelsMark = NULL;
  FSizeX = ALargX; FSizeY = ALargY; FSizeZ = ALargZ;
  
  FImage = new TColor[FSizeX*FSizeY*FSizeZ];
  FIsOk = ( FImage!=NULL );

  FVoxelsMark = new bool[FSizeX*FSizeY*FSizeZ];
  FIsOk |= ( FVoxelsMark!=NULL );
  if (!isOk())
    {
      cerr<<"Echec d'allocation mémoire dans la lecture de l'image 3d."<<endl;
      delete []FImage;
      delete []FVoxelsMark; 
      return false;
    }
  
  ifstream is;
  uint16_t val16;
  uint16_t val16l;
  is.open( AFilename, ios::binary );
  for ( uint32_t z = 0 ; z < getZSize() ; ++z )
    for ( uint32_t y = 0 ; y < getYSize() ; ++y )
      for ( uint32_t x = 0 ; x < getXSize() ; ++x )
	{
	  is.read((char *)&val16, sizeof(val16));
	  
	  if( AOrder == B_ENDIAN )
	    {
	      val16l = val16 << 8;
	      val16l = val16l + (val16 >> 8);
	      val16 = val16l;
	    }

	  setVoxel    (x,y,z,val16);
	  setVoxelMark(x,y,z,false);
	}
  is.close();
  
  return true;
}
//******************************************************************************
bool CImage3D::loadRaw16( const char * AFilename, uint32_t ALargX,
			  uint32_t ALargY, uint32_t AFirstPlane,
			  int ANbPlaneToRead, uint32_t ALg,
			  TByteOrder AOrder )
{
// 0) On détruit l'ancienne image
  delete []FImage; FImage = NULL;
  delete []FVoxelsMark; FVoxelsMark = NULL;
  FSizeX = ALargX; FSizeY = ALargY; FSizeZ = 0;
  
// 1) On calcule le préfixe à partir de AFilename en cherchant le `%`
  char* prefix = computePrefix(AFilename, ALg); // Initialise prefix
  
// 2) On calcule la hauteur de l'image
  FSizeZ = computeZLarg(prefix, AFirstPlane, ANbPlaneToRead, ALg);
      
// 3) On lit toute les images
  FImage = new TColor[FSizeX*FSizeY*FSizeZ];
  FIsOk = ( FImage!=NULL );

  FVoxelsMark = new bool[FSizeX*FSizeY*FSizeZ];
  FIsOk |= ( FVoxelsMark!=NULL );
  if (!isOk())
    {
      cerr<<"Echec d'allocation mémoire dans la lecture de l'image 3d."<<endl;
      delete []FImage;
      delete []prefix;
      delete []FVoxelsMark; 
      return false;
    }
  
  ifstream is;
  char chaine[strlen(AFilename)+ALg+3];
  uint16_t val16;
  uint16_t val16l;
  for (uint32_t z = 0; z < getZSize(); ++z)
    {
      sprintf(chaine,prefix,z+AFirstPlane);
      
      is.open(chaine, ios::binary);
      for (uint32_t y=0; y<getYSize(); ++y)
	for (uint32_t x=0; x<getXSize(); ++x)
	  {
	    is.read((char *)&val16, sizeof(val16));
	    
	    if( AOrder == B_ENDIAN )
	      {
		val16l = val16 << 8;
		val16l = val16l + (val16 >> 8);
		val16 = val16l;
	      }

	    setVoxel    (x,y,z,val16);
	    setVoxelMark(x,y,z,false);
	  }
      is.close();
    }
  
  delete []prefix;
  return true;
}
//******************************************************************************
void CImage3D::writeRaw3D( const char * AFilename,
			   bool AOneFile ) const
{
  std::ofstream AStream( AFilename, ios::out | ios::binary );
  if ( !AStream )
    {
      cout<<"Problème d'ouverture du fichier '"<<AFilename<<"'. "
	  <<" Echec de la sauvegarde."<<endl;
      return;
    }
  
  char FileName[256];
  char FileNameWithPath[519];
  
  strncpy( FileNameWithPath, AFilename,512 );
  FileNameWithPath[512]=0;
  
  char *extension=strrchr(FileNameWithPath,'.');
  if ( extension!=NULL )
    *extension = 0; // S'il y avait une extension, on la supprime
  
  // Et on rajoute -%.raw
  if( !AOneFile )
    strcat( FileNameWithPath, "-%.raw");
  else
    strcat( FileNameWithPath, ".raw");
  
  // On récupère le nom sans le chemin d'accès.
  char * chemin = strrchr(FileNameWithPath,'/');
  if ( chemin==NULL )
    {
      strncpy( FileName, FileNameWithPath, 255 );
      FileName[255]=0;
    }
  else
    {
      strncpy( FileName, chemin+1,255);
      FileName[255]=0;
    }

  // 1) Ecriture du Raw3D
  AStream<<"FileName: "       << FileName   <<endl;
  AStream<<"SizeX: "          << getXSize() <<endl;
  AStream<<"SizeY: "          << getYSize() <<endl;
  if( AOneFile )
    AStream<<"SizeZ: "        << getZSize() <<endl; 
  AStream<<"FirstPlane: "     << 0          <<endl;
  AStream<<"NombreChiffres: " << 3          <<endl;
  if( !AOneFile )
    AStream<<"NbPlaneToRead: "  << 0          <<endl;
  AStream.close();

  if( !AOneFile )
    writeRaw16(FileNameWithPath );
  else
    writeRaw16File( FileNameWithPath );
}
//******************************************************************************
void CImage3D::writeRaw16File( const char * AFilename, 
			       uint32_t AFirstPlane,
			       uint32_t ANbPlaneToWrite ) const
{
  assert (isOk());
  assert( AFirstPlane + ANbPlaneToWrite <= getZSize() );
    
  if ( ANbPlaneToWrite == 0 )
    ANbPlaneToWrite = getZSize();
  else
    ANbPlaneToWrite += AFirstPlane - 1;

// 2) On ecrit toute les images
  ofstream os;
  uint16_t val16;

  os.open( AFilename, ios::out | ios::binary );
  if ( !os )
    {
      std::cerr << "Error : unable to open the file." << std::endl;
      return;
    }
  for ( uint32_t z = AFirstPlane ; z < ANbPlaneToWrite ; ++z )
    for ( uint32_t y = 0 ; y < getYSize() ; ++y )
      for ( uint32_t x = 0 ; x < getXSize() ; ++x )
	{
	  val16=static_cast<uint16_t>(getVoxel(x,y,z));
	  os.write((char *)&val16, sizeof(val16));	    
	}
  os.close();
}
//******************************************************************************
void CImage3D::writeRaw16( const char * AFilename, uint32_t AFirstPlane,
			   uint32_t ANbPlaneToWrite,
			   uint32_t ALg ) const
{
  assert (isOk());

// 1) On calcule le préfixe à partir de AFilename 
  char * prefix = computePrefix(AFilename, ALg); // Initialise prefix
  if (ANbPlaneToWrite==0) ANbPlaneToWrite=getZSize();

// 2) On ecrit toute les images
  ofstream os;
  char chaine[strlen(AFilename)+ALg+3];
  uint16_t val16;

  uint32_t i = 0;
  while ( AFirstPlane+i < getZSize() && i<ANbPlaneToWrite )
    {
      sprintf( chaine, prefix, AFirstPlane+i );
      os.open( chaine, ios::out | ios::binary );
      if ( !os )
	{
	  cout<<"Problème d'ouverture du fichier '"<<chaine<<"'. "
	      <<" Echec de la sauvegarde."<<endl;
	  delete []prefix;
	  return;
	}
      
      for (uint32_t y=0; y<getYSize(); ++y)
	{
	  for (uint32_t x=0; x<getXSize(); ++x)
	    {
	      val16=static_cast<uint16_t>(getVoxel(x,y,AFirstPlane+i));
	      os.write((char *)&val16, sizeof(val16));	    
	    }
	}
      os.close();
      ++i;
    }
  
  delete []prefix;
}
//******************************************************************************
char* CImage3D::computePrefix( const char * AFileName, uint32_t ALg ) const
{
  assert(AFileName!=NULL && AFileName[0]!=0);

  // +9 car on ajoute  %0nd. (Donc probleme si on a n > 999999)
  char* prefix = new char[strlen(AFileName)+9];

  const char* posoOfOct = strchr( AFileName, '%' ); // Premiere occurence de '%' ou NULL
  
  if ( posoOfOct==NULL )
    {
      strcpy(prefix, AFileName);
    }
  else
    {
      ++posoOfOct;
      strncpy( prefix, AFileName, posoOfOct-AFileName ); // recopie jusqu'au % (compris a cause du ++ précédent)
      prefix[posoOfOct-AFileName]=0;
      char tmp[9];
      sprintf(tmp,"0%uu",ALg);
      strcat ( prefix, tmp );           // ajoute %04d (exemple si n=4)
      strcat ( prefix, posoOfOct );      // recopie le reste de la chaine
    }  

  return prefix;
}
//******************************************************************************
void CImage3D::histogram( std::ostream & out )
{
  TColor hist[ 65536 ];
  for( uint i = 0 ; i < 65536 ; ++i )
    hist[ i ] = 0;

  for( unsigned long i = 0 ; i < FSizeX * FSizeY * FSizeZ ; i++ )
    {
      hist[ FImage[i] ] += 1;
    }

  for( unsigned int i = 0 ; i < 65536 ; ++i )
    {
      out << i << " " << hist[ i ] << std::endl;
    }
}
//******************************************************************************
void CImage3D::genCubeImage(unsigned int ACubeSize)
{
  assert (isOk());
  assert( ACubeSize>0 );
  
  // On vide l'image
  emptyImage();
  
  TColor id = NULL_COLOR;
  for (unsigned int z=0; z<getZSize();++z)
    {
      for (unsigned int y=0; y<getYSize();++y)
	{
	  for (unsigned int x=0; x<getXSize();++x)
	    {
	      id = 1+
		(x / ACubeSize) +
		(y / ACubeSize) * ((getXSize()+ACubeSize-1)/ACubeSize) +
		(z / ACubeSize) * ((getXSize()+ACubeSize-1)/ACubeSize) *
		((getYSize()+ACubeSize-1)/ACubeSize);
	      setVoxel( x,y,z, id );
	    }
	}
    }
}
//******************************************************************************
void CImage3D::genImageRandomly( unsigned int ANbRegions,
				 unsigned int AAverageSize )
{
  assert( isOk() );
  assert( ANbRegions>0 );
  
  // Initialisation du générateur de nombre aléatoire.
  srand(getpid()*time(NULL));

  // On vide l'image
  emptyImage();

  // On tire des graines de manière aléatoires
  // (-1 car la region NULL_COLOR sera notre fond).
  CVoxel graines[ANbRegions-1];
  CVoxel current, next;
  unsigned int i=0;
  unsigned int j=0;
  unsigned int nbVoxel=0;

  for (i=1; i<ANbRegions; ++i)
    {
      do
	{ 
	  current.set(rand()%getXSize(), rand()%getYSize(), rand()%getZSize());
	}
      while(getVoxel(current)!=NULL_COLOR);
      setVoxel( current, i );
      graines[i-1] = current;
    }

  // Maintenant on fait grossir les regions a partir des graines
  for (i=1; i<ANbRegions; ++i)
    {
      unsigned int size = rand()%(2*AAverageSize);
      
      queue<CVoxel> toTreat;

      toTreat.push(graines[i-1]); setVoxel( graines[i-1], NULL_COLOR );
      while( size>0 && !toTreat.empty() )
	{
	  current = toTreat.front(); toTreat.pop();
	  if ( getVoxel(current)==NULL_COLOR )
	    {
	      setVoxel(current,i); ++nbVoxel;
	      --size;

	      for (j=FIRST_DIR_6CONNECTED; j<LAST_DIR_6CONNECTED; ++j)
		{
		  if ( existNeighboorVoxel(current, j) && rand()%101<40)
		    {
		      next = current.neighboor(j);
		      if ( getVoxel(next)==NULL_COLOR )
			toTreat.push(next);
		    }
		}
	    }
	}
    }
}

private:    
    /// Les tailles de l'image en X, Y et Z
    uint32_t FSizeX, FSizeY, FSizeZ;
    bool FIsOk; // vrai si l'image est "valide", faux sinon
    
    /// La matrice d'identifiant
    TColor * FImage;

    /// La matrice de marque des voxels, matrice de booléens
    bool * FVoxelsMark;
  };
} // namespace Map3d
//******************************************************************************
#endif // IMAGE_3D_HH
//******************************************************************************
