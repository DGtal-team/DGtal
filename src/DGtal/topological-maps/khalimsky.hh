//******************************************************************************
#ifndef KHALIMSKY_HH
#define KHALIMSKY_HH
//******************************************************************************
#include <bitset>
#include <stdint.h>
#include <cassert>
#include <cstring>
#include <iostream>
#include "triplet.hh"

typedef uint16_t TKhalimskyElt;

//******************************************************************************
/** Organisation des bits de chaque voxel :
 *
 *  0 : Pointel		 7 : Pointel fictif
 *
 *  1 : Lignel x	 8 : Lignel x   fictif
 *  2 : Lignel  y	 9 : Lignel  y  fictif
 *  3 : Lignel   z	10 : Lignel   z fictif
 *
 *  4 : Surfel .yz      11 : Marque Surfel .yz 
 *  5 : Surfel x.z      12 : Marque Surfel x.z 
 *  6 : Surfel xy.      13 : Marque Surfel xy. 
 */
enum
  {
    POINTEL = 1,
    LIGNELX = 2, LIGNELY = 4, LIGNELZ = 8,
    SURFELYZ = 16, SURFELXZ = 32, SURFELXY = 64,

    POINTEL_FICTIF = 128,
    LIGNELX_FICTIF = 256, LIGNELY_FICTIF = 512, LIGNELZ_FICTIF = 1024,
    MARK_SURFELYZ = 2048, MARK_SURFELXZ = 4096, MARK_SURFELXY = 8192
  };

//******************************************************************************
/** Classe permettant de manipuler les matrices de Khalimsky (pointels, lignels
 *  et surfels associés à une image discrète 3d).
 */
//******************************************************************************
class CKhalimsky
{
public:
  //******************************************************************************

  void CKhalimsky::empty()
  {
    //   for (unsigned int i=0;i<FSize;++i)
    //     FMatrix[i]=TKhalimskyElt(0);
    memset(FMatrix,0,FSize*sizeof(TKhalimskyElt));
    FNbSurfelsOn	   = 0;
    FNbMarkedSurfels = 0;
    FSurfelMaskMark  = false;
  }
  //******************************************************************************

  CKhalimsky::CKhalimsky(TNatural ASizeX,
			 TNatural ASizeY,
			 TNatural ASizeZ) :
    FSizeX	  (ASizeX+1),
    FSizeY	  (ASizeY+1),
    FSizeZ	  (ASizeZ+1)
  {
    assert(0 < ASizeX);
    assert(0 < ASizeY);
    assert(0 < ASizeZ);

    FDeltaX = (unsigned int) 1;
    FDeltaY = (unsigned int) FSizeX;
    FDeltaZ = (unsigned int) FSizeX*FSizeY;

    FSize = (unsigned int)(FSizeX) * FSizeY * FSizeZ;

    FMatrix = new TKhalimskyElt[FSize];

    empty();
  }
  //------------------------------------------------------------------------------

  CKhalimsky::~CKhalimsky()
  {
    delete [] FMatrix;
  }
  //******************************************************************************

  TNatural  CKhalimsky::getSizeX() const
  { return FSizeX; }
  //------------------------------------------------------------------------------

  TNatural  CKhalimsky::getSizeY() const
  { return FSizeY; }
  //------------------------------------------------------------------------------

  TNatural  CKhalimsky::getSizeZ() const
  { return FSizeZ; }
  //------------------------------------------------------------------------------

  unsigned int CKhalimsky::getNbSurfelsOn() const
  { return FNbSurfelsOn; }
  //------------------------------------------------------------------------------

  unsigned int CKhalimsky::getNbMarkedSurfels() const
  { return FNbMarkedSurfels; }
  //------------------------------------------------------------------------------

  unsigned int CKhalimsky::getNumberOfByte() const
  { return sizeof(CKhalimsky)+(FSize*2); }
  //******************************************************************************

  CTriplet CKhalimsky::normaliseTripletPointel(const CTriplet& ATriplet) const
  {
    assert( ATriplet.getX()<=FSizeX );
    assert( ATriplet.getY()<=FSizeY );
    assert( ATriplet.getZ()<=FSizeZ );

    CTriplet res(ATriplet);
  
    // On à le droit de déborder de un autour de la matrice, mais on la replie
    // sur un tore (droit = gauche + 1, devant = derriere + 1).
  
    if ( res.getX()==FSizeX )
      { res.setX(0); res.incY(); }

    if ( res.getY()>=FSizeY )
      { res.incY(-FSizeY); res.incZ(); }

    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CKhalimsky::normaliseTripletLinel(const CTriplet& ATriplet) const
  {
    assert( ATriplet.getX()<=FSizeX );
    assert( ATriplet.getY()<=FSizeY );
    assert( ATriplet.getZ()<=FSizeZ ); 

    CTriplet res(normaliseTripletPointel(ATriplet));
  
    if ( res.getLinel()==XNEG )
      {
	if ( res.getX()==0 ) res.setX(FSizeX-1);
	else res.setX(res.getX()-1);

	res.setLinel(XPOS);
      }
  
    if ( res.getLinel()==YNEG )
      {
	if ( res.getY()==0 ) res.setY(FSizeY-1);
	else res.incY(-1);

	res.setLinel(YPOS);
      }
  
    if ( res.getLinel()==ZNEG )
      {
	if ( res.getZ()==0) res.setZ(FSizeZ-1);
	else res.incZ(-1);

	res.setLinel(ZPOS);
      }

    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CKhalimsky::normaliseTripletSurfel(const CTriplet& ATriplet) const
  {
    assert( ATriplet.getX()<=FSizeX );
    assert( ATriplet.getY()<=FSizeY );
    assert( ATriplet.getZ()<=FSizeZ );

    CTriplet res(normaliseTripletLinel(ATriplet));

    bool reverse = (res!=ATriplet); // verifie si on a normalisé le premier linel
  
    if ( res.getLinel2()==XNEG )
      {
	if ( res.getX()==0 ) res.setX(FSizeX-1);
	else res.setX(res.getX()-1);

	reverse = !reverse; // on inverse si on normalise le linel2.
	res.setLinel2(XPOS);
      }
  
    if ( res.getLinel2()==YNEG )
      {
	if ( res.getY()==0 ) res.setY(FSizeY-1);
	else res.setY(res.getY()-1);

	reverse = !reverse; // on inverse si on normalise le linel2.
	res.setLinel2(YPOS);
      }
  
    if ( res.getLinel2()==ZNEG )
      {
	if ( res.getZ()==0) res.setZ(FSizeZ-1);
	else res.setZ(res.getZ()-1);

	reverse = !reverse; // on inverse si on normalise le linel2.
	res.setLinel2(ZPOS);
      }

    if ( reverse ) // si on a effectué un nombre impair de normalisation,
      {           // on inverse les linels pour conserver la bonne orientation.
	res.swapLinels();
      }
  
    return res;
  }
  //******************************************************************************

  bool CKhalimsky::isNormalisedTripletPointel(const CTriplet& ATriplet) const
  { return ATriplet==normaliseTripletPointel(ATriplet); }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isNormalisedTripletLinel(const CTriplet& ATriplet) const
  { return ATriplet==normaliseTripletLinel(ATriplet); }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isNormalisedTripletSurfel(const CTriplet& ATriplet) const
  { return ATriplet==normaliseTripletSurfel(ATriplet); }
  //******************************************************************************

  unsigned int CKhalimsky::getIndex(const CTriplet& ATriplet) const
  {
    assert( isNormalisedTripletPointel(ATriplet) );

    return
      FDeltaX * ATriplet.getX() +
      FDeltaY * ATriplet.getY() +
      FDeltaZ * ATriplet.getZ() ;
  }
  //------------------------------------------------------------------------------

  TKhalimskyElt CKhalimsky::getKhalimskyElt(const CTriplet& ATriplet) const
  { return FMatrix[getIndex(ATriplet)]; }
  //------------------------------------------------------------------------------

  TKhalimskyElt CKhalimsky::getBits(unsigned int AIndex,
				    TKhalimskyElt AMask) const
  {
    assert (AIndex < FSize);
  
    return FMatrix[AIndex] & AMask;
  }
  //------------------------------------------------------------------------------

  TKhalimskyElt CKhalimsky::getBits(const CTriplet& ATriplet,
				    TKhalimskyElt AMask) const
  { return getBits(getIndex(ATriplet),AMask); }
  //------------------------------------------------------------------------------

  void CKhalimsky::setBits(unsigned int AIndex, TKhalimskyElt AMask, bool AOn)
  {
    assert(AIndex < FSize);

    if (AOn)
      FMatrix[AIndex] |= AMask;
    else
      FMatrix[AIndex] &= ~AMask;
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::setBits(const CTriplet& ATriplet,
			   TKhalimskyElt AMask, bool AOn)
  { setBits(getIndex(ATriplet),AMask, AOn); }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isBits(unsigned int AIndex, TKhalimskyElt AMask) const
  { return (getBits(AIndex,AMask)==AMask); }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isBits(const CTriplet& ATriplet, TKhalimskyElt AMask) const
  { return (isBits(getIndex(ATriplet),AMask)); }
  //******************************************************************************

  bool CKhalimsky::isPCell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletPointel(ATriplet));
  
    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return false;

    return getBits(triplet, POINTEL);
  }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isLCell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletLinel(ATriplet));

    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return false;

    switch (ATriplet.getLinel())
      {
      case XNEG:
      case XPOS: return getBits(triplet,LIGNELX);
      case YNEG:
      case YPOS: return getBits(triplet,LIGNELY);
      case ZNEG:
      case ZPOS: return getBits(triplet,LIGNELZ);
      default: assert(false);
      }
    return false;
  }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isL2Cell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(ATriplet);
    triplet.swapLinels();
    return isLCell(triplet);
  }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isFictivePCell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletPointel(ATriplet));
  
    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return false;

    return getBits(triplet, POINTEL_FICTIF);
  }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isFictiveLCell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletPointel(ATriplet));
  
    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return false;

    switch (ATriplet.getLinel())
      {
      case XNEG:
      case XPOS: return getBits(triplet,LIGNELX_FICTIF);
      case YNEG:
      case YPOS: return getBits(triplet,LIGNELY_FICTIF);
      case ZNEG:
      case ZPOS: return getBits(triplet,LIGNELZ_FICTIF);
      default: assert(false);
      }
    return false;
  }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isSCell(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletSurfel(ATriplet));

    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return false;

    switch (ATriplet.getLinel())
      {
      case XNEG:
      case XPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case YNEG:
	  case YPOS: return getBits(triplet,SURFELXY);
	  case ZNEG:
	  case ZPOS: return getBits(triplet,SURFELXZ);
	  default: assert(false);
	  }
	break;

      case YNEG:
      case YPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: return getBits(triplet,SURFELXY);
	  case ZNEG:
	  case ZPOS: return getBits(triplet,SURFELYZ);
	  default: assert(false);
	  }
	break;
      
      case ZNEG:
      case ZPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: return getBits(triplet,SURFELXZ);
	  case YNEG:
	  case YPOS: return getBits(triplet,SURFELYZ);
	  default: assert(false);
	  }
	break;
      
      default: assert(false);
      }
    return false;
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::setPCell(const CTriplet& ATriplet, bool AOn)
  { setBits(getIndex(normaliseTripletPointel(ATriplet)), POINTEL, AOn); }
  //------------------------------------------------------------------------------

  void CKhalimsky::setLCell(const CTriplet& ATriplet, bool AOn)
  {
    unsigned int index = getIndex(normaliseTripletLinel(ATriplet));

    switch (ATriplet.getLinel())
      {
      case XNEG: 
      case XPOS: setBits(index, LIGNELX, AOn); break;
      case YNEG: 
      case YPOS: setBits(index, LIGNELY, AOn); break;
      case ZNEG: 
      case ZPOS: setBits(index, LIGNELZ, AOn); break;
      default: assert(false);
      }
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::setFictivePCell(const CTriplet& ATriplet, bool AOn)
  { setBits(getIndex(normaliseTripletPointel(ATriplet)), POINTEL_FICTIF, AOn); }
  //------------------------------------------------------------------------------

  void CKhalimsky::setFictiveLCell(const CTriplet& ATriplet, bool AOn)
  {
    unsigned int index = getIndex(normaliseTripletLinel(ATriplet));

    switch ( ATriplet.getLinel() )
      {
      case XNEG:
      case XPOS: setBits(index, LIGNELX_FICTIF, AOn); break;
      case YNEG:
      case YPOS: setBits(index, LIGNELY_FICTIF, AOn); break;
      case ZNEG:
      case ZPOS: setBits(index, LIGNELZ_FICTIF, AOn); break;
      default: assert(false);
      }
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::setSCell(const CTriplet& ATriplet, bool AOn)
  {
    unsigned int index = getIndex(normaliseTripletSurfel(ATriplet));

    TKhalimskyElt bit = 0;
    switch ( ATriplet.getLinel() )
      {
      case XNEG:
      case XPOS:
	switch (ATriplet.getLinel2())
	  {
	  case YNEG:
	  case YPOS: bit = SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = SURFELXZ; break;
	  default: assert(false);
	  }
	break;
      
      case YNEG:
      case YPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = SURFELYZ; break;
	  default: assert(false);
	  }
	break;

      case ZNEG:
      case ZPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = SURFELXZ; break;
	  case YNEG:
	  case YPOS: bit = SURFELYZ; break;
	  default: assert(false);
	  }
	break;
      default: assert(false);
      }
	  
    if ( isBits(index, bit)!=AOn )
      {
	setBits(index, bit, AOn);
	FNbSurfelsOn += ( AOn ? +1 : -1 );

	// Si le surfel est actuellement marqué, alors on décompte les marques
	if( !AOn )
	  if( isBits( index, bit << 7 ) != FSurfelMaskMark )
	    FNbMarkedSurfels--;

	// Initialisation de la marque du surfel à non-marqué
	setBits(index, bit << 7, FSurfelMaskMark);
      }
  }
  //******************************************************************************

  bool CKhalimsky::isSurfelMarked(const CTriplet& ATriplet) const
  {
    CTriplet triplet(normaliseTripletSurfel(ATriplet));

    if ( triplet.getX()>=FSizeX ||  triplet.getY()>=FSizeY ||
	 triplet.getZ()>=FSizeZ )
      return FSurfelMaskMark;

    TKhalimskyElt bit = 0;  
    switch ( ATriplet.getLinel() )
      {
      case XNEG:
      case XPOS:
	switch (ATriplet.getLinel2())
	  {
	  case YNEG:
	  case YPOS: bit = MARK_SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = MARK_SURFELXZ; break;
	  default: assert(false);
	  }
	break;
      
      case YNEG:
      case YPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = MARK_SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = MARK_SURFELYZ; break;
	  default: assert(false);
	  }
	break;
      
      case ZNEG:
      case ZPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = MARK_SURFELXZ; break;
	  case YNEG:
	  case YPOS: bit = MARK_SURFELYZ; break;
	  default: assert(false);
	  }
	break;
      default: assert(false);
      }

    return ( isBits(triplet,bit)!=FSurfelMaskMark );
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::setSurfelMark(const CTriplet& ATriplet, bool AOn)
  {
    unsigned int index = getIndex(normaliseTripletSurfel(ATriplet));

    TKhalimskyElt bit = 0;  
    switch ( ATriplet.getLinel() )
      {
      case XNEG:
      case XPOS:
	switch (ATriplet.getLinel2())
	  {
	  case YNEG:
	  case YPOS: bit = MARK_SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = MARK_SURFELXZ; break;
	  default: assert(false);
	  }
	break;
      
      case YNEG:
      case YPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = MARK_SURFELXY; break;
	  case ZNEG:
	  case ZPOS: bit = MARK_SURFELYZ; break;
	  default: assert(false);
	  }
	break;
      
      case ZNEG:
      case ZPOS: 
	switch (ATriplet.getLinel2())
	  {
	  case XNEG:
	  case XPOS: bit = MARK_SURFELXZ; break;
	  case YNEG:
	  case YPOS: bit = MARK_SURFELYZ; break;
	  default: assert(false);
	  }
	break;
      default: assert(false);
      }
  
    if ( isBits(index,bit)!=(AOn ^ FSurfelMaskMark) )
      {
	setBits(index, bit, AOn ^ FSurfelMaskMark);
	FNbMarkedSurfels += ( AOn ? +1 : -1 );
      }
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::markSurfel(const CTriplet& ATriplet)
  { setSurfelMark(ATriplet,true); }  
  //------------------------------------------------------------------------------

  void CKhalimsky::unmarkSurfel(const CTriplet& ATriplet)
  { setSurfelMark(ATriplet,false); }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isWholeSurfelsUnmarked() const
  { return FNbMarkedSurfels==0; }
  //------------------------------------------------------------------------------

  bool CKhalimsky::isWholeSurfelsMarked() const
  { return FNbMarkedSurfels==FNbSurfelsOn; }
  //------------------------------------------------------------------------------

  void CKhalimsky::negateSurfelMaskMark()
  {
    FSurfelMaskMark  = !FSurfelMaskMark;
    FNbMarkedSurfels = FNbSurfelsOn-FNbMarkedSurfels;

    assert( !isSurfelMarked(CTriplet(0,0,0,XPOS,YPOS)) );
  }
  //------------------------------------------------------------------------------

  void CKhalimsky::unmarkAllSurfels()
  {
    // Si aucun surfel n'est marqué, on ne fait rien.
    if ( isWholeSurfelsUnmarked() ) return;
  
    // Si tout les surfels sont marqués, on inverse le masque
    if ( isWholeSurfelsMarked() )
      {
	negateSurfelMaskMark();
      }
    // Sinon on est obligé de parcourir tout les surfels un à un
    else
      {
	for (unsigned int i=0;i<FSize;++i)
	  {
	    setBits(i, MARK_SURFELXY, FSurfelMaskMark);
	    setBits(i, MARK_SURFELXZ, FSurfelMaskMark);
	    setBits(i, MARK_SURFELYZ, FSurfelMaskMark);
	  }
      
	FNbMarkedSurfels = 0;
      }
  }

private:
  /// @name Attributs privés
  //@{
  TNatural       FSizeX , FSizeY , FSizeZ;
  unsigned int   FSize;
  unsigned int   FDeltaX, FDeltaY, FDeltaZ;
  TKhalimskyElt* FMatrix;
  unsigned int   FNbSurfelsOn;
  unsigned int   FNbMarkedSurfels;
  bool           FSurfelMaskMark;
  //@}
};
//******************************************************************************
#endif // KHALIMSKY_HH
//******************************************************************************
