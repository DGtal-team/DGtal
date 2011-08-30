//******************************************************************************
#ifndef REGION_HH
#define REGION_HH
//******************************************************************************
#include <bitset>
#include <cstdlib>
#include <iostream>
#include <cassert>
#include "topological-dart.hh"
#include "coordinate.hh"
#include "kernel-types.hh"
//******************************************************************************
namespace Map3d
{
  class CDart;

  /// Une région, c'est :
  ///   - un identifiant et des pointeurs sur père, frère et premier fils.
  ///   - un brin représentant.
  /// Ce représentant est forcément sur le bord externe de la région
  /// (à l'exeption de la région infinie qui n'a pas de bord externe).
  /// On a également des marques utiles lors de parcours.
  class CRegion
  {
    friend class CTopologicalMap;
  public:


CRegion::CRegion() :
  FId                ( static_cast<TRegionId>(0) ), 
  FFirstVoxel        ( CCoordinate() ), 
  FFather            ( NULL ), 
  FFirstSon          ( NULL ), 
  FBrother           ( NULL ),
  FPrevBrother       ( NULL ),
  FNextSameCC        ( NULL ),
  FPrevSameCC        ( NULL ),
  FRepresentativeDart( NULL ),
  FUFTreeFather      ( this ),
  FUFTreeHeight      ( 0 ),
  FColorMin          ( 0xFFFF ),
  FColorMax          ( 0 ),
  FSumColor          ( 0 ),
  FSumColor2         ( 0 ),
  FNbVoxels          ( 0 ),
  FInternalContrast  ( 0. ),
  FPrev              (NULL),
  FNext              (NULL),
  FNbSurfels         ( 0 )
#ifndef NO_INCREMENTAL_EULER
  ,
  FVertices          ( 0 ),
  FEdges             ( 0 ),
  FFaces             ( 0 )
#endif //NO_INCREMENTAL_EULER
{}
//------------------------------------------------------------------------------

CRegion::CRegion( TRegionId AId ) :
  FId                ( AId ), 
  FFirstVoxel        ( CCoordinate() ), 
  FFather            ( NULL ), 
  FFirstSon          ( NULL ), 
  FBrother           ( NULL ),
  FPrevBrother       ( NULL ),
  FNextSameCC        ( NULL ),
  FPrevSameCC        ( NULL ),
  FRepresentativeDart( NULL ),
  FUFTreeFather      ( this ),
  FUFTreeHeight      ( 0 ),
  FColorMin          ( 0xFFFF ),
  FColorMax          ( 0 ),
  FSumColor          ( 0 ),
  FSumColor2         ( 0 ),
  FNbVoxels          ( 0 ),
  FInternalContrast  ( 0. ),
  FPrev              (NULL),
  FNext              (NULL),
  FNbSurfels         ( 0 )
#ifndef NO_INCREMENTAL_EULER
  ,
  FVertices          ( 0 ),
  FEdges             ( 0 ),
  FFaces             ( 0 )
#endif //NO_INCREMENTAL_EULER
{}
//------------------------------------------------------------------------------

CRegion::CRegion( TRegionId AId, TColor AVoxel, CDart* ADart, 
		  CRegion* AFather ) :
  FId                ( AId     ), 
  FFirstVoxel        ( CCoordinate() ), 
  FFather            ( AFather ), 
  FFirstSon          ( NULL    ), 
  FBrother           ( NULL    ),
  FPrevBrother       ( NULL    ),
  FNextSameCC        ( NULL    ),
  FPrevSameCC        ( NULL    ),
  FRepresentativeDart( ADart   ),
  FUFTreeFather      ( this ),
  FUFTreeHeight      ( 0 ),
  FColorMin          ( AVoxel ),
  FColorMax          ( AVoxel ),
  FSumColor          ( AVoxel ),
  FSumColor2         ( AVoxel*AVoxel ),
  FNbVoxels          ( 1 ),
  FInternalContrast  ( 0. ),
  FPrev              (NULL),
  FNext              (NULL),
  FNbSurfels         ( 0 )
#ifndef NO_INCREMENTAL_EULER
  ,
  FVertices          ( 0 ),
  FEdges             ( 0 ),
  FFaces             ( 0 )
#endif //NO_INCREMENTAL_EULER
{}
//------------------------------------------------------------------------------

CRegion::CRegion( TRegionId AId, TColor AVoxel, CRegion* AFather ) :
  FId                ( AId     ), 
  FFirstVoxel        ( CCoordinate() ), 
  FFather            ( AFather ), 
  FFirstSon          ( NULL    ), 
  FBrother           ( NULL    ),
  FPrevBrother       ( NULL    ),
  FNextSameCC        ( NULL    ),
  FPrevSameCC        ( NULL    ),
  FRepresentativeDart( NULL    ),
  FUFTreeFather      ( this ),
  FUFTreeHeight      ( 0 ),
  FColorMin          ( AVoxel ),
  FColorMax          ( AVoxel ),
  FSumColor          ( AVoxel ),
  FSumColor2         ( AVoxel*AVoxel ),
  FNbVoxels          ( 1 ),
  FInternalContrast  ( 0. ),
  FPrev              (NULL),
  FNext              (NULL),
  FNbSurfels         ( 0 )
#ifndef NO_INCREMENTAL_EULER
  ,
  FVertices          ( 0 ),
  FEdges             ( 0 ),
  FFaces             ( 0 )
#endif //NO_INCREMENTAL_EULER
{}
//******************************************************************************

void CRegion::setBrother( CRegion* ARegion )
{ FBrother=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setPrevBrother( CRegion* ARegion )
{ FPrevBrother=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setFirstSon( CRegion* ARegion )
{ FFirstSon=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setFather( CRegion* ARegion )
{ FFather=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setNextSameCC( CRegion* ARegion )
{ FNextSameCC=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setPrevSameCC( CRegion* ARegion )
{ FPrevSameCC=ARegion; }
//------------------------------------------------------------------------------

void CRegion::setId( TRegionId AId )
{ FId=AId; }
//******************************************************************************

CRegion* CRegion::getFather() const
{ return FFather; }
//------------------------------------------------------------------------------

CRegion* CRegion::getBrother() const
{ return FBrother; }
//------------------------------------------------------------------------------

CRegion* CRegion::getPrevBrother() const
{ return FPrevBrother; }
//------------------------------------------------------------------------------

CRegion* CRegion::getFirstSon() const
{ return FFirstSon; }
//------------------------------------------------------------------------------

CRegion* CRegion::getNextSameCC() const
{ return FNextSameCC; }
//------------------------------------------------------------------------------

CRegion* CRegion::getPrevSameCC() const
{ return FPrevSameCC; }
//------------------------------------------------------------------------------

TRegionId CRegion::getId() const
{ return FId; }
//------------------------------------------------------------------------------

unsigned int CRegion::getNbVoxels() const
{ return FNbVoxels; }
//******************************************************************************

bool CRegion::existBrother() const
{ return ( getBrother()!=NULL ); }
//------------------------------------------------------------------------------

bool CRegion::existPrevBrother() const
{ return ( getPrevBrother()!=NULL ); }
//------------------------------------------------------------------------------

bool CRegion::existSon() const
{ return ( getFirstSon() != NULL ); }
//------------------------------------------------------------------------------

bool CRegion::existFather() const
{ return ( getFather()!=NULL ); }
//------------------------------------------------------------------------------

bool CRegion::existNextSameCC() const
{ return ( getNextSameCC()!=NULL ); }
//------------------------------------------------------------------------------

bool CRegion::existPrevSameCC() const
{ return ( getPrevSameCC()!=NULL ); }
//------------------------------------------------------------------------------

bool CRegion::firstRegionCC() const
{ return ( !existPrevSameCC() ); }
//------------------------------------------------------------------------------

bool CRegion::isFirstSon() const
{ return ( this == getFather()->getFirstSon() ); }
//------------------------------------------------------------------------------
 
bool CRegion::isInfiniteRegion() const
{ return false; }
//******************************************************************************
 
void CRegion::incNbVoxels( unsigned int ANb )
{ FNbVoxels += ANb; }
//------------------------------------------------------------------------------
 
void CRegion::decNbVoxels( unsigned int ANb )
{
  assert ( FNbVoxels>=ANb );
  FNbVoxels -= ANb;
}
//******************************************************************************

void CRegion::setRepresentativeDart( CDart* ADart )
{ FRepresentativeDart = ADart; }
//------------------------------------------------------------------------------

CDart* CRegion::getRepresentativeDart() const
{ return FRepresentativeDart; }
//******************************************************************************

CRegion* CRegion::getNext() const
{ return FNext; }
//------------------------------------------------------------------------------

CRegion* CRegion::getPrev() const
{ return FPrev; }
//------------------------------------------------------------------------------

void CRegion::setNext(CRegion* ARegion)
{ FNext = ARegion; }
//------------------------------------------------------------------------------

void CRegion::setPrev(CRegion* ARegion)
{ FPrev = ARegion; }
//------------------------------------------------------------------------------

bool CRegion::isUFTreeRoot()
{ return FUFTreeFather == this; }
//******************************************************************************

double CRegion::getInternalContrast()
{ return FInternalContrast; }
//------------------------------------------------------------------------------

void CRegion::setInternalContrast( double AValue )
{ FInternalContrast = AValue; }
//******************************************************************************

unsigned int CRegion::getNbSurfels() const
{ return FNbSurfels; }
//******************************************************************************

void CRegion::setNbSurfels( unsigned int AValue )
{ FNbSurfels = AValue; }
//******************************************************************************

const CCoordinate & CRegion::getFirstVoxel() const
{ return FFirstVoxel; }
//------------------------------------------------------------------------------

void CRegion::setFirstVoxel( const CCoordinate & ACoord )
{ FFirstVoxel = ACoord; }
//******************************************************************************

void CRegion::resetColorInformations() 
{ 
  FSumColor = 0;
  FSumColor2 = 0; 
  FColorMin = 0xFFFF; 
  FColorMax = 0; 
  FNbVoxels = 0;
} 
//******************************************************************************

TColor CRegion::getColorMin () const
{ return FColorMin; }
//------------------------------------------------------------------------------

void CRegion::setColorMin ( TColor AColor )
{ FColorMin = AColor; }
//------------------------------------------------------------------------------

TColor CRegion::getColorMax () const
{ return FColorMax; }
//------------------------------------------------------------------------------

void CRegion::setColorMax ( TColor AColor )
{ FColorMax = AColor; }
//------------------------------------------------------------------------------

TColor CRegion::getColorMean () const
{ return isInfiniteRegion() ? 0 : static_cast<TColor>(FSumColor/FNbVoxels); }
//******************************************************************************

TColor CRegion::getRange() const
{ return FColorMax-FColorMin; }
//******************************************************************************

TColor CRegion::getColorMean2 () const
{ return isInfiniteRegion() ? 0 : static_cast<TColor>(FSumColor2/FNbVoxels); }
//******************************************************************************

TSumColor CRegion::getSumColor () const
{ return FSumColor; }
//******************************************************************************

TSumColor CRegion::getSumColor2 () const
{ return FSumColor2; }
//******************************************************************************

void CRegion::setNullColor()
{ FSumColor = 0;  FSumColor2 = 0;}
//******************************************************************************

bool CRegion::isInRegion ( TColor AVoxel, TColor AThreshold )
{ return ( AThreshold >= std::max( FColorMax, AVoxel ) -
	   std::min( FColorMin, AVoxel ) ); }
//******************************************************************************

bool CRegion::isMergeable( CRegion* ARegion, TColor AThreshold )
{ return ( AThreshold >= std::max( FColorMax, ARegion->getColorMax() ) -
	   std::min( FColorMin, ARegion->getColorMin() ) ); }
//******************************************************************************

void CRegion::addVoxel( TColor AVoxel )
{
  if( AVoxel < FColorMin )
    FColorMin = AVoxel;
  if( AVoxel > FColorMax )
    FColorMax = AVoxel;
  
  FSumColor  += AVoxel;
  FSumColor2 += AVoxel*AVoxel;
  FNbVoxels  += 1;
}
//******************************************************************************

void CRegion::removeVoxel( TColor AVoxel )
{
  FSumColor  -= AVoxel;
  FSumColor2 -= AVoxel*AVoxel;
  FNbVoxels  -= 1;
}
//******************************************************************************

void CRegion::addVoxels ( CRegion* ARegion )
{
  TColor minC = std::min(getColorMin(),ARegion->getColorMin());
  TColor maxC = std::max(getColorMax(),ARegion->getColorMax());
  
  FSumColor  += ARegion->FSumColor;
  FSumColor2 += ARegion->FSumColor2;
  FNbVoxels  += ARegion->FNbVoxels;

  setColorMin(minC);
  setColorMax(maxC); 
}
//******************************************************************************

unsigned int CRegion::depth() const
{
  unsigned int res = 0;
  unsigned int tmp = 0;

  if ( existBrother() )
    res = getBrother()->depth();

  if ( existNextSameCC() )
    {
      tmp = getNextSameCC()->depth();
      if ( tmp > res ) res = tmp;
    }

  if ( existSon() )
    {
      tmp = 1 + getFirstSon()->depth();
      if ( tmp > res ) res = tmp;
    }

  return res;
}
//******************************************************************************

void CRegion::removeIsolatedRegion()
{
  // Si la région est isolée, il n'y a pas de région dans la mêmeCC.
  assert ( !existNextSameCC() );  
 
  if ( isFirstSon() ) // Si la région était le premier fils
    {
      assert( existFather() );
      getFather()->setFirstSon(getBrother());
    }
  else
    { // Sinon il faut mettre à jour le champ brother de la
      // région précédente.
      assert( existPrevBrother() );
      getPrevBrother()->setBrother(getBrother());
    }
  
  if ( existBrother() ) // Si la région n'est pas la dernière
    { // Il faut mettre à jour le champ prevBrother de la
      // région suivante.
      getBrother()->setPrevBrother(getPrevBrother());
    }

  assert ( getRepresentativeDart()!=NULL );
  assert ( !getRepresentativeDart()->isFree3() );
  CRegion* boundingReg = static_cast<CTopologicalDart*>
    (getRepresentativeDart()->getBeta3())->getRegion();

  assert ( boundingReg!=NULL );
  
  boundingReg->addVoxels(this);  
}
//******************************************************************************

void CRegion::disconnectRegion()
{
  assert( !isInfiniteRegion() );
  if(firstRegionCC())
    {
      if(existNextSameCC())
	{
	  CRegion* r1 = getNextSameCC();
	  CRegion* tmp = r1;
	  while( tmp != NULL) 
	    {
	      if((((CTopologicalDart*)
		   (tmp->getRepresentativeDart()->getBeta3()))
		  ->getRegion()) == tmp->getFather())
		{
		  r1 = tmp;
		  break;
		}
	      tmp = tmp->getNextSameCC();
	    }
	  // On retire r1 de la liste de la CC
	  r1->getPrevSameCC()->setNextSameCC(r1->getNextSameCC());
	  if(r1->existNextSameCC())
	    r1->getNextSameCC()->setPrevSameCC(getPrevSameCC());
	  // On remplace la région courante par r1 dans l'arborescence
	  r1->setBrother(getBrother());
	  r1->setPrevBrother(getPrevBrother());
	  r1->setNextSameCC(getNextSameCC());
	  r1->setPrevSameCC(getPrevSameCC());
	  if(r1->existBrother())
	    r1->getBrother()->setPrevBrother(r1);
	  if(r1->existPrevBrother())
	    r1->getPrevBrother()->setBrother(r1);
	  else
	    getFather()->setFirstSon(r1);
	  if(r1->existNextSameCC())
	    r1->getNextSameCC()->setPrevSameCC(r1);
	  if(r1->existPrevSameCC())
	    r1->getPrevSameCC()->setNextSameCC(r1);
      	}
      else
	{
	  if(existBrother())
	    getBrother()->setPrevBrother(getPrevBrother());
	  if(existPrevBrother())
	    getPrevBrother()->setBrother(getBrother());
	  else
	    {
	      assert( getFather() != NULL );
	      if( getFather()->getFirstSon() == this )
		getFather()->setFirstSon(getBrother());
	    }
	}   
    }
  else
    {
      getPrevSameCC()->setNextSameCC(getNextSameCC());
      if(existNextSameCC())
	getNextSameCC()->setPrevSameCC(getPrevSameCC());
    }
  setBrother(NULL);
  setPrevBrother(NULL);
  setNextSameCC(NULL);
  setPrevSameCC(NULL);
  setFather(NULL);
}
//******************************************************************************

void CRegion::addSon( CRegion* ARegion )
{ 
  assert( ARegion!=NULL );

  ARegion->FFather      = this;
  ARegion->FBrother     = FFirstSon;
  ARegion->FPrevBrother = NULL;
  if(existSon()) FFirstSon->setPrevBrother(ARegion);
  FFirstSon             = ARegion;
}
//------------------------------------------------------------------------------

void CRegion::addSons( CRegion* ARegion )
{ 
  CRegion* current = ARegion;
  CRegion* brother;
  while( current!=NULL )
    {
      brother = current->getBrother();
      addSon(current);
      current = brother;
    }
}
//------------------------------------------------------------------------------

void CRegion::addBrother( CRegion* ARegion )
{ 
  assert( getFather()!=NULL );
  getFather()->addSon(ARegion);
}
//------------------------------------------------------------------------------

void CRegion::addSameCC( CRegion* ARegion )
{
  assert( ARegion!=NULL );
  assert( ARegion!=this );
  
  ARegion->FFather     = FFather;
  ARegion->FNextSameCC = FNextSameCC;
  ARegion->FPrevSameCC = this;
  if(existNextSameCC()) FNextSameCC->setPrevSameCC(ARegion);
  FNextSameCC          = ARegion;
}
//******************************************************************************

bool CRegion::getMark(int AMarkNumber) const
{
  assert( 0<=AMarkNumber && AMarkNumber<NB_MARKS_REGION );

  return FMarks[AMarkNumber];
}
//------------------------------------------------------------------------------

void CRegion::setMark(int AMarkNumber, bool AValue)
{
  assert( 0<=AMarkNumber && AMarkNumber<NB_MARKS_REGION );

  FMarks.set(AMarkNumber, AValue);
}
//------------------------------------------------------------------------------

std::bitset<NB_MARKS_REGION> CRegion::getMarks() const
{ return FMarks; }
//------------------------------------------------------------------------------

void CRegion::setMarks(const std::bitset<NB_MARKS_REGION>& AMarks)
{ this->FMarks = AMarks; }
//******************************************************************************
#ifndef NO_INCREMENTAL_EULER
//------------------------------------------------------------------------------

unsigned int CRegion::getNbVertices() const
{ return FVertices; }

unsigned int CRegion::getNbEdges() const
{ return FEdges; }

unsigned int CRegion::getNbFaces() const
{ return FFaces; }
//------------------------------------------------------------------------------

void CRegion::setNbVertices( unsigned int ANb )
{ FVertices = ANb; }

void CRegion::setNbEdges( unsigned int ANb )
{ FEdges = ANb; }

void CRegion::setNbFaces( unsigned int ANb )
{ FFaces = ANb; }
//------------------------------------------------------------------------------

void CRegion::incNbVertices( int ANb )
{
  assert( ANb+int(FVertices)>=0 );
  FVertices += ANb;
}

void CRegion::incNbEdges( int ANb )
{
  assert( ANb+int(FEdges)>=0 );
  FEdges += ANb;
}

void CRegion::incNbFaces( int ANb )
{
  assert( ANb+int(FFaces)>=0 );
  FFaces += ANb;
}
//------------------------------------------------------------------------------

int CRegion::getEuler() const
{ return int(FVertices+FFaces)-int(FEdges); }
//------------------------------------------------------------------------------
#endif //NO_INCREMENTAL_EULER
//******************************************************************************

CRegion* CRegion::find()
{
  CRegion* res = FUFTreeFather;
  while ( res->FUFTreeFather!=res )
    {
      res = res->FUFTreeFather;
    }
  
  CRegion* actu = this;
  CRegion* next = NULL;
  while ( actu!=res )
    {
      next = actu->FUFTreeFather;
      actu->FUFTreeFather = res;
      actu = next;
    }
  
  return res;
}
//******************************************************************************

void CRegion::merge( CRegion* ARegion )
{
  CRegion* region1 = find();
  CRegion* region2 = ARegion->find();
  
  if ( region1==region2 ) return; // Les 2 arbres sont déjà dans la même classe.

  // Lors de la fusion, on met la région de premier voxel le plus petit comme 
  // père de l'autre
  if( region2->getFirstVoxel() < region1->getFirstVoxel() )
    {
      region1->FUFTreeFather=region2;
      region2->addVoxels(region1);
#ifndef NO_INCREMENTAL_EULER  
      region2->incNbVertices( region1->getNbVertices() );
      region2->incNbEdges( region1->getNbEdges() );
      region2->incNbFaces( region1->getNbFaces() );
#endif //NO_INCREMENTAL_EULER  
    }
  else
    {
      region2->FUFTreeFather=region1;
      region1->addVoxels(region2);
#ifndef NO_INCREMENTAL_EULER  
      region1->incNbVertices( region2->getNbVertices() );
      region1->incNbEdges( region2->getNbEdges() );
      region1->incNbFaces( region2->getNbFaces() );
#endif //NO_INCREMENTAL_EULER  
    }
}
//******************************************************************************

CInfiniteRegion::CInfiniteRegion() :
  CRegion()
{}
//------------------------------------------------------------------------------

bool CInfiniteRegion::isInfiniteRegion() const
{ return true; }

//******************************************************************************
void CRegion::draw()
{
  std::cout<<"================= Arbre d'inclusion ================="<<std::endl;

  CDynamicCoverageRegions it(this);
  while ( it.cont() )
    {
      if ( it->isInfiniteRegion() ) 
	{
	  std::cout<<"Infinite"<<std::endl;
	}
      else 
	{
	  // Affichage de la composante connexe

	  if(it->firstRegionCC())
	    {
	      // Affichage du père
	      if(it->getFather()->isInfiniteRegion()) std::cout<<"Infinite - ";
	      else std::cout << it->getFather()->getId() << " - ";
	      
	      std::cout << "(  @" << it->getId() << "  ";
	      CRegion* tmp = it->getNextSameCC();
	      while(NULL != tmp)
		{
		  std::cout << tmp->getId() << "  ";
		  tmp = tmp->getNextSameCC();
		}
	      std::cout << ")";
	      std::cout << std::endl;
	    }
	}
      ++it;
    }
  std::cout<<"============== Fin d'affichage de l'arbre ==========="<<std::endl;
}

CRegion* CRegion::find( TRegionId ARegionId )
{
  for ( CDynamicCoverageRegions it(this); it.cont(); ++it )
    {
      if ( it->getId()==ARegionId ) return *it;
    }
  
  return NULL;
}

bool CRegion::exist( TRegionId AId )
{ return ( find(AId)!=NULL ); }

protected:
    /// Identifiant de la région
    TRegionId FId;

    /// Coordonnée du premier voxel de la région dans l'ordre de parcours.
    CCoordinate FFirstVoxel;
    
    /// Les pointeurs sur les voisins
    CRegion* FFather;      // Le père
    CRegion* FFirstSon;    // Le premier fils
    CRegion* FBrother;     // Le frère
    CRegion* FPrevBrother; // Le précédent dans les frères
    CRegion* FNextSameCC;  // La liste des régions dans la même CC
    CRegion* FPrevSameCC;  // Le précédent dans la même CC

    /// Le brin représentant
    CDart* FRepresentativeDart;

    /// Parent dans l'arbre union-find des régions
    CRegion*     FUFTreeFather;
    /// Hauteur de l'arbre union-find des régions
    unsigned int FUFTreeHeight;

    /// Couleur minimum de la région (pour présegmentation)
    TColor FColorMin;
    /// Couleur maximum de la région (pour présegmentation)
    TColor FColorMax;
    
    /// Couleur moyenne de la region
    TSumColor FSumColor;

    /// Couleur carre de la region
    TSumColor FSumColor2;

    /// Le nombre de voxel de la région
    unsigned int FNbVoxels;
    
    /// Le contraste interne de la région
    double FInternalContrast;

    /// Valeur des marques booléennes du brin
    std::bitset<NB_MARKS_REGION> FMarks;

    /// Région précédente dans la liste chaînée des régions.
    CRegion* FPrev;
    /// Région suivante dans la liste chaînée des régions.
    CRegion* FNext;

    /// Surface (in number of surfels) of the region
    unsigned int FNbSurfels;

#ifndef NO_INCREMENTAL_EULER
    /// Champs pour la caractéristique d'Euler en incrémental
    unsigned int FVertices, FEdges, FFaces;
#endif //NO_INCREMENTAL_EULER
  };

  /// La classe région infinie. La seule différence concerne la méthode
  /// isInfiniteRegion qui va désormais retourner vrai.
  class CInfiniteRegion : public CRegion
  {
  public:
    CInfiniteRegion();
    virtual ~CInfiniteRegion();
    
    virtual bool isInfiniteRegion() const;
  };
  
} // namespace Map3d
//******************************************************************************
#endif // REGION_HH
//******************************************************************************
