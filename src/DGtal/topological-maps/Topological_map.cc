//******************************************************************************
#include <bitset>
#include "topological-dart.hh"
#include "vertex.hh"
#include "khalimsky.hh"
#include "face.hh"
#include "dynamic-coverage.hh"
#include "region.hh"
#include "coverage-region.hh"
//******************************************************************************
extern long int nbEdgeParcouru;  
extern long int nbAppelCanRemoveEdge;
#ifdef COUNT_INTERVOXEL_ACCESS
unsigned int nbPCell, nbLCell, nbSCell, nbMarksSCell;
#endif
namespace Map3d
{
//******************************************************************************
void CTopologicalMap::initDart(CDart* ADart,const CTriplet & ATriplet,
			       CRegion* ARegion, CFace* AFace)
{
  static_cast<CTopologicalDart*>(ADart)->init(ATriplet, ARegion, AFace);
}
//------------------------------------------------------------------------------
CTopologicalDart* CTopologicalMap::addMapDart( const CTriplet & ATriplet, 
					       CRegion* ARegion, CFace* AFace )
{
  assert( FKhalimsky!=NULL );

  // On appelle la méthode de la classe mère.
  CTopologicalDart* res
    = static_cast<CTopologicalDart*>(CMapGeneric::addMapDart());

  // Et on initialise la partie additionnelle pour le CTopologicalDart.
  initDart(res,FKhalimsky->normaliseTripletPointel(ATriplet), ARegion, AFace);

  return res;
}
//------------------------------------------------------------------------------

void CTopologicalMap::empty()
{
  CMapGeneric::empty();  

  assert( getNbDarts()==0 );

  emptyRegions();
  emptyFaces();
  
  // delete FImage;
  FImage = NULL;
  delete FKhalimsky; FKhalimsky = NULL;
  
#ifdef REGIONS_LOCALIZATION
  delete FRegionsLoc; 
  FRegionsLoc = NULL;
#endif // REGIONS_LOCALIZATION
 
  // Marque pour les arêtes fictives
  FFictiveMark = getNewMark();
  assert( FFictiveMark!=-1 );  
}
//******************************************************************************

int CTopologicalMap::getFictiveMark() const
{ return FFictiveMark; }
//------------------------------------------------------------------------------

TNatural CTopologicalMap::getKhalimskySizeX() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getSizeX();
}
//------------------------------------------------------------------------------

TNatural CTopologicalMap::getKhalimskySizeY() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getSizeY();
}
//------------------------------------------------------------------------------

TNatural CTopologicalMap::getKhalimskySizeZ() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getSizeZ();
}
//------------------------------------------------------------------------------

unsigned int CTopologicalMap::getNbSurfelsOn() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getNbSurfelsOn();
}
//------------------------------------------------------------------------------

unsigned int CTopologicalMap::getNbMarkedSurfels() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getNbMarkedSurfels();
}
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getMemoryForMap() const
{ return sizeof(CTopologicalMap)+getNbDartsInTabs()*sizeof(CTopologicalDart)
    + sizeof(FTabsDarts); }
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getMemoryForInclusionTree() const
{
  assert( getInclusionTreeRoot()!=NULL );
  return (getNbRegions()-1)*sizeof(CRegion)+sizeof(CInfiniteRegion);
}
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getMemoryForFace() const
{ return getNbFaces()*sizeof(CFace); }
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getMemoryForKhalimsky() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->getNumberOfByte();
}
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getMemoryForImage() const
{
  assert( FImage!=NULL );
  return FImage->getNumberOfByte();
}
//------------------------------------------------------------------------------

unsigned long int CTopologicalMap::getTotalMemory() const
{
  return getMemoryForMap() + getMemoryForInclusionTree() +
    getMemoryForImage() + getMemoryForKhalimsky() + getMemoryForFace();
}
//******************************************************************************

CTriplet & CTopologicalMap::getTriplet( CDart * ADart ) const
{
  assert( ADart!=NULL );
  return static_cast<CTopologicalDart*>(ADart)->triplet();
}
//------------------------------------------------------------------------------

void CTopologicalMap::setTriplet( CDart * ADart, const CTriplet & ATriplet )
{
  assert( ADart!=NULL );
  static_cast<CTopologicalDart*>(ADart)->triplet() = ATriplet; 
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isRepresentative(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( getRegion(ADart)!=NULL );

  return getRegion(ADart)->getRepresentativeDart()==ADart;
}
//******************************************************************************

bool CTopologicalMap::isPCell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
#ifdef COUNT_INTERVOXEL_ACCESS
  ++nbPCell;
#endif
  return FKhalimsky->isPCell(ATriplet);
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isLCell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
#ifdef COUNT_INTERVOXEL_ACCESS
  ++nbLCell;
#endif
  return FKhalimsky->isLCell(ATriplet);
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isL2Cell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
#ifdef COUNT_INTERVOXEL_ACCESS
  ++nbLCell;
#endif
  return FKhalimsky->isL2Cell(ATriplet);
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isSCell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
#ifdef COUNT_INTERVOXEL_ACCESS
  ++nbSCell;
#endif
  return FKhalimsky->isSCell(ATriplet);
}    
//------------------------------------------------------------------------------

bool CTopologicalMap::isFictivePCell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->isFictivePCell(ATriplet);
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isFictiveLCell(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->isFictiveLCell(ATriplet);
}
//------------------------------------------------------------------------------

CTriplet CTopologicalMap::normaliseTripletPointel(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->normaliseTripletPointel(ATriplet);
}
//------------------------------------------------------------------------------

CTriplet CTopologicalMap::normaliseTripletLinel  (const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->normaliseTripletLinel(ATriplet);
}
//------------------------------------------------------------------------------

CTriplet CTopologicalMap::normaliseTripletSurfel (const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->normaliseTripletSurfel(ATriplet);
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isSurfelMarked(const CTriplet& ATriplet) const
{
  assert( FKhalimsky!=NULL );
#ifdef COUNT_INTERVOXEL_ACCESS
  ++nbMarksSCell;
#endif
  return FKhalimsky->isSurfelMarked(ATriplet);
}
//------------------------------------------------------------------------------

void CTopologicalMap::setSurfelMark(const CTriplet& ATriplet, bool AOn)
{
  assert( FKhalimsky!=NULL );
  FKhalimsky->setSurfelMark(ATriplet,AOn);
}
//------------------------------------------------------------------------------

void CTopologicalMap::markSurfel(const CTriplet& ATriplet)
{
  assert( FKhalimsky!=NULL );
  FKhalimsky->markSurfel(ATriplet);
}
//------------------------------------------------------------------------------

void CTopologicalMap::unmarkSurfel(const CTriplet& ATriplet)
{
  assert( FKhalimsky!=NULL );
  FKhalimsky->unmarkSurfel(ATriplet);
}
//------------------------------------------------------------------------------

void CTopologicalMap::unmarkAllSurfels()
{
  assert( FKhalimsky!=NULL );
  FKhalimsky->unmarkAllSurfels();
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isWholeSurfelsMarked() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->isWholeSurfelsMarked();
}
//------------------------------------------------------------------------------

bool CTopologicalMap::isWholeSurfelsUnmarked() const
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->isWholeSurfelsUnmarked();
}
//------------------------------------------------------------------------------

void CTopologicalMap::negateSurfelMaskMark()
{
  assert( FKhalimsky!=NULL );
  return FKhalimsky->negateSurfelMaskMark();
}
//******************************************************************************

bool CTopologicalMap::isDegreTwoPointel(const CTriplet& ATriplet) const
{
  assert( isLCell(ATriplet) );
  
  CTriplet triplet(ATriplet.getNextLinel());
  int count = 1;
  while ( count<3 && triplet!=ATriplet )
    {
      if ( isLCell(triplet) ) ++count;
      triplet.setNextLinel();
    }
    
  return (count==2);
}
//------------------------------------------------------------------------------

CTriplet CTopologicalMap::getOtherLinel(const CTriplet& ATriplet) const
{
  assert( isDegreTwoPointel(ATriplet) );
  assert( isLCell(ATriplet) );

  CTriplet triplet(ATriplet.getNextLinel());
  bool found = false;
  while ( !found )
    {
      assert( triplet!=ATriplet );
      if ( isLCell(triplet) ) found = true;
      else triplet.setNextLinel();
    }

  assert( found );
  return triplet;
}
//******************************************************************************

bool CTopologicalMap::isDegreTwoLinel(const CTriplet& ATriplet) const
{
  assert( isSCell(ATriplet) );
  
  CTriplet triplet(ATriplet.getNextSurfel());
  int count = 1;
  while ( count<3 && triplet!=ATriplet )
    {
      if ( isSCell(triplet) ) ++count;
      triplet.setNextSurfel();
    }
    
  return (count==2);
}
//------------------------------------------------------------------------------

CTriplet CTopologicalMap::getOtherSurfel(const CTriplet& ATriplet) const
{
// assert( isDegreTwoLinel(ATriplet) );
  assert( isSCell(ATriplet) );

  CTriplet triplet(ATriplet.getNextSurfel());
  bool found = false;
  while ( !found )
    {
      assert(triplet!=ATriplet);
      if ( isSCell(triplet) ) found = true;
      else triplet.setNextSurfel();
    }

  assert( found );
  // To conserve the orientation of the surface we have to change pointel
  return triplet.getNextPointel();
}
//******************************************************************************

bool CTopologicalMap::isLocalDegreeTwoEdge(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isFree2(ADart) && !isFree3(ADart) );

  return (beta23(ADart)==beta32(ADart));
}
//******************************************************************************

bool CTopologicalMap::isFictiveEdge(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isMarked(ADart, FFictiveMark) ||
	  (isLocalDegreeTwoEdge(ADart) &&
	   isMarked(beta2(ADart), FFictiveMark) &&
	   isMarked(beta3(ADart), FFictiveMark) &&
	   isMarked(beta23(ADart), FFictiveMark)) );
  
  return isMarked(ADart, FFictiveMark);
}
//------------------------------------------------------------------------------

void CTopologicalMap::markFictiveEdge(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( isLocalDegreeTwoEdge(ADart) );

  setMark(       ADart , FFictiveMark);
  setMark(beta2 (ADart), FFictiveMark);
  setMark(beta3 (ADart), FFictiveMark);
  setMark(beta23(ADart), FFictiveMark);
}
//******************************************************************************

bool CTopologicalMap::isEdgeLoop(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isFree2(ADart) );
  
  bool res = false;
  
  CDynamicCoverageVertex it(this, ADart);

  for ( ; it.cont() && !res; ++it )
    {
      if( *it==beta2(ADart) )
	{	  
	  res = true; // Cette arête est une boucle
	}
    }

  return res;
}
//******************************************************************************

bool CTopologicalMap::isVolumeSphere(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isFree2(ADart) );

  return ( beta0(ADart)==beta2(ADart) ) && ( beta1(ADart)==beta2(ADart) );
}
//******************************************************************************

int CTopologicalMap::reelVertexDegree(CDart* ADart)
{
  assert( ADart!=NULL );

  int mark = getNewMark();
  int nbReelEdges = 0;

  CDynamicCoverageVertex it(this, ADart);
  
  for (; it.cont(); ++it)
    {
      if ( !isMarked(*it, mark) )
	{
	  if ( !isFictiveEdge(*it) )
	    {
	      ++nbReelEdges;
	    }

	  markOrbit(*it, ORBIT_EDGE, mark);
	}
    }
  
  for (it.reinit(); it.cont(); ++it)
    {
      if ( isMarked(*it, mark) )
	unmarkOrbit(*it, ORBIT_EDGE, mark);
    }

  freeMark(mark);
  
  return nbReelEdges;
}
//******************************************************************************

int CTopologicalMap::findVertexType( CDart* ADart, CDart** AFirstDart )
{
  assert( ADart!=NULL );

  int  markVertex = getNewMark();
  int  markEdge	  = getNewMark();
  int  res	  = 0;
  int  nbReel	  = 0; // Pour compter le nombre d'arête réelle non boucle
  bool reelLoop	  = false; 
  int  directInfo = getNewDirectInfo();
  
  assert( directInfo!=-1 );

  CBasicDynamicCoverageVertex it(this, ADart, markVertex, directInfo);

  while( it.cont() ) ++it;

  // On parcours le sommet. On s'arrête si on a trouvé et une boucle réelle
  // et une arête non boucle, ou plus de 2 arêtes réelles.
  for ( it.reinit(); it.cont() && nbReel<=2 && (!reelLoop || nbReel==1); ++it )
    {
      if ( !isMarked(*it, markEdge) )
	{
#ifndef NO_FICTIVE_EDGE_SHIFTING
	  if ( !isFictiveEdge(*it) ) // On n'est pas sur une arête fictive
#endif
	    {
	      if ( isMarked(beta2(*it), markVertex) )
		{
		  reelLoop = true; // Ici l'arête réelle est une boucle
		}
	      else
		{
		  if ( nbReel==0 )
		    *AFirstDart = *it; // Cette arête est réelle et non boucle
		}
	      ++nbReel; // On compte le nombre d'arête réelles
	      markOrbit(*it, ORBIT_EDGE, markEdge);
	    }
	}
    }
	  
  if ( reelLoop ) // On a trouvé une boucle non-fictive
    {
      if ( nbReel==1 ) res = 1; // C'est la seule arête fictive
      else res = 3; // Il y a d'autres arêtes fictives incidentes
    }
  else
    {
      if ( nbReel==2 )
	res = 2; // On a exactement 2 arêtes réelles non boucle
      else if ( nbReel!=0 ) res = 3; // cas ==1 ou >2
    }
	  
  // On refait le parcours du sommet pour enlever les marques
  negateMaskMark(markVertex);
  for ( it.reinit(); it.cont(); ++it )
    {
      if ( isMarked(*it, markEdge) )
	unmarkOrbit(*it, ORBIT_EDGE, markEdge);
    }
  negateMaskMark(markVertex);
  
  freeMark(markVertex);
  freeMark(markEdge);
  freeDirectInfo(directInfo);
  
  return res;  
}
//******************************************************************************

CDart* CTopologicalMap::findIncidentEdgeNonLoop(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isFree2(ADart) );

  CDart* res = NULL;
  int markVertex = getNewMark();
  int directInfo = getNewDirectInfo();

  assert( directInfo!=-1 );
  
  CBasicDynamicCoverageVertex it(this, ADart, markVertex, directInfo);

  while( it.cont() ) ++it; // Première boucle pour marquer le sommet
  
  for ( it.reinit(); it.cont() && res==NULL; ++it )
    {
      if ( !isMarked(beta2(*it), markVertex) )
	{
	  res = *it; // Cette arête n'est pas une boucle
	}
    }

  negateMaskMark(markVertex);
  for ( it.reinit(); it.cont(); ++it ); // Pour démarquer le sommet
  negateMaskMark(markVertex);
  
  freeMark(markVertex);
  freeDirectInfo(directInfo);

  return res;
}
//******************************************************************************

bool CTopologicalMap::canRemoveEdgeWithoutDisconnection(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isFree2(ADart) );
  
  if ( beta0(ADart)==beta2(ADart) )
    {
      if ( beta1(ADart)==beta2(ADart) )
	return false; // Le cas de la sphère
      else
	return true;  // Une arête pendante
    }
  else if ( beta1(ADart)==beta2(ADart) )
    return true; // Une arête pendante

  ++nbAppelCanRemoveEdge; // Analyse des parcours

  if (getFace(ADart)->find()==getFace(beta2(ADart))->find())
    return false;
  
  return true;
}
//******************************************************************************

void CTopologicalMap::shiftFictiveEdge(CDart* ADart, CDart* ADart2)
{
  assert( ADart!=NULL && ADart2!=NULL );
  assert( ADart!=ADart2 );
  assert( !isFree0(ADart) && !isFree1(ADart)  );
  assert( !isFree2(ADart) && !isFree2(ADart2) );
  assert( !isFree3(ADart) && !isFree3(ADart2) );
  assert( isFictiveEdge(ADart) );

  // Modification du pointel (cohérence géométrique avec les autres brins)
  assert( isSCell(getTriplet(ADart2)) );
  
  static_cast<CTopologicalDart*>(ADart)->triplet() = getTriplet(ADart2);
  static_cast<CTopologicalDart*>(beta32(ADart))->triplet()
    = getTriplet(beta31(ADart2));
  
  CDart* b1 = beta0 (ADart);
  CDart* b2 = beta21(ADart);
  CDart* b3 = beta0 (ADart2);
  
  linkBeta1(b1,b2);               linkBeta0(beta3(b1),beta3(b2));
  linkBeta1(b3,ADart);            linkBeta0(beta3(b3),beta3(ADart));
  linkBeta1(beta2(ADart),ADart2); linkBeta0(beta23(ADart),beta3(ADart2));
}
//******************************************************************************

void CTopologicalMap::shiftAllFictiveEdgesAroundEdge(CDart* ADart)
{
  assert( ADart!=NULL );
  assert( !isEdgeLoop(ADart) );
  
  CDart* actu    = ADart;
  CDart* toShift = NULL;
  CDart* tmp     = NULL;

  do
    {
      toShift = beta02(actu);
      
      while( isFictiveEdge(toShift) && toShift!=actu )
	{
	  tmp = beta02(toShift);
	  shiftFictiveEdge( toShift, beta1(actu) );
	  toShift = tmp;
	}
      actu  = beta23(actu);    // Puis on passe a l'autre volume.
    }
  while ( actu!=ADart );
}
//******************************************************************************

CDart* CTopologicalMap::createSquareFace(const CTriplet& ATriplet,
					 CRegion* ARegion, CFace* AFace)
{
  CTopologicalDart* first = addMapDart(ATriplet, ARegion, AFace);
  CTopologicalDart* prev  = first;
  CTopologicalDart* actu;

  CTriplet otherTriplet(ATriplet.getNextPointel()); otherTriplet.swapLinels();
  actu = addMapDart(otherTriplet, ARegion, AFace);
  linkBeta1(prev, actu);
  prev = actu;

  otherTriplet.setNextPointel(); otherTriplet.swapLinels();
  actu = addMapDart(otherTriplet, ARegion, AFace);
  linkBeta1(prev, actu);
  prev = actu;

  otherTriplet.setNextPointel(); otherTriplet.swapLinels();
  actu = addMapDart(otherTriplet, ARegion, AFace);
  linkBeta1(prev, actu);
  prev = actu;
  
  linkBeta1(prev, first);

  return first;
}
//******************************************************************************

void CTopologicalMap::sew3TwoSquareFaces(CDart* ADart1, CDart* ADart2)
{
  assert( ADart1!=NULL && ADart2!=NULL );
  assert( beta1111(ADart1)==ADart1 && beta1111(ADart2)==ADart2 );
  
  linkBeta3(ADart1, ADart2);
  linkBeta3(beta1(ADart1), beta0(ADart2));
  linkBeta3(beta11(ADart1), beta00(ADart2));
  linkBeta3(beta0(ADart1), beta1(ADart2));
  
  setFace(ADart1,getFace(ADart2));
  setFace(beta1(ADart1),getFace(beta1(ADart2)));
  setFace(beta11(ADart1),getFace(beta11(ADart2)));
  setFace(beta0(ADart1),getFace(beta0(ADart2)));
}
//******************************************************************************

CDart* CTopologicalMap::createCube(CDart* ALast, CDart* AUp, CDart* ABehind,
				   const CTriplet &ATriplet, CRegion* ARegion)
{
  assert( FKhalimsky!=NULL );
  assert( ALast!=NULL && AUp!=NULL && ABehind!=NULL );

  CTriplet triplet(ATriplet);
  CTriplet otherTriplet(ATriplet.getX()+1,
			ATriplet.getY()+1,
			ATriplet.getZ()+1,
			ZNEG, YNEG);

  CDart* f1=createSquareFace(otherTriplet, ARegion); // Face de droite

  otherTriplet.setLinel(XNEG); otherTriplet.setLinel2(ZNEG);
  CDart* f2=createSquareFace(otherTriplet, ARegion); // Face de devant
 
  triplet.setLinel(YPOS); triplet.setLinel2(ZPOS);
  CDart* f3=createSquareFace(triplet, ARegion, newFace()); // Face de gauche

  triplet.setLinel(ZPOS); triplet.setLinel2(XPOS);
  CDart* f4=createSquareFace(triplet, ARegion, newFace()); // Face de derrière
 
  otherTriplet.setLinel(YNEG); otherTriplet.setLinel2(XNEG);
  CDart* f5=createSquareFace(otherTriplet, ARegion); // Face de dessous
 
  triplet.setLinel(XPOS); triplet.setLinel2(YPOS);
  CDart* f6=createSquareFace(triplet, ARegion, newFace()); // Face de dessus

  // La face "du bas"
  linkBeta2(f5,        beta0(f1));
  linkBeta2(beta1(f5), beta1(f4));
  linkBeta2(beta11(f5),beta11(f3));
  linkBeta2(beta0(f5), f2);
  
  // La face "du haut"
  linkBeta2(f6,        beta0(f4));
  linkBeta2(beta1(f6), beta1(f1));
  linkBeta2(beta11(f6),beta11(f2));
  linkBeta2(beta0(f6), f3);
  
  // Le tour
  linkBeta2(f1,        beta0(f2));
  linkBeta2(f4,        beta0(f3));
  linkBeta2(beta11(f1),beta11(f4));
  linkBeta2(beta1(f2), beta1(f3));

  if ( ALast!=NULL   ) sew3TwoSquareFaces(ALast,  beta0 (f3));
  if ( AUp!=NULL     ) sew3TwoSquareFaces(AUp,    beta11(f6));
  if ( ABehind!=NULL ) sew3TwoSquareFaces(ABehind,beta11(f4));

#ifndef NO_INCREMENTAL_EULER
  // Mise à jour des caractéristiques topologiques.
  ARegion->incNbVertices(8);
  ARegion->incNbEdges   (12);
  ARegion->incNbFaces   (6);
#endif // NO_INCREMENTAL_EULER
  
  return beta11(f1);
}
//******************************************************************************

CDart* CTopologicalMap::precodeL8( CDart* ALast, CDart* AUp, CDart* ABehind,
				   const CTriplet& ATriplet )
{
  assert( ALast!=NULL && AUp!=NULL && ABehind!=NULL );
  assert( !isFree2(ALast) && !isFree2(AUp) && !isFree2(ABehind) );
  assert( beta2 (ALast)  ==beta00(ABehind) );
  assert( beta02(ABehind)==beta00(AUp) );
  assert( beta02(ALast)  ==beta0 (AUp) );

  assert( beta1111(ALast)  ==ALast   );
  assert( beta1111(AUp)    ==AUp     );
  assert( beta1111(ABehind)==ABehind );
  
  CDart* t1 = beta2 (AUp);
  CDart* t2 = beta12(AUp);
  CDart* t3 = beta12(ALast);
  CDart* t4 = beta12(ABehind);
  CDart* actu = NULL;

//   unlinkBeta2( beta0(ALast) );
//   unlinkBeta2( beta1(ALast) );
//   unlinkBeta2( beta0(ABehind) );
//   unlinkBeta2( beta1(ABehind) );
//   unlinkBeta2( beta0(AUp) );
//   unlinkBeta2( beta11(AUp) );
  
  // 1) Tout d'abord la topologie.
  
  // Face de gauche => Face de devant
  linkBeta2( beta0(ALast), t1);
  linkBeta2( beta1(ALast), AUp);
  
  // Face de derrière => Face de droite
  linkBeta2( beta0(ABehind), t2);
  linkBeta2( beta1(ABehind), beta1(AUp));
  
  // Face du haut => Face du bas
  linkBeta2( beta0 (AUp), t3);
  linkBeta2( beta11(AUp), t4);

  // 2) Puis la géométrie.

  CTriplet triplet( ATriplet.getX()+1, ATriplet.getY()+1, ATriplet.getZ()+1,
		    ZNEG, YNEG );

  // Face de droite
  actu = beta11(ABehind);

  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);
  
  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );

  // Face de devant  
  actu = beta1(ALast);
  triplet.setTriplet(ATriplet.getX()+1, ATriplet.getY()+1, ATriplet.getZ()+1,
		     XNEG, ZNEG);
  
  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);
  
  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );
  
  // Face de dessous
  actu = beta1(AUp);
  triplet.setTriplet(ATriplet.getX()+1, ATriplet.getY()+1, ATriplet.getZ()+1,
		     YNEG, XNEG);

  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);
  
  setTriplet( actu, triplet );
  triplet.setNextPointel(); triplet.swapLinels(); actu=beta1(actu);

  setTriplet( actu, triplet );

  return ABehind;
}
//******************************************************************************

CVertex CTopologicalMap::barycenter(CDart * ADart, TOrbit AOrbit)
{
  assert(ADart!=NULL);

  CVertex bary(ORIGIN);
  int n = 0;
  int treated = getNewMark();

  CCoverage * cov = getDynamicCoverage(ADart, AOrbit);

  assert(cov!=NULL);

  for (; cov->cont(); ++(*cov))
    if ( !isMarked(**cov, treated) )
      {
	bary += getTriplet(**cov);	
	++n;
	markOrbit(**cov, ORBIT_VERTEX, treated);
      }

  // Démarquage:
  for ( cov->reinit(); cov->cont(); ++(*cov))
    {
      if ( isMarked(**cov, treated) )
	unmarkOrbit(**cov, ORBIT_VERTEX, treated);
    }
  
  delete cov;

  freeMark(treated);

  return n==0 ? ORIGIN : bary/n;
}
//******************************************************************************

void CTopologicalMap::edgeContraction( CDart* ADart )
{
  assert( ADart!=NULL );
  assert( !isFree0(ADart) && !isFree1(ADart) );
  assert( !isFree2(ADart) && !isFree3(ADart) );
  assert( isLocalDegreeTwoEdge(ADart) );
  
  // On marque les brins qui vont être supprimés.
  delMapDart(beta23(ADart));
  delMapDart(beta3 (ADart));
  delMapDart(beta2 (ADart));
  delMapDart(       ADart );

  // On fait attention aux cas ou on va supprimer un représentant.
  if ( isMarked(getRegion(ADart)->getRepresentativeDart(),FMarkToDelete) )
    {
      CDart* newRepresentant =
	( !isMarked(beta0(ADart),FMarkToDelete)  ? beta0(ADart) :
	  !isMarked(beta1(ADart),FMarkToDelete)  ? beta1(ADart) : NULL );
      assert( newRepresentant!=NULL );
	  
      getRegion(ADart)->setRepresentativeDart(newRepresentant);
    }

  if ( isMarked(getRegion(beta3(ADart))->
		getRepresentativeDart(),FMarkToDelete) )
    {
      CDart* newRepresentant =
	( !isMarked(beta30(ADart),FMarkToDelete)  ? beta30(ADart) :
	  !isMarked(beta31(ADart),FMarkToDelete)  ? beta31(ADart) : NULL );
      assert( newRepresentant!=NULL );
	  
      getRegion(beta3(ADart))->setRepresentativeDart(newRepresentant);
    }

  // Modification des pointels (cohérence géométrique avec les autres brins).
  CTriplet t1 = getTriplet(ADart);
  CTriplet t2 = getTriplet(beta23(ADart));
  assert( t1.samePointel(t2) );
  CDart* current = beta1(ADart);
  do
    {
      if ( !isMarked(current, FMarkToDelete) )
 	{
 	  assert( isFictiveEdge(current) );
 	  assert( !isFree2(current) && !isFree3(beta2(current)) );
 	  static_cast<CTopologicalDart*>(current)->triplet()	     = t1;
 	  static_cast<CTopologicalDart*>(beta23(current))->triplet() = t2;
 	}
      assert( !isFree0(current) && !isFree2(beta0(current)) );
      current = beta02(current);
    }
  while( current!=beta1(ADart) );
  
  // Contraction effective.
  linkBeta1( beta320(ADart), beta321(ADart) );  
  linkBeta1( beta30 (ADart), beta31 (ADart) );
  linkBeta1( beta20 (ADart), beta21 (ADart) );
  linkBeta1( beta0  (ADart), beta1  (ADart) );

  assert( !isMarked(getRegion(ADart)->getRepresentativeDart(),FMarkToDelete) );
  assert( !isMarked(getRegion(beta3(ADart))->
		    getRepresentativeDart(),FMarkToDelete) );
}
//******************************************************************************

CCoordinate CTopologicalMap::getVoxelFromTriplet(const CTriplet & ATriplet)const
{
  CTriplet tripletNormalise = normaliseTripletSurfel(ATriplet);
  CCoordinate AVertex(tripletNormalise);
  
  switch(tripletNormalise.getLinel())
    {
    case XPOS:
      switch(tripletNormalise.getLinel2())
	{
	  // case YNEG: AVertex.setZ(AVertex.getZ()-1); break;
	case YPOS: break;
	case ZPOS: AVertex.setY(AVertex.getY()-1); break;
	default: std::cout<<"PROBLEME";exit(-1);
	}
      break;
    case YPOS:
      switch(tripletNormalise.getLinel2())
	{
	case XPOS: AVertex.setZ(AVertex.getZ()-1); break;
	case ZPOS: break;
	default: std::cout<<"PROBLEME";exit(-1);
	  // case ZNEG: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case ZPOS:
      switch(tripletNormalise.getLinel2())
	{
	  // case XNEG: AVertex.setY(AVertex.getY()-1); break;
	case XPOS: break;
	case YPOS: AVertex.setX(AVertex.getX()-1); break;
	default: std::cout<<"PROBLEME";exit(-1);
	}
      break;
    default:
      std::cout<<"PROBLEME";exit(-1);
    }
  return AVertex;
}
//------------------------------------------------------------------------------
const CCoordinate CTopologicalMap::getVoxelIn( const CTriplet & ATriplet ) const
{
  assert( normaliseTripletSurfel( ATriplet ) == ATriplet );
  CCoordinate AVertex(ATriplet.getX(),ATriplet.getY(),ATriplet.getZ());
  switch(ATriplet.getLinel())
    {
    case XPOS:
      switch(ATriplet.getLinel2())
	{
	case YNEG: AVertex.setZ(AVertex.getZ()-1); break;
	case ZPOS: AVertex.setY(AVertex.getY()-1); break;
	}
      break;
    case XNEG:
      AVertex.setX(AVertex.getX()-1);
      switch(ATriplet.getLinel2())
	{
	case YPOS: AVertex.setZ(AVertex.getZ()-1); break;
	case ZNEG: AVertex.setY(AVertex.getY()-1); break;
	}
      break;
    case YPOS:
      switch(ATriplet.getLinel2())
	{
	case XPOS: AVertex.setZ(AVertex.getZ()-1); break;
	case ZNEG: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case YNEG:
      AVertex.setY(AVertex.getY()-1);
      switch(ATriplet.getLinel2())
	{
	case XNEG: AVertex.setZ(AVertex.getZ()-1); break;
	case ZPOS: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case ZPOS:
      switch(ATriplet.getLinel2())
	{
	case XNEG: AVertex.setY(AVertex.getY()-1); break;
	case YPOS: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case ZNEG:
      AVertex.setZ(AVertex.getZ()-1);
      switch(ATriplet.getLinel2())
	{
	case XPOS: AVertex.setY(AVertex.getY()-1); break;
	case YNEG: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    }
  return AVertex;
}
//------------------------------------------------------------------------------
const CCoordinate CTopologicalMap::getVoxelOut( const CTriplet & ATriplet ) 
  const
{
  assert( normaliseTripletSurfel( ATriplet ) == ATriplet );
  CCoordinate AVertex(ATriplet.getX(),ATriplet.getY(),ATriplet.getZ());
  switch(ATriplet.getLinel())
    {
    case XPOS:
      switch(ATriplet.getLinel2())
	{
	case YPOS: AVertex.setZ(AVertex.getZ()-1); break;
	case ZNEG: AVertex.setY(AVertex.getY()-1); break;
	}
      break;
    case XNEG:
      AVertex.setX(AVertex.getX()-1);
      switch(ATriplet.getLinel2())
	{
	case YNEG: AVertex.setZ(AVertex.getZ()-1); break;
	case ZPOS: AVertex.setY(AVertex.getY()-1); break;
	}
      break;
    case YPOS:
      switch(ATriplet.getLinel2())
	{
	case XNEG: AVertex.setZ(AVertex.getZ()-1); break;
	case ZPOS: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case YNEG:
      AVertex.setY(AVertex.getY()-1);
      switch(ATriplet.getLinel2())
	{
	case XPOS: AVertex.setZ(AVertex.getZ()-1); break;
	case ZNEG: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case ZPOS:
      switch(ATriplet.getLinel2())
	{
	case XPOS: AVertex.setY(AVertex.getY()-1); break;
	case YNEG: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    case ZNEG:
      AVertex.setZ(AVertex.getZ()-1);
      switch(ATriplet.getLinel2())
	{
	case XNEG: AVertex.setY(AVertex.getY()-1); break;
	case YPOS: AVertex.setX(AVertex.getX()-1); break;
	}
      break;
    }
  return AVertex;
}
//******************************************************************************
const CCoordinate CTopologicalMap::getVoxelIn( CDart * ADart ) const
{ return getVoxelIn( normaliseTripletSurfel( getTriplet( ADart ) ) ); }
//------------------------------------------------------------------------------
const CCoordinate CTopologicalMap::getVoxelOut( CDart * ADart ) const
{ return getVoxelOut( normaliseTripletSurfel( getTriplet( ADart ) ) ); }
//*****************************************************************************
void CTopologicalMap::cleanMap()
{
  for( CDynamicCoverageAll it( this ) ; it.cont() ; it++ )
    {
      setRegion( *it, getRegion(*it) );
      setFace  ( *it, getFace(*it)   );
    }
  cleanAllRegions();
  cleanAllFaces();
}
//******************************************************************************
CTopologicalMap::CTopologicalMap() :
  CMapGeneric       (ASizeDartArray),
  FFirstRegion      (NULL),
  FFirstFace        (NULL),
  FImage            (NULL),
  FKhalimsky        (NULL)
{
  empty();
}
//------------------------------------------------------------------------------
CTopologicalMap::CTopologicalMap(CImage3D* AImage) :
  CMapGeneric       (ASizeDartArray),
  FFirstRegion      (NULL),
  FFirstFace        (NULL),
  FImage            (NULL),
  FKhalimsky        (NULL)
{
  empty();
  
  // Extraction de la carte à partir de l'image
  extractTopologicalMap(AImage);
}
//******************************************************************************
CTopologicalMap::~CTopologicalMap()
{
  empty();
  freeMark(FFictiveMark);
  // delete FImage;
  delete FKhalimsky;
}
//******************************************************************************

//******************************************************************************
} // namespace Map3d
//******************************************************************************
