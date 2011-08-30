//******************************************************************************
#ifndef __FACE_HH__
#define __FACE_HH__
//******************************************************************************
#include <cassert>
#include <cstdlib>
#include <algorithm>
//******************************************************************************
namespace Map3d
{
//******************************************************************************
class CFace
{
public:
//******************************************************************************

CFace::CFace() :
  FFather   (this),
  FHeight   (0),
  FExternalContrast(0.),
  FSurfelsCount(1),
  FPrev     (NULL),
  FNext     (NULL)
{}
//------------------------------------------------------------------------------

CFace::CFace(CFace* AFace) :
  FFather   (AFace),
  FHeight   (0),
  FExternalContrast(0.),
  FSurfelsCount(1),
  FPrev     (NULL),
  FNext     (NULL)
{}
//******************************************************************************

CFace* CFace::getNext() const
{ return FNext; }
//------------------------------------------------------------------------------

CFace* CFace::getPrev() const
{ return FPrev; }
//------------------------------------------------------------------------------

void CFace::setNext(CFace* AFace)
{ FNext = AFace; }
//------------------------------------------------------------------------------

void CFace::setPrev(CFace* AFace)
{ FPrev = AFace; }
//------------------------------------------------------------------------------

bool CFace::isUFTreeRoot()
{ return FFather == this; }
//******************************************************************************

double CFace::getExternalContrast()
{ return FExternalContrast; }
//------------------------------------------------------------------------------

void CFace::setExternalContrast( double AValue )
{ FExternalContrast = AValue; }
//******************************************************************************

unsigned int CFace::getSurfelsCount()
{ return FSurfelsCount; }
//------------------------------------------------------------------------------

void CFace::setSurfelsCount( unsigned int ASurfelsCount )
{ FSurfelsCount = ASurfelsCount; }
//******************************************************************************

CFace* CFace::find()
{
  CFace* res = FFather;
  while ( res->FFather!=res )
    {
      ++nbEdgeParcouru; // On compte le nombre d'éléments parcouru.
      res = res->FFather;
    }

  CFace* actu = this;
  CFace* next = NULL;
  while ( actu!=res )
    {
      next = actu->FFather;
      actu->FFather = res;
      actu = next;
    }
  
  return res;
}
//******************************************************************************

void CFace::merge( CFace* AFace )
{
  CFace* face1 = find();
  CFace* face2 = AFace->find();
  double newExternalContrast = std::min(face1->getExternalContrast(),face2->getExternalContrast());
  unsigned int newSurfelsCount = face1->getSurfelsCount() + face2->getSurfelsCount();

  if ( face1==face2 ) return; // Les 2 arbres sont déjà dans la même classe.

  // Lors de la fusion, on met le plus petit sous le plus grand.
  if ( face1->FHeight < face2->FHeight )
    face1->FFather=face2;
  else
    {
      face2->FFather=face1;
      if ( face1->FHeight == face2->FHeight )
	++face1->FHeight; // Ici la hauteur de l'arbre 1 augmente.
    }
  
  // Mise à jour du paramètre de contraste externe
  find()->setExternalContrast(newExternalContrast);
  find()->setSurfelsCount(newSurfelsCount);
}
  
private:
  /// Père dans l'arbre union-find des faces
  CFace*       FFather;

  /// Hauteur de l'arbre union-find sous jacent
  unsigned int FHeight;

  /// Le contraste externe de la face
  double FExternalContrast;

  /// Nombre de surfels appartenant à la face
  unsigned int FSurfelsCount;
  
  /// Face précédente dans la liste chaînée des faces.
  CFace* FPrev;
  /// Face suivante dans la liste chaînée des faces.
  CFace* FNext;
};
//******************************************************************************
} // namespace Map3d
//******************************************************************************
#endif // __FACE_HH__
//******************************************************************************
