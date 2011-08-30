//******************************************************************************
#ifndef TOPOLOGICAL_DART_HH
#define TOPOLOGICAL_DART_HH
//******************************************************************************
#include "Dart.h"
#include "triplet.hh"
#include "region.hh"
#include "face.hh"
//******************************************************************************

namespace Map3d
{
  class CRegion;
  class CFace;

  template<typename Refs>
  class CTopologicalDart: public Dart<3,Refs>
  {
  public:
    CTopologicalDart::CTopologicalDart() :
      CDart   (),
      FTriplet(),
      FRegion (NULL),
      FFace   (NULL)
    {}

    void CTopologicalDart::init(const CTriplet & ATriplet, CRegion* ARegion)
    {
      FTriplet = ATriplet;
      FRegion  = ARegion;
    }

    void CTopologicalDart::init(const CTriplet & ATriplet, CRegion* ARegion,
				CFace* AFace)
    {
      FTriplet = ATriplet;
      FRegion  = ARegion;
      FFace    = AFace;
    }  

    CTriplet& CTopologicalDart::triplet()
    { return FTriplet; }

    CRegion* CTopologicalDart::getRegion() const
    { return FRegion->find(); }

    void CTopologicalDart::setRegion(CRegion* ARegion)
    {
      assert( ARegion!=NULL );
  
      FRegion=ARegion->find();
    }

    CFace* CTopologicalDart::getFace()
    { return FFace->find(); }

    void CTopologicalDart::setFace(CFace* AFace)
    {
      assert( AFace!=NULL );
  
      FFace=AFace->find();
    }

  protected:
    // @name Attributs
    //@{

    /// Le triplet "géométrique" associé au brin.
    CTriplet FTriplet;

    /// La région d'appartenance du brin.
    CRegion* FRegion;
    
    /// Un pointeur vers l'arbre union-find associé à la face contenant ce brin
    CFace*   FFace;
    
    //@}
  };

} // namespace GMap3d
//******************************************************************************
#endif // TOPOLOGICAL_DART_HH
//******************************************************************************
