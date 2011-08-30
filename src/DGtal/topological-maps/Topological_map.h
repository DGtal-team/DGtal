//******************************************************************************
#ifndef TOPOLOGICAL_MAP_HH
#define TOPOLOGICAL_MAP_HH
//******************************************************************************
#include <set>
#include <bitset>
#include <list>
#include "Combinatorial_map.h"
#include "image3D.hh"
#include "triplet.hh"
#include "vertex.hh"
#include "khalimsky.hh"
#include "limited-stack.hh"
#include "pair.hh"
#include "stack.hh"
#include "darts-stack.hh"
#include "coordinate.hh"
//#include "volume.hh"
//******************************************************************************

namespace Map3d
{
  class CTopologicalDart;
  class CRegion;  
  class CFace;

  struct Topological_dart_items
  {
    template < class Refs >
    struct Dart_wrapper
    {
      typedef CTopologicalDart<Refs> Dart;
      typedef CGAL::cpp0x::tuple<> Attributes;
    };
  };

  
  /** Définit la classe CTopologicalMap qui représente une carte topologique
   *  3D associée à une géométrie de type "matrice de Khalimsky".
   *  Les opérations implémentées dans cette classe sont
   *  topologiques et géométriques.
   *
   */
  class CTopologicalMap: public CGAL::Combinatorial_map<3, Topological_dart_items>
  {
  public:
    /// @name Constructeur et destructeur
    //@{

    /**
     *  Constructeur par défaut, qui ne crée aucune structure cad que tout
     *  les pointeurs sont vides.
     *  @param ASizeDartArray La taille de chaque tableau de brins
     */
    CTopologicalMap(unsigned int ASizeDartArray = 5000);
    
    /**
     *  Constructeur avec une image en paramètre.
     *  La carte créée est vide.
     *  La matrice de Khalimsky associée est créé à la taille de l'image,
     *  puis l'image est balayée afin de créer la carte et d'initialiser
     *  la matrice.
     *
     * @param AImage Image3D extraite lors de la création de la carte.
     * @param ASizeDartArray La taille de chaque tableau de brins
     */
    CTopologicalMap(CImage3D* AImage);

    /**
     *  Destruction de l'instance.
     *  Tous les brins de la carte ainsi que leurs plongements sont détruits.
     *  L'image, la matrice de Khalimsky et l'arbre d'inclusion également.
     */
    ~CTopologicalMap();

    //@}

    /**
     *  Crée une nouvelle instance de CTopologicalDart, dont le plongement est
     *  donné par ATriplet.
     *
     *  @param  ATriplet Un triplet
     *  @param  ARegion Un pointeur vers une région
     *  @param  AFace Un pointeur vers une face
     *  @return Un nouveau brin de type CTopologicalDart
     */
    virtual CTopologicalDart* addMapDart(const CTriplet & ADoublet,
					 CRegion* ARegion, CFace* AFace);

    /** 
     *  Méthode permettant de vider la carte ainsi que de détruire les
     *  structures annexes. La méthode removeAllDarts est appelée (pour
     *  garder les marques utilisées). Puis l'image, la matrice de
     *  Khalimsky et l'arbre d'inclusion sont détruit.
     */
    void empty();

    //@}
    
    /// @name Degré et fusions
    //@{

    /**
     *  @param  ADart Un brin de la carte
     *  @return Vrai si l'arête incidente à ADart est localement de degrée deux.
     */
    bool isLocalDegreeTwoEdge(CDart* ADart);
    
    /**
     *  @param  ADart Un brin de la carte
     *  @return Vrai ssi l'arête incidente à ADart est une boucle
     *          (c'est à dire incidente 2 fois au même sommet).
     */
    bool isEdgeLoop(CDart* ADart);
    
    /**
     *  @param  ADart Un brin de la carte
     *  @return Vrai ssi le volume incident à ADart est une sphère
     *          (c'est à dire composé de 2 sommets, une arête et une face).
     *          Notez qu'une région étant une sphère est une région isolée.
     */
    bool isVolumeSphere(CDart* ADart);

    /**
     *  @param  ADart Un brin de la carte
     *  @return Vrai ssi l'arête incidente à ADart est une arête fictive.
     */
    bool isFictiveEdge(CDart* ADart);
    
    /**
     *  @param  ADart Un brin de la carte
     *  Marque tout les brins de l'arête incidente à ADart comme fictif.
     */
    void markFictiveEdge(CDart* ADart);
    
    /**
     *  @param  ADart Un brin de la carte
     *  @return Vrai ssi on peut supprimer l'arête incidente à ADart sans
     *          déconnecter la carte en plusieurs composantes connexes.
     */
    bool canRemoveEdgeWithoutDisconnection(CDart* ADart);
    
    /**
     *  @param  ADart Un brin de la carte
     *  @return Le nombre d'arêtes réelles incidentes au sommet touchant ADart.
     */
    int reelVertexDegree(CDart* ADart);
    
    /** 
     * Fournit le type du sommet incident à ADart.
     *
     *  @param ADart Un brin de la carte
     *  @param AResDart  l'adresse d'un brin : paramètre retour
     *  @return 0 ssi le sommet est fictif (cas de degré réel 0) ;
     *          1 ssi la seule arête réelle incidente au sommet est
     *            une boucle ;
     *          2 ssi le sommet est de degré réel 2 et que les 2 arêtes réelles
     *            ne sont pas des boucles. Dans ce cas uniquement, *AResDart
     *            contiendra un des brin de ces arêtes incident au sommet ;
     *          3 ssi soit il y a deux arêtes réelles incidente au sommet,
     *            et au moins une est une boucle, soit le degré réel est > 2.
     */
    int findVertexType(CDart* ADart, CDart** AResDart);
    
    /**
     * Renvoit un brin sur une arête incidente qui n'est pas une boucle.
     *
     *  @param ADart Un brin de la carte
     *  @return Un brin d'une arête non-boucle incidente au sommet
     *             désigné par ADArt s'il en existe une
     *          NULL Sinon
     */
    CDart* findIncidentEdgeNonLoop(CDart* ADart);

    /**
     *  Pousse l'arête fictive incidente au brin ADart sur le "prochain"
     *  sommet (le sommet incident à ADart2)
     *
     *  @param  ADart  Un brin de l'arête à pousser.
     *  @param  ADart2 Un brin désignant l'endroit où l'arête va être mise.
     */
    void shiftFictiveEdge(CDart* ADart, CDart* ADart2);
      
    /**
     *  Pousse toute les arêtes fictives incidentes au sommet et à l'arête
     *  désignés par le brin ADart, sauf l'arête incidente à ADart
     *  (dans le cas ou elle est fictive puisque si elle est réelle
     *   la question ne se pose pas). Les arêtes fictives sont décallées
     *  sur le second sommet de l'arête incidente à ADart.
     *  
     *  @pre l'arête incidente à ADart ne doit pas être une boucle
     *  @param  ADart Un brin appartenant au sommet "à nettoyer".
     */
    void shiftAllFictiveEdgesAroundEdge(CDart* ADart);
      
    /**
     *  Supprime l'arête pendande incidente à ADart en "recollant" les
     *  deux arêtes incidente au sommet du coté non pendant.
     *  Cette fonction utilise le fait que les brins ne sont pas supprimés
     *  directement mais marqués (cf. l'allocateur dans CMapBasic).
     *
     *  @pre Le sommet incident à ADart est de degré 1 (arête pendante)
     *  @param  ADart Un brin de la carte du coté du sommet de degré 1
     */
    void danglinEdgeRemoval(CDart* ADart);

    /**  Contracte l'arête incidente à ADart en "recollant" les deux sommets
     *  autour de cette arête.
     *  Cette fonction utilise le fait que les brins ne sont pas supprimés
     *  directement mais marqués (cf. l'allocateur dans CMapBasic).
     *
     *  @param  ADart       Un brin de la carte
     */
    void edgeContraction( CDart* ADart );

    //@}

    /// @return la marque utilisée pour les arêtes fictives
    int getFictiveMark() const;
    
    /// @return la matrice de Khalimsky
    CKhalimsky* getKhalimsky() { return FKhalimsky; }

    /// @return la matrice image 3D
    CImage3D* getImage() { return FImage; }

    /// @return le nombre d'octets utilisé par la carte.
    unsigned long int getMemoryForMap() const;

    /// @return le nombre d'octets utilisé par l'arbre d'inclusion.
    unsigned long int getMemoryForInclusionTree() const;

    /// @return le nombre d'octets utilisé par les arbres union-find
    unsigned long int getMemoryForFace() const;

    /// @return le nombre d'octets utilisé par la matrice de Khalimsky.
    unsigned long int getMemoryForKhalimsky() const;

    /// @return le nombre d'octets utilisé par l'image.
    unsigned long int getMemoryForImage() const;

    /// @return le nombre d'octets total utilisé.
    unsigned long int getTotalMemory() const;

    /**
     *  Retourne le triplet associé au brin.
     *  @param ADart le brin
     *  @return Le triplet.
     */
    CTriplet & getTriplet( CDart * ADart ) const;

    /**
     *  Affecte le triplet associé au brin.
     *  @param ADart    le brin
     *  @param ATriplet le nouveau triplet
     */
    void setTriplet( CDart * ADart, const CTriplet & ATriplet );

    /**
     *  Test pour savoir si un brin est représentant de sa région.
     *  @param ADart le brin
     *  @return Vrai ou faux selon si le brin est représentant ou non.
     */
    bool isRepresentative(CDart* ADart);

    /// @name Accesseurs sur les cellules
    //@{
    bool isPCell (const CTriplet& ATriplet) const;
    bool isLCell (const CTriplet& ATriplet) const;
    bool isSCell (const CTriplet& ATriplet) const;
    bool isL2Cell(const CTriplet& ATriplet) const;
    
    bool isFictivePCell(const CTriplet& ATriplet) const;
    bool isFictiveLCell(const CTriplet& ATriplet) const;

    CTriplet normaliseTripletPointel(const CTriplet& ATriplet) const;
    CTriplet normaliseTripletLinel  (const CTriplet& ATriplet) const;
    CTriplet normaliseTripletSurfel (const CTriplet& ATriplet) const;

    bool isSurfelMarked  (const CTriplet& ATriplet) const;
    void setSurfelMark   (const CTriplet& ATriplet, bool AOn);
    void markSurfel      (const CTriplet& ATriplet);
    void unmarkSurfel    (const CTriplet& ATriplet);
    void unmarkAllSurfels();
    bool isWholeSurfelsMarked() const;
    bool isWholeSurfelsUnmarked() const;
    void negateSurfelMaskMark();
    //@}
    
    /// @name Méthodes pour explorer une surface.
    //@{
    /// Retourne vrai ssi le pointel de ATriplet est de degré 2
    ///   (cad incident exactement à 2 lignels).
    bool isDegreTwoPointel(const CTriplet& ATriplet) const;
    
    /**
     *  Retourne l'autre lignel incident au pointel de ATriplet.
     *
     *  @pre le pointel doit être de degré 2 (cf ci-dessus).
     *  @return l'autre lignel désigné par un CTriplet.
     */
    CTriplet getOtherLinel(const CTriplet& ATriplet) const;

    /// Retourne vrai ssi le lignel1 de ATriplet est de degré 2
    ///   (cad incident exactement à 2 surfels).
    bool isDegreTwoLinel(const CTriplet& ATriplet) const;
    
    /**
     *  Retourne l'autre surfel incident au lignel1 de ATriplet.
     *
     *  @pre le lignel1 doit être de degré 2 (cf ci-dessus).
     *  @return l'autre surfel désigné par un CTriplet.
     */
    CTriplet getOtherSurfel(const CTriplet& ATriplet) const;
    //@}
    
  private:
    /**
     *  Pour marquer le sommet incident à ADart uniquement à l'intérieur
     *  du volume avec la marque AMark.
     *
     *  @param ADart un brin désignant le sommet à marquer
     *  @param AMark la marque
     */
    void markVertexInVolume( CDart* ADart, int AMark );

    /**
     *  Pour démarquer le sommet incident à ADart uniquement à l'intérieur
     *  du volume avec la marque AMark.
     *
     *  @param ADart un brin désignant le sommet à démarquer
     *  @param AMark la marque
     */
    void unmarkVertexInVolume( CDart* ADart, int AMark );

    
    ///@name Création d'objet de base.
    //@{
    
    /**
     *  Crée une face carré plongée (chaque brin correspond à un
     *    triplet dans la matrice).
     *
     *  @param  ATriplet le triplet du premier brin de la face.
     *  @param  ARegion  la région d'appartenance de la face.
     *  @param  ATree    un arbre union-find.
     *  @return Le premier brin de la face.
     */
    CDart* createSquareFace(const CTriplet& ATriplet, CRegion* ARegion,
			    CFace* ATree=NULL);

    /**
     *  3-couds 2 faces carrés
     *
     *  @param  ADart1 le brin de la première face
     *  @param  ADart2 le brin de la deuxième face
     */
    void sew3TwoSquareFaces(CDart* ADart1, CDart* ADart2);

    /**
     *  Crée un cube plongé et le cous par beta3 aux brins "ALast"
     *  "AUp" et "ABehind".
     *
     *  @param ALast    un brin last
     *  @param AUp      un brin up
     *  @param ABehind  un brin behind
     *  @param ATriplet un triplet désignant le pointel en haut
     *         à gauche et derrière
     *  @param  ARegion la région d'appartenance du cube.
     *  @return Un brin du cube.
     */
    CDart* createCube(CDart* ALast, CDart* AUp, CDart* ABehind,
		      const CTriplet& ATriplet, CRegion* ARegion);
    
    /**
     *  Précode L8
     *
     *  @param ALast    un brin last
     *  @param AUp      un brin up
     *  @param ABehind  un brin behind
     *  @param ATriplet un triplet désignant le pointel en haut
     *         à gauche et derrière
     *  @return Le prochain last 
     */
    CDart* precodeL8(CDart* ALast, CDart* AUp, CDart* ABehind,
		     const CTriplet& ATriplet);
    
    //@}

  public:
    /**
     *  Extrait la carte topologique de l'image passé en paramètre.
     *
     *  @param AImage un pointeur vers l'image à extraire.
     *  @param AThreshold Seuil de présegmentation de l'image.
     *  @param AStep  pour extraire l'image en pas à pas si vrai
     */
    CDart* extractTopologicalMap( CImage3D* AImage,
				  TColor    AThreshold = 0,
				  bool      AStep = false );

    /**
     *  Extrait un voxel supplémentaire de l'image passé en paramètre.
     *
     *  @param AImage un pointeur vers l'image à extraire.
     *  @param ALast  le dernier brin, résultat de l'appel précédent à
     *                extractTopologicalMapOneStep
     *  @param Ax  les coordonnées 3D du voxel courant
     *  @param Ay  les coordonnées 3D du voxel courant
     *  @param Az  les coordonnées 3D du voxel courant
     *  @param AMarkTreated une marque pour les sommets déjà traités
     *  @param ADart une référence vers un brin à surveiller, s'il est
     *         supprimé il est mis à NULL
     *  @param AThreshold Seuil de présegmentation (defaut : 0)
     *  @return le prochain last
     */
    CDart* extractTopologicalMapOneStep( CImage3D* AImage,
					 CDart*    ALast,
					 unsigned int Ax,
					 unsigned int Ay,
					 unsigned int Az,
					 int AMarkTreated,
					 CDart* & ADart,
					 TColor    AThreshold = 0
					 );

  private:
    /**
     *  Boucle principale pour l'extraction de la carte topologique. Equivalent
     *  d'appeller pour chaque voxel la fonction extractTopologicalMapOneStep,
     *  mais on a recopier pour acceller les temps d'exécutions.
     */
    CDart* extractTopologicalMapMainLoop( CImage3D* AImage, 
					  CDart* ALast,
					  TColor    AThreshold = 0
					  );

    /**
     *  Crée le bord initial supérieur avant l'extraction d'une image 3D.
     *
     *  @param ALargX largeur en X de l'image
     *  @param ALargY largeur en Y de l'image
     *  @return Le brin en haut à gauche incident au premier pointel
     *          (cf. thèse page 166).
     */
    CDart* makeBorder( int ALargX, int ALargY );

    /**
     *  Détruit le bord initial supérieur après l'extraction d'une image 3D.
     *  (ce bord est la surface incidente à ADart).
     *
     *  @param ADart le dernier last qui appartient au bord à détruire.
     */
    void destroyBorder( CDart* ADart );

    /**
     *  Calcule l'arbre d'inclusion des régions. La liste AList est la liste
     *  des régions rencontrées dans l'ordre du balayage
     *  (haut-bas, derrière-devant et gauche-droite). Attention, cette méthode
     *  fonctionne uniquement pour les images bien formée.
     */
    void computeInclusionTree();
    
    /** Compte le nombre de sommets incident à la face carré désignée par ADart. 
     * @warning Ce code ne fonctionne que si la face à 4 arêtes ou moins (optimal 
     * pour 4 arêtes). La fonction générique fonctionant sur des faces ayant autant 
     * d'arêtes que l'on veut est countNbVerticesIncidentToFace.
     *
     * @see countNbVerticesIncidentToFace
     *
     * @param  ADart brin désignant la face
     * @param  AMarkVertex marque utilisée pour marquer les sommets déjà traités
     * @return le nombre de sommets incident à la face en ignorant les brins
     *         supprimés ou déjà traités (ie marqués par AMarkVertex)
     */
    int countNbVerticesIncidentToFace4( CDart* ADart, int AMarkVertex );

    
    /**
     *  Calcule le brin "up" à partir du brin "last".
     *
     *  @param  ADart le brin "last".
     *  @return Le brin "up".
     */
    CDart* computeUpFromLast(CDart* ADart);
    
    /**
     *  Calcule le brin "behind" à partir du brin "last".
     *
     *  @param  ADart le brin "last".
     *  @return Le brin "behind".
     */
    CDart* computeBehindFromLast(CDart* ADart);
    
    /**
     *  "Traite" l'arête AnEdge : teste si elle est de degré deux et
     *   si elle peut être supprimée sans déconnexion. Si oui, la supprime
     *   ainsi que le lignel correspondant dans la matrice de Khalimsky.
     *
     *  @param AnEdge   Les brins de l'arête à traiter.
     *  @param ATriplet Le triplet associé à l'arête.
     */
    void processEdge( CLimitedStack<CDart*>& AnEdge,
		      CTriplet& ATriplet );

    /**
     *  "Traite" le sommet AVertex : teste si il est de degré deux.
     *   Si oui, le sommet est supprimé (en décalent éventuellement les
     *   arêtes fictives incidentes). ainsi que le pointel correspondant
     *   dans la matrice de Khalimsky.
     *
     *  @param AVertex  Les brins du sommet à traiter.
     *  @param ATriplet Le triplet associé à l'arête.
     *  @param AMarkTreated La marque utilisée pour marquer les sommets
     *                      de degré réel différent de zéro (utile pour la
     *                      méthode removeAllReelDegreeZeroVertex).
     */
    void processVertex( CLimitedStack<CDart*>& AVertex,
			CTriplet& ATriplet,
			int AMarkTreated );

    /**
     *  Supprime tout les sommets de degré réel 0, en ayant préalablement
     *  décalé les arêtes fictives incidentes à ce sommet sauf 2.
     *  Normalement on peut le fait en même temps que le balayage de l'image
     *  en modifiant les 2 méthodes processEdge et processVertex mais cela
     *  complique le code et les explications (cf. revue CVIU 3D).
     *
     *  @param AMarkTreated La marque utilisée pour marquer les sommets
     *                      de degré réel différent de zéro.
     */
    void removeAllReelDegreeZeroVertex( int AMarkTreated );


  public:
    /**
     * Accesseur sur la taille de l'image.
     *
     * @return la taille de l'image en X.
     */
    unsigned int getSizeX() const;

    /**
     * Accesseur sur la taille de l'image.
     *
     * @return la taille de l'image en Z.
     */
    unsigned int getSizeY() const;
    
    /**
     * Accesseur sur la taille de l'image.
     *
     * @return la taille de l'image en Y.
     */
    unsigned int getSizeZ() const;

    /**
     * Accesseur sur les voxels de l'image.
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @return l'identifiant couleur du voxel (Ax,Ay,Az).
     */
    TColor getVoxel( unsigned int Ax, unsigned int Ay, unsigned int Az ) const;

    /**
     * Accesseur sur les voxels de l'image.
     *
     * @param ACoord coordonnées du voxel.
     * @return l'identifiant couleur du voxel ACoord.
     */
    TColor getVoxel( const CCoordinate & ACoord ) const;

    /**
     * Accesseur sur les marques des voxels.
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @return True si le voxel (Ax,Ay,Az) est marqué.
     */
    bool isVoxelMarked(unsigned int Ax, unsigned int Ay, unsigned int Az) const;

    /**
     * Définit la valeur de la marque d'un voxel.
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @param AOn valeur de la marque.
     */
    void setVoxelMark(unsigned int Ax, unsigned int Ay, unsigned int Az, bool AOn);

    /**
     * Marque le voxel désigné.
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @see setVoxelMark unmarkVoxel
     */
    void markVoxel(unsigned int Ax, unsigned int Ay, unsigned int Az);

    /**
     * Démarque le voxel désigné.
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @see setVoxelMark markVoxel
     */
    void unmarkVoxel(unsigned int Ax, unsigned int Ay, unsigned int Az);

    /**
     * Accesseur sur les marques des voxels.
     *
     * @param ACoord coordonnées du voxel.
     * @return True si le voxel (Ax,Ay,Az) est marqué.
     */
    bool isVoxelMarked( const CCoordinate & ACoord ) const;

    /**
     * Définit la valeur de la marque d'un voxel.
     *
     * @param ACoord coordonnées du voxel.
     * @param AOn valeur de la marque.
     */
    void setVoxelMark( const CCoordinate & ACoord, bool AOn);

    /**
     * Marque le voxel désigné.
     *
     * @param ACoord coordonnées du voxel.
     * @see setVoxelMark unmarkVoxel
     */
    void markVoxel( const CCoordinate & ACoord );

    /**
     * Démarque le voxel désigné.
     *
     * @param ACoord coordonnées du voxel.
     * @see setVoxelMark markVoxel
     */
    void unmarkVoxel( const CCoordinate & ACoord );

    /**
     * Démarque tous les voxels.
     */
    void unmarkAllVoxels();

    /**
     * Donne le voxel interne à la région délimité par le triplet courant.
     *
     * @param ATriplet un triplet.
     * @return Le voxel qui est à l'intérieur.
     */
    const CCoordinate getVoxelIn( const CTriplet & ATriplet ) const;

    /**
     * Donne le voxel externe à la région délimité par le triplet courant.
     *
     * @param ATriplet un triplet.
     * @return Le voxel qui est à l'extérieur.
     */
    const CCoordinate getVoxelOut( const CTriplet & ATriplet ) const;

    /**
     * Donne le voxel interne à la région délimité par le triplet du brin courant.
     *
     * @see getVoxelIn()
     * @param ADart un brin.
     * @return Le voxel qui est à l'intérieur.
     */
    const CCoordinate getVoxelIn( CDart * ADart ) const;

    /**
     * Donne le voxel externe à la région délimité par le triplet courant.
     *
     * @see getVoxelOut()
     * @param ADart un brin.
     * @return Le voxel qui est à l'extérieur.
     */
    const CCoordinate getVoxelOut( CDart * ADart ) const;

    /**
     * Est-ce que le voxel désigné est dans la région infinie?
     *
     * @param Ax coordonnée du voxel selon l'axe X.
     * @param Ay coordonnée du voxel selon l'axe Y.
     * @param Az coordonnée du voxel selon l'axe Z.
     * @return True si le voxel est dans la région infinie. False sinon.
     */
    bool isVoxelInInfiniteRegion(unsigned int Ax, unsigned int Ay,
				 unsigned int Az) const;

    /** Est-ce que le voxel désigné est dans la région infinie?
     *
     * @param ACoord coordonnées du voxel.
     * @return True si le voxel est dans la région infinie. False sinon.
     */
    bool isVoxelInInfiniteRegion( const CCoordinate & ACoord ) const;

    /**
     * Est-ce que les voxels désignés ont la même couleur?
     *
     * @param Ax1 coordonnée du voxel 1 selon l'axe X.
     * @param Ay1 coordonnée du voxel 1 selon l'axe Y.
     * @param Az1 coordonnée du voxel 1 selon l'axe Z.
     * @param Ax2 coordonnée du voxel 2 selon l'axe X.
     * @param Ay2 coordonnée du voxel 2 selon l'axe Y.
     * @param Az2 coordonnée du voxel 2 selon l'axe Z.
     * @return True si les voxels ont la même couleur.
     */
    bool sameVoxel(unsigned int Ax1, unsigned int Ay1, unsigned int Az1,
		   unsigned int Ax2, unsigned int Ay2, unsigned int Az2) const;

    /**
     * Les méthodes suivantes testent l'appartenante à la même région de deux
     * voxels. Le premier mot de la méthode désigne la position du premier voxel
     * par rapport au voxel courant (Ax, Ay, Az). Le deuxième mot (après le -)
     * désigne la position du deuxième voxel par rapport au premier (et donc
     * est soit up, left ou behind). 
     */
    bool sameVoxelActuLeft    (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelActuBehind  (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelActuUp      (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelLeftBehind  (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelLeftUp      (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelBehindLeft  (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelBehindUp    (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelUpLeft      (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelUpBehind    (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelUpbehindLeft(unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelLeftbehindUp(unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelLeftupBehind(unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    /**
     * Les méthodes suivantes testent l'appartenance à la même région d'un voxel
     * situé à l'extérieur de la région avec ses voisins respectivement de gauche,
     * de derrière et de dessus (left, behind, up).
     */
    bool sameVoxelOutLeft     (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelOutBehind   (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;

    bool sameVoxelOutUp       (unsigned int Ax, unsigned int Ay, 
			       unsigned int Az) const;


    //******************************************************************************

    
    //******************************************************************************
  public:
    /**
     * Donne le voxel interne à la région délimité par le triplet ATriplet.
     * @param ATriplet Surfel appartenant à la frontière d'une région.
     * @return Le voxel qui est à l'intérieur.
     */
    CCoordinate getVoxelFromTriplet( const CTriplet & ATriplet ) const;

    /**
     * Remonte le lien de chaque brin à la racine des arbres union-find des 
     * régions et des faces.
     */
    void cleanMap();
    
  private:
    /// @name Champs privés
    //@{
    CImage3D*   FImage;
    CKhalimsky* FKhalimsky;
    
    /// Marque pour les arêtes fictives
    int FFictiveMark;

    //@}
  };
  //******************************************************************************
} // namespace Map3d
//******************************************************************************
#endif // TOPOLOGICAL_MAP_HH
//******************************************************************************

