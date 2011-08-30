#ifndef COORDINATE_HH
#define COORDINATE_HH
//*****************************************************************************
#include "inline-macro.hh"
#include "triplet.hh"

/**
 * La classe CCoordinate représente une coordonnée 3D entière.
 * @author Alexandre Dupas
 */
class CCoordinate
{
private:
  int FCoord[3];

public:
  // @name Constructeurs et destructeur
  // @{

  /**
   * Constructeur par défaut
   * Construction d'une nouvelle instance de la classe, initialisée
   * au sommet origine (0,0,0).
   *
   * @return Une nouvelle instance de la classe
   */
  CCoordinate();

  /**
   * Constructeur avec initialisation
   * Construction d'une nouvelle instance de la classe, initialisée au
   * sommet de coordonnées (Ax,Ay,Az).
   *
   * @param Ax La première  coordonnée du sommet
   * @param Ay La deuxième  coordonnée du sommet
   * @param Az La troisième coordonnée du sommet
   */
  CCoordinate(int Ax, int Ay, int Az);

  /**
   * Constructeur avec initialisation
   * Construction d'une nouvelle instance de la classe, initialisée au
   * sommet de coordonnées (ATab[0],ATab[1],ATab[2]).
   *
   * @param ATab Un tableau des 3 coordonnées du sommet
   */
  CCoordinate(int ATab[3]);

  /**
   * Constructeur par copie
   * Construction d'une nouvelle instance de la classe, en copiant le sommet
   * passé en paramètre.
   *
   * @param AVertex le sommet à copier
   */
  CCoordinate(const CCoordinate& AVertex);

  /**
   * Constructeur prenant en paramètre un triplet.
   * Construction d'une nouvelle instance de la classe, en copiant uniquement
   * les coordonnées du pointel du triplet passé en paramètre.
   *
   * @param ATriplet le triplet à "copier"
   */
  CCoordinate(const CTriplet& ATriplet);

  /**
   * Destructeur
   */
  ~CCoordinate();

  // @}
  // @name Accesseurs
  // @{

  /**
   * Accès en lecture à la première composante du sommet.
   */
  int getX() const;

  /**
   * Accès en lecture à la deuxième composante du sommet.
   */
  int getY() const;

  /**
   * Accès en lecture à la troisième composante du sommet.
   */
  int getZ() const;

  /**
   * Accès en lecture à une composante du sommet.
   *
   * @param ADim Une dimension (0, 1 ou 2)
   */
  int getCoord(int ADim) const;

  /**
   * Positionne la première composante du sommet à la valeur ANewX.
   *
   * @param ANewX Une valeur quelconque
   */
  void setX(int ANewX);

  /**
   * Positionne la deuxième composante du sommet à la valeur ANewY.
   *
   * @param ANewY Une valeur quelconque
   */
  void setY(int ANewY);

  /**
   * Positionne la troisième composante du sommet à la valeur ANewZ.
   *
   * @param ANewZ Une valeur quelconque
   */
  void setZ(int ANewZ);

  /**
   * Change la valeur de la composante ADim du sommet.
   *
   * @param ADim Une dimension (0, 1 ou 2)
   * @param ANewCoord Une valeur quelconque
   */
  void setCoord(int ADim, int ANewCoord);

  /**
   * Affecte les trois composantes du sommet.
   *
   * @param ANewX Une valeur quelconque
   * @param ANewY Une valeur quelconque
   * @param ANewZ Une valeur quelconque
   */
  void setXYZ(int ANewX, int ANewY, int ANewZ);

  /** Retourne la norme au carré du vecteur correspondant à la coordonnée
   * actuelle.
   */
  int sqrNorm() const;

  // @}
  // @name Divers opérateurs
  // @{

  CCoordinate& operator=(const CCoordinate& AVertex);

  CCoordinate operator+(const CCoordinate& AVertex);
  CCoordinate operator-(const CCoordinate& AVertex);

  CCoordinate operator*(const int AScalar);
  CCoordinate operator/(const int AScalar);

  bool operator==(const CCoordinate& AVertex) const;
  bool operator!=(const CCoordinate& AVertex) const;

  friend bool operator <  (CCoordinate u,CCoordinate v);
  friend bool operator <= (CCoordinate u,CCoordinate v);
  friend bool operator >  (CCoordinate u,CCoordinate v);
  friend bool operator >= (CCoordinate u,CCoordinate v);

  /**
   * Affichage dans un flot de la coordonnée ACoordinate.
   *
   * @param  AStream  Le flot dans lequel afficher le triplet
   * @param  ACoordinate Les coordonnées à afficher
   * @return Le flot utilisé pour l'affichage
   */
  friend std::ostream& operator<<(std::ostream& AStream,
				  const CCoordinate& ACoordinate);

};

// @}
// @name Constantes
// @{
  
/**
 * Le sommet de cordonnées (0,0,0).
 */
static const CCoordinate COORDINATE_ORIGIN(0,0,0);
  
/**
 * Le vecteur de composantes (1,0,0).
 */
static const CCoordinate COORDINATE_OX(1,0,0);
  
/**
 * Le vecteur de composantes (0,1,0).
 */
static const CCoordinate COORDINATE_OY(0,1,0);
  
/**
 * Le vecteur de composantes (0,0,1).
 */
static const CCoordinate COORDINATE_OZ(0,0,1);
  
/** BASE
 * Les trois vecteurs de base OX, OY et OZ placés dans un tableau.
 */
static const CCoordinate COORDINATE_BASE[3] = { COORDINATE_OX, 
						COORDINATE_OY, 
						COORDINATE_OZ };
  
// @}

//******************************************************************************
#include INCLUDE_INLINE("coordinate.icc")
//******************************************************************************
#endif // VERTEX_HH
//******************************************************************************
