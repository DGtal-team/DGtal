//******************************************************************************
#ifndef VOXEL_HH
#define VOXEL_HH
//******************************************************************************
enum 
{
  FIRST_DIR = 0,

  FIRST_DIR_6CONNECTED = FIRST_DIR,
  
  LEFT   = FIRST_DIR,
  BEHIND,
  TOP,
  RIGHT,
  FRONT,
  BOTTOM,

  LAST_DIR_6CONNECTED,

  LAST_DIR=LAST_DIR_6CONNECTED,

  FIRST_DIR_18CONNECTED = LAST_DIR_6CONNECTED,
  
  LEFT_BEHIND = FIRST_DIR_18CONNECTED,
  RIGHT_BEHIND,
  TOP_BEHIND,
  BOTTOM_BEHIND,
  LEFT_FRONT,
  RIGHT_FRONT,
  TOP_FRONT,
  BOTTOM_FRONT,
  LEFT_TOP,
  RIGHT_TOP,
  LEFT_BOTTOM,
  RIGHT_BOTTOM,

  LAST_DIR_18CONNECTED
};
//******************************************************************************
typedef int TVoxelCoord;

class CVoxel
{
public:
  CVoxel();
  CVoxel(TVoxelCoord Ax, TVoxelCoord Ay, TVoxelCoord Az);
  CVoxel(const CVoxel&);
  ~CVoxel();
  
  // Accesseurs
  TVoxelCoord getX() const;
  TVoxelCoord getY() const;
  TVoxelCoord getZ() const;

  // Modificateurs
  void setX(TVoxelCoord);
  void setY(TVoxelCoord);
  void setZ(TVoxelCoord);
  void set (TVoxelCoord,TVoxelCoord,TVoxelCoord);

  // Incrementation
  void incX(int ADx = 1);
  void incY(int ADy = 1);
  void incZ(int ADz = 1);

  // Opérateurs
  CVoxel& operator = (const CVoxel&);
  
  bool operator == (const CVoxel&) const;
  bool operator != (const CVoxel&) const;

  friend std::ostream& operator << (std::ostream & os,const CVoxel&);

  // Voisins
  CVoxel left() const;
  CVoxel right() const;
  CVoxel top() const;
  CVoxel bottom() const;
  CVoxel front() const;
  CVoxel behind() const;

  CVoxel leftBehind() const;
  CVoxel rightBehind() const;
  CVoxel topBehind() const;
  CVoxel bottomBehind() const;
  CVoxel leftFront() const;
  CVoxel rightFront() const;
  CVoxel topFront() const;
  CVoxel bottomFront() const;
  CVoxel leftTop() const;
  CVoxel rightTop() const;
  CVoxel leftBottom() const;
  CVoxel rightBottom() const;
  
  CVoxel neighboor(unsigned int ADir) const;
  
private:
  TVoxelCoord Fx, Fy, Fz;
};
//******************************************************************************
#include <cassert>
//******************************************************************************
inline
CVoxel::CVoxel() : Fx(0), Fy(0), Fz(0)
{}
inline
CVoxel::CVoxel(TVoxelCoord Ax, TVoxelCoord Ay, TVoxelCoord Az) :
  Fx(Ax), Fy(Ay), Fz(Az)
{}
inline
CVoxel::CVoxel(const CVoxel& AVoxel) :
  Fx(AVoxel.Fx), Fy(AVoxel.Fy), Fz(AVoxel.Fz)
{}
//------------------------------------------------------------------------------
inline
CVoxel::~CVoxel()
{}
//------------------------------------------------------------------------------
inline
TVoxelCoord CVoxel::getX() const 
{ return Fx; }
inline
TVoxelCoord CVoxel::getY() const
{ return Fy; }
inline
TVoxelCoord CVoxel::getZ() const
{ return Fz; }
//------------------------------------------------------------------------------
inline
void CVoxel::setX(TVoxelCoord AValue)
{ Fx = AValue; }
inline
void CVoxel::setY(TVoxelCoord AValue)
{ Fy = AValue; }
inline
void CVoxel::setZ(TVoxelCoord AValue)
{ Fz = AValue; }
inline
void CVoxel::set(TVoxelCoord Ax, TVoxelCoord Ay, TVoxelCoord Az)
{ Fx=Ax; Fy=Ay; Fz=Az; }
//------------------------------------------------------------------------------
inline
void CVoxel::incX(int ADx)
{ assert( ADx+(int)Fx>=0 ); Fx += ADx; }
inline
void CVoxel::incY(int ADy)
{ assert( ADy+(int)Fy>=0 ); Fy += ADy; }
inline
void CVoxel::incZ(int ADz)
{ assert( ADz+(int)Fz>=0 ); Fz += ADz; }
//------------------------------------------------------------------------------
inline
CVoxel& CVoxel::operator = (const CVoxel& AVoxel)
{
  Fx = AVoxel.Fx; Fy = AVoxel.Fy; Fz = AVoxel.Fz;
  return *this;
}
//------------------------------------------------------------------------------
inline
bool CVoxel::operator == (const CVoxel& AVoxel) const
{ return Fx==AVoxel.Fx && Fy==AVoxel.Fy && Fz==AVoxel.Fz; }
inline
bool CVoxel::operator != (const CVoxel& AVoxel) const
{ return !operator==(AVoxel); }
//------------------------------------------------------------------------------
inline
CVoxel CVoxel::left() const
{ return CVoxel(Fx-1, Fy, Fz); }
inline
CVoxel CVoxel::right() const
{ return CVoxel(Fx+1, Fy, Fz); }
inline
CVoxel CVoxel::behind() const
{ return CVoxel(Fx, Fy-1, Fz); }
inline
CVoxel CVoxel::front() const
{ return CVoxel(Fx, Fy+1, Fz); }
inline
CVoxel CVoxel::bottom() const
{ return CVoxel(Fx, Fy, Fz-1); }
inline
CVoxel CVoxel::top() const
{ return CVoxel(Fx, Fy, Fz+1); }
inline
CVoxel CVoxel::leftBehind() const
{ return CVoxel(Fx-1, Fy-1, Fz); }
inline
CVoxel CVoxel::rightBehind() const
{ return CVoxel(Fx+1, Fy-1, Fz); }
inline
CVoxel CVoxel::topBehind() const
{ return CVoxel(Fx, Fy-1, Fz+1); }
inline
CVoxel CVoxel::bottomBehind() const
{ return CVoxel(Fx, Fy-1, Fz-1); }
inline
CVoxel CVoxel::leftFront() const
{ return CVoxel(Fx-1, Fy+1, Fz); }
inline
CVoxel CVoxel::rightFront() const
{ return CVoxel(Fx+1, Fy+1, Fz); }
inline
CVoxel CVoxel::topFront() const
{ return CVoxel(Fx, Fy+1, Fz+1); }
inline
CVoxel CVoxel::bottomFront() const
{ return CVoxel(Fx, Fy+1, Fz-1); }
inline
CVoxel CVoxel::leftTop() const
{ return CVoxel(Fx-1, Fy, Fz+1); }
inline
CVoxel CVoxel::rightTop() const
{ return CVoxel(Fx+1, Fy, Fz+1); }
inline
CVoxel CVoxel::leftBottom() const
{ return CVoxel(Fx-1, Fy, Fz-1); }
inline
CVoxel CVoxel::rightBottom() const
{ return CVoxel(Fx+1, Fy, Fz-1); }
inline
CVoxel CVoxel::neighboor(unsigned int ADir) const
{
  switch(ADir)
    {
    case LEFT:   	return left  	   (); break;
    case RIGHT:  	return right 	   (); break;
    case BEHIND: 	return behind	   (); break;
    case FRONT:  	return front 	   (); break;
    case BOTTOM: 	return bottom	   (); break;
    case TOP:    	return top   	   (); break;
    case LEFT_BEHIND: 	return leftBehind  (); break;
    case RIGHT_BEHIND: 	return rightBehind (); break;
    case TOP_BEHIND: 	return topBehind   (); break;
    case BOTTOM_BEHIND: return bottomBehind(); break;
    case LEFT_FRONT: 	return leftFront   (); break;
    case RIGHT_FRONT: 	return rightFront  (); break;
    case TOP_FRONT: 	return topFront	   (); break;
    case BOTTOM_FRONT: 	return bottomFront (); break;
    case LEFT_TOP: 	return leftTop	   (); break;
    case RIGHT_TOP: 	return rightTop	   (); break;
    case LEFT_BOTTOM: 	return leftBottom  (); break;
    case RIGHT_BOTTOM: 	return rightBottom (); break;

    default: assert(false);
    }
  return *this;
}
//******************************************************************************
inline
std::ostream& operator << (std::ostream & os, const CVoxel & AVoxel)
{
  os<<"("<<AVoxel.getX()<<", "<<AVoxel.getY()<<", "<<AVoxel.getZ()<<")";
  return os;
}
//******************************************************************************
#endif // VOXEL_HH
//******************************************************************************
