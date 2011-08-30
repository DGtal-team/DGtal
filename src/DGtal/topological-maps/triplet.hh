//******************************************************************************
#ifndef TRIPLET_HH
#define TRIPLET_HH
//******************************************************************************
#include <iostream>
#include <cassert>
#include <stdint.h>
//******************************************************************************
typedef uint16_t TNatural;
typedef uint8_t  TLinel;

enum
  {
    XPOS = 0, YPOS, ZPOS, XNEG, YNEG, ZNEG
  };

//******************************************************************************
#define GET_OPOSITE_LINEL(L) ((L>=3 ? (L-3) : L+3))
#define GET_POSITIVE_LINEL(L) ((L>=3 ? (L-3) : L))

// Retourne la troisième direction: suppose que L1 et L2 ne sont pas
// dans la même direction et retourne la 3ème direction en positif.
#define GET_THIRD_DIRECTION(L1,L2)			\
  ((3-(GET_POSITIVE_LINEL(L1)+GET_POSITIVE_LINEL(L2))))

//******************************************************************************
#define MAKE_LINEL_SURFEL(L, S) (((S) << 3) | (L))
#define GET_LINEL(LS) ((LS) & 0x07)
#define GET_SURFEL(LS) (((LS) >> 3) & 0x07)
#define SET_LINEL(LS, L) (((LS) & 0x38) | (L & 0x07))
#define SET_SURFEL(LS, S) (((S & 0x07) << 3) | (LS & 0x07))

//******************************************************************************
class CTriplet
{
public:

  CTriplet::CTriplet() :
    Fx(0), Fy(0), Fz(0),
    FLS(MAKE_LINEL_SURFEL(XPOS, YPOS))
  {}
  //******************************************************************************

  CTriplet::CTriplet(const CTriplet & ATriplet) :
    Fx(ATriplet.Fx), Fy(ATriplet.Fy), Fz(ATriplet.Fz),
    FLS(ATriplet.FLS)
  {}
  //******************************************************************************

  CTriplet::CTriplet(TNatural Ax, TNatural Ay, TNatural Az) :
    Fx(Ax), Fy(Ay), Fz(Az),
    FLS(MAKE_LINEL_SURFEL(XPOS, YPOS))
  {}
  //******************************************************************************

  CTriplet::CTriplet(TNatural Ax, TNatural Ay, TNatural Az,
		     const TLinel& ALinel, const TLinel& ALinel2) :
    Fx(Ax), Fy(Ay), Fz(Az),
    FLS(MAKE_LINEL_SURFEL(ALinel, ALinel2))
  {}
  //******************************************************************************

  CTriplet& CTriplet::operator = (const CTriplet & ATriplet)
  {
    Fx  = ATriplet.Fx;
    Fy  = ATriplet.Fy;
    Fz  = ATriplet.Fz;
    FLS = ATriplet.FLS;
    return *this;
  }
  //------------------------------------------------------------------------------

  bool CTriplet::operator == (const CTriplet & ATriplet) const
  {
    return ( Fx == ATriplet.Fx && Fy == ATriplet.Fy && Fz == ATriplet.Fz &&
	     FLS == ATriplet.FLS);
  }
  //------------------------------------------------------------------------------

  bool CTriplet::operator != (const CTriplet & ATriplet) const
  { return ! operator==(ATriplet); }
  //------------------------------------------------------------------------------

  bool CTriplet::operator < (const CTriplet & ATriplet) const
  {
    return ( ( Fz < ATriplet.Fz ) ||
	     ( Fz == ATriplet.Fz && Fy < ATriplet.Fy ) ||
	     ( Fz == ATriplet.Fz && Fy == ATriplet.Fy && Fx < ATriplet.Fx ) ||
	     ( Fz == ATriplet.Fz && Fy == ATriplet.Fy && Fx == ATriplet.Fx && 
	       ( getLinel() <  ATriplet.getLinel() ||
		 ( getLinel() == ATriplet.getLinel() &&
		   getLinel2() < ATriplet.getLinel2() ) ) ) );
  }
  //******************************************************************************

  bool CTriplet::operator <= (const CTriplet & ATriplet) const
  { return operator<( ATriplet ) || operator==( ATriplet ); }
  //******************************************************************************

  bool CTriplet::operator > (const CTriplet & ATriplet) const
  { return ! operator<=( ATriplet ); }
  //******************************************************************************

  bool CTriplet::operator >= (const CTriplet & ATriplet) const
  { return ! operator<( ATriplet ); }
  //******************************************************************************

  CTriplet::~CTriplet()
  {}
  //******************************************************************************
  TNatural CTriplet::getX() const { return Fx; }
  TNatural CTriplet::getY() const { return Fy; }
  TNatural CTriplet::getZ() const { return Fz; }

  TNatural CTriplet::incX(int ANb) { Fx+=ANb; return Fx; }
  TNatural CTriplet::incY(int ANb) { Fy+=ANb; return Fy; }
  TNatural CTriplet::incZ(int ANb) { Fz+=ANb; return Fz; }

  void CTriplet::setX(const TNatural& Ax) { Fx = Ax; }
  void CTriplet::setY(const TNatural& Ay) { Fy = Ay; }
  void CTriplet::setZ(const TNatural& Az) { Fz = Az; }

  //******************************************************************************

  TLinel CTriplet::getLinel() const
  { return GET_LINEL(FLS); }


  void CTriplet::setLinel(const TLinel& ALinel)
  { FLS = SET_LINEL(FLS, ALinel); }


  TLinel CTriplet::getLinel2() const
  { return GET_SURFEL(FLS); }


  void CTriplet::setLinel2(const TLinel& ALinel)
  { FLS = SET_SURFEL(FLS, ALinel); }


  bool CTriplet::samePointel(const CTriplet & ATriplet) const
  { return (Fx==ATriplet.Fx && Fy==ATriplet.Fy && Fz==ATriplet.Fz); }


  void CTriplet::setPointel(const CTriplet & ATriplet)
  {
    Fx=ATriplet.Fx;
    Fy=ATriplet.Fy;
    Fz=ATriplet.Fz;
  }


  void CTriplet::setPointel(const TNatural& Ax, const TNatural& Ay,
			    const TNatural& Az)
  { Fx=Ax; Fy=Ay; Fz=Az; }


  void CTriplet::swapLinels()
  {
    TLinel linel = getLinel();
    setLinel (getLinel2());
    setLinel2(linel);
  }


  void CTriplet::reverseLinel()
  { setLinel(GET_OPOSITE_LINEL(getLinel())); }


  void CTriplet::reverseLinel2()
  { setLinel2(GET_OPOSITE_LINEL(getLinel2())); }


  void CTriplet::setTriplet(const TNatural& Ax, const TNatural& Ay,
			    const TNatural& Az,
			    const TLinel& ALinel, const TLinel& ALinel2)
  {
    Fx=Ax; Fy=Ay; Fz=Az;
    setLinel(ALinel); setLinel2(ALinel2);
  }

  //******************************************************************************

  void CTriplet::setNextTripletDir1()
  {
    if      ( getLinel()==XNEG ) --Fx;
    else if ( getLinel()==XPOS ) ++Fx;
    else if ( getLinel()==YNEG ) --Fy;
    else if ( getLinel()==YPOS ) ++Fy;
    else if ( getLinel()==ZNEG ) --Fz;
    else if ( getLinel()==ZPOS ) ++Fz;
  }
  //------------------------------------------------------------------------------

  void CTriplet::setNextTripletDir2()
  {
    if      ( getLinel2()==XNEG ) --Fx;
    else if ( getLinel2()==XPOS ) ++Fx;
    else if ( getLinel2()==YNEG ) --Fy;
    else if ( getLinel2()==YPOS ) ++Fy;
    else if ( getLinel2()==ZNEG ) --Fz;
    else if ( getLinel2()==ZPOS ) ++Fz;
  }
  //------------------------------------------------------------------------------

  void CTriplet::setPrevTripletDir1()
  {
    if      ( getLinel()==XNEG ) ++Fx;
    else if ( getLinel()==XPOS ) --Fx;
    else if ( getLinel()==YNEG ) ++Fy;
    else if ( getLinel()==YPOS ) --Fy;
    else if ( getLinel()==ZNEG ) ++Fz;
    else if ( getLinel()==ZPOS ) --Fz;
  }
  //------------------------------------------------------------------------------

  void CTriplet::setPrevTripletDir2()
  {
    if      ( getLinel2()==XNEG ) ++Fx;
    else if ( getLinel2()==XPOS ) --Fx;
    else if ( getLinel2()==YNEG ) ++Fy;
    else if ( getLinel2()==YPOS ) --Fy;
    else if ( getLinel2()==ZNEG ) ++Fz;
    else if ( getLinel2()==ZPOS ) --Fz;
  }
  //******************************************************************************

  CTriplet CTriplet::getReverseTripletDir1() const
  {
    CTriplet res(*this);
    res.reverseLinel();
    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getReverseTripletDir2() const
  {
    CTriplet res(*this);
    res.reverseLinel2();
    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getPrevTripletDir1() const
  {
    CTriplet res(*this);
    res.setPrevTripletDir1();
    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getNextTripletDir1() const
  {
    CTriplet res(*this);
    res.setNextTripletDir1();
    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getPrevTripletDir2() const
  {
    CTriplet res(*this);
    res.setPrevTripletDir2();
    return res;
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getNextTripletDir2() const
  {
    CTriplet res(*this);
    res.setNextTripletDir2();
    return res;
  }
  //******************************************************************************

  void CTriplet::setNextPointel()
  {
    setNextTripletDir1();
    setLinel(GET_OPOSITE_LINEL(getLinel()));
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getNextPointel() const
  {
    CTriplet res(*this);
    res.setNextPointel();  
    return res;
  }
  //******************************************************************************

  void CTriplet::setNextPointelDir2()
  {
    setNextTripletDir2();
    setLinel2(GET_OPOSITE_LINEL(getLinel2()));
  }
  //------------------------------------------------------------------------------

  CTriplet CTriplet::getNextPointelDir2() const
  {
    CTriplet res(*this);
    res.setNextPointelDir2();  
    return res;
  }
  //******************************************************************************

  void CTriplet::setNextLinel()
  {
    switch( getLinel() )
      {
      case XNEG: setLinel(XPOS); break;
      case XPOS: setLinel(YNEG); break;
      case YNEG: setLinel(YPOS); break;
      case YPOS: setLinel(ZNEG); break;
      case ZNEG: setLinel(ZPOS); break;
      case ZPOS: setLinel(XNEG); break;
      default: assert(false);
      }
  }
  //******************************************************************************

  CTriplet CTriplet::getNextLinel() const
  {
    CTriplet res(*this);
    res.setNextLinel();  
    return res;
  }
  //******************************************************************************

  void CTriplet::setNextSurfel()
  {
    switch ( getLinel() )
      {
      case XNEG:
	{
	  switch ( getLinel2() )
	    {
	    case YNEG: setLinel2(ZPOS); break;
	    case YPOS: setLinel2(ZNEG); break;
	    case ZNEG: setLinel2(YNEG); break;
	    case ZPOS: setLinel2(YPOS); break;
	    default: assert(false);
	    }
	}
	break;
      case XPOS:
	{
	  switch ( getLinel2() )
	    {
	    case YNEG: setLinel2(ZNEG); break;
	    case YPOS: setLinel2(ZPOS); break;
	    case ZNEG: setLinel2(YPOS); break;
	    case ZPOS: setLinel2(YNEG); break;
	    default: assert(false);
	    }
	}
	break;
      case YNEG:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(ZNEG); break;
	    case XPOS: setLinel2(ZPOS); break;
	    case ZNEG: setLinel2(XPOS); break;
	    case ZPOS: setLinel2(XNEG); break;
	    default: assert(false);
	    }
	}
	break;
      case YPOS:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(ZPOS); break;
	    case XPOS: setLinel2(ZNEG); break;
	    case ZNEG: setLinel2(XNEG); break;
	    case ZPOS: setLinel2(XPOS); break;
	    default: assert(false);
	    }
	}
	break;
      case ZNEG:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(YPOS); break;
	    case XPOS: setLinel2(YNEG); break;
	    case YNEG: setLinel2(XNEG); break;
	    case YPOS: setLinel2(XPOS); break;
	    default: assert(false);
	    }
	}
	break;
      case ZPOS:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(YNEG); break;
	    case XPOS: setLinel2(YPOS); break;
	    case YNEG: setLinel2(XPOS); break;
	    case YPOS: setLinel2(XNEG); break;
	    default: assert(false);
	    }
	}
	break;
      default: assert(false);
      }  
  }
  //******************************************************************************

  CTriplet CTriplet::getNextSurfel() const
  {
    CTriplet res(*this);
    res.setNextSurfel();  
    return res;
  }
  //******************************************************************************

  void CTriplet::setPrevSurfel()
  {
    switch ( getLinel() )
      {
      case XNEG:
	{
	  switch ( getLinel2() )
	    {
	    case YNEG: setLinel2(ZNEG); break;
	    case YPOS: setLinel2(ZPOS); break;
	    case ZNEG: setLinel2(YPOS); break;
	    case ZPOS: setLinel2(YNEG); break;
	    default: assert(false);
	    }
	}
	break;
      case XPOS:
	{
	  switch ( getLinel2() )
	    {
	    case YNEG: setLinel2(ZPOS); break;
	    case YPOS: setLinel2(ZNEG); break;
	    case ZNEG: setLinel2(YNEG); break;
	    case ZPOS: setLinel2(YPOS); break;
	    default: assert(false);
	    }
	}
	break;
      case YNEG:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(ZPOS); break;
	    case XPOS: setLinel2(ZNEG); break;
	    case ZNEG: setLinel2(XNEG); break;
	    case ZPOS: setLinel2(XPOS); break;
	    default: assert(false);
	    }
	}
	break;
      case YPOS:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(ZNEG); break;
	    case XPOS: setLinel2(ZPOS); break;
	    case ZNEG: setLinel2(XPOS); break;
	    case ZPOS: setLinel2(XNEG); break;
	    default: assert(false);
	    }
	}
	break;
      case ZNEG:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(YNEG); break;
	    case XPOS: setLinel2(YPOS); break;
	    case YNEG: setLinel2(XPOS); break;
	    case YPOS: setLinel2(XNEG); break;
	    default: assert(false);
	    }
	}
	break;
      case ZPOS:
	{
	  switch ( getLinel2() )
	    {
	    case XNEG: setLinel2(YPOS); break;
	    case XPOS: setLinel2(YNEG); break;
	    case YNEG: setLinel2(XNEG); break;
	    case YPOS: setLinel2(XPOS); break;
	    default: assert(false);
	    }
	}
	break;
      default: assert(false);
      }  
  }
  //******************************************************************************

  CTriplet CTriplet::getPrevSurfel() const
  {
    CTriplet res(*this);
    res.setPrevSurfel();  
    return res;
  }
  //******************************************************************************

  bool CTriplet::isPositiveDirection( const CTriplet & ATriplet )
  {
    return GET_LINEL(ATriplet.FLS) < 3;
  }

  friend std::ostream& operator<<( std::ostream& AStream,
				   const CTriplet& ATriplet )
  {
    AStream << "[ ("
	    << ATriplet.Fx << ", "
	    << ATriplet.Fy << ", "
	    << ATriplet.Fz
	    << ") ";

    switch (GET_LINEL(ATriplet.FLS))
      {
      case XPOS: AStream<<"XPOS "; break;
      case YPOS: AStream<<"YPOS "; break;
      case ZPOS: AStream<<"ZPOS "; break;
      case XNEG: AStream<<"XNEG "; break;
      case YNEG: AStream<<"YNEG "; break;
      case ZNEG: AStream<<"ZNEG "; break;
      }

    switch (GET_SURFEL(ATriplet.FLS))
      {
      case XPOS: AStream<<"XPOS"; break;
      case YPOS: AStream<<"YPOS"; break;
      case ZPOS: AStream<<"ZPOS"; break;
      case XNEG: AStream<<"XNEG"; break;
      case YNEG: AStream<<"YNEG"; break;
      case ZNEG: AStream<<"ZNEG"; break;
      }
  
    return AStream<<"]";
  }
  

private:
  /// @name Attributs privés
  //@{
  TNatural Fx, Fy, Fz;
  TLinel   FLS; // contient en fait 2 lignels
  //@}
};
//******************************************************************************
#endif // TRIPLET_HH
//******************************************************************************
