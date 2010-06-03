/**
 * @file   logo.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample drawing using the board library.
 * 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 */
/*
 * @file   example1.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample drawing using the board library.
 * 
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Board.h"
using namespace LibBoard;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main( int, char *[] )
{
  Board board;

  // board.clear( Color(0,120,0) );
  board << Text( -0.45, -0.2, "LibBoard", 
		 Fonts::Helvetica, "'Bookman Old Style',Verdana", 64.0f, Color::Green );
  board.setLineWidth( 2.5 ).setPenColorRGBi( 255, 100, 0 );
  board.setLineStyle( Shape::SolidStyle );
  board.setLineJoin( Shape::MiterJoin );
  board.setLineCap( Shape::RoundCap );
  board.drawLine( -0.5, -0.27, 0.5, -0.27 ); 
  board.addDuplicates( board.last(), 10, 0, -0.02 );

  Point p = board.last<Line>().boundingBox().topLeft();
  int n = 20;
  double angle = -M_PI/(2*n);
  while ( n-- ) 
    board << board.last<Line>().rotated( angle, p );

  n = 30;
  angle = -M_PI/(n);
  while ( n-- ) 
    board << board.last<Line>().scaled( 0.95 ).rotated( angle, p );

  board.saveEPS( "logo_A4.eps", Board::A4 );
  board.saveFIG( "logo_A4.fig", Board::A4 );

  board.scale( 310 );
  board.saveSVG( "logo_A4.svg", -1, -1, 10 );
  exit(0);
}
