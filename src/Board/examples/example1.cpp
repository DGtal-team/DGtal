/**
 * @file   example1.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample drawing using the board library.
 * 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
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

int main( int, char *[] )
{
  Board board;

  // Default unit is the Postscript dot (1/720 inch).

  board.drawRectangle( -1, 1, 2.0, 2.0 );
  board.setPenColorRGBi( 0, 0, 255 );
  board.fillCircle( 0.0, 0.0, 0.05 );

  std::vector<Point> points;
  for ( double x = -1.0; x < 1.0; x += 0.02 ) {
    points.push_back( Point( x, sin(x*6.28) ) );
  }
  board.setLineWidth( 1.0 );
  board.setPenColorRGBi( 0, 0, 255 );
  board.drawPolyline( points );
  board.drawArrow( 0, 0, 0.5, 0.5, true );

  board.fillGouraudTriangle( -0.5, 0, Color( 255, 0, 0 ), 
			     0, 0, Color( 0, 255, 0 ),
			     -0.3, 0.3, Color( 0, 0, 255 ) );

  
  board.setPenColorRGBi( 255, 1, 1 ).fillGouraudTriangle( -0.5, 0, 1.0f,
							  0, 0, 2.0f,
							  -0.3, -0.3, 0.1f );

  board.scale(100);

  board.saveEPS( "draw1.eps" );
  board.saveFIG( "draw1.fig" );
  board.saveSVG( "draw1.svg" );
  board.saveEPS( "draw1_Letter.eps", Board::Letter );
  board.saveFIG( "draw1_Letter.fig", Board::Letter );
  board.saveSVG( "draw1_Letter.svg", Board::Letter );
  exit(0);
}
