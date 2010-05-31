/**
 * @file   example4.cpp
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
 * @file   example4.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Illustration of the use of Board, ShapeList and Group.
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Board.h"

using namespace LibBoard;

int main( int , char *[] )
{
  Board board;
  board.clear( Color(200,255,200) );

  for ( double x = -20.0; x <= 20; x+= 1.0 )
    for ( double y = -20.0; y <= 20; y+= 1.0 ) {
      board << Dot(x,y,Color::Black,1.0);
      board.setPenColorRGBf( 1.0, 0.0, 0.0 );
      board.drawDot( x+0.2, y+0.2 );      
    }
  

  Group g;
  g << Line( -5, 2, 5, 2, Color::Red, 1 )
    << Rectangle( -5, 2, 1, 4, Color::None, Color::Blue, 1 )
    << Rectangle( 4, 2, 1, 4, Color::None, Color::Blue, 1 )
    << Ellipse( 0, 0, 5, 2, Color::Red, Color::None, 1 );

  Group g2(g);
  Group g3(g);
  g2.translate( 0, 6 );
  g3.translate( 0, -6 );

  Group f;
  f << g << g2 << g3;

  board << f.rotateDeg( 45.0 );

  ShapeList l;
  Circle c( 0, 0, 1.8, Color::Red, Color::Green, 0.02 );
  for ( int i = 0; i < 5; ++i ) 
    l << c.scale( 1, 0.5 );

  
  for ( double x = -20, a = 0; x < 20 ; x += 4, a += 0.3 ) {
    board << l.rotated( a ).translate( x, 10 );
  }

  g.clear();

  Rectangle r1( 2, 2, 3, 1, Color::Black, Color::None, 0.01 );
  Rectangle r2 = r1.translated( -8, -8 );
  for ( double alpha = 0; alpha < 2*M_PI; alpha += 0.2 ) {
    g << r1.rotated( alpha );
    board << r2.rotated( alpha, r2.topLeft() );
  }
  
  board << g.scale( 1.5, 1 );

  board.scale(10);

  board.saveEPS( "draw4.eps", Board::A4 );
  board.saveFIG( "draw4.fig", Board::A4 );
  board.saveSVG( "draw4.svg", Board::A4 );
  exit(0);
}
