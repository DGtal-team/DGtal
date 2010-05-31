/**
 * @file   arrows.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample program to check that arrows are correctly drawn.
 *
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 */
/*
 * @file   arrows.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample program to check that arrows are correctly drawn.
 *
 */
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <string>
#include "Board.h"
using namespace LibBoard;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float  random_gray() {
  return rand() / static_cast<float>(RAND_MAX);
}

int main( int , char *[] )
{
  Board board;
  board.clear( Color(233,250,140) );
  board.setLineWidth(1);
  srand( time(0) );
  float radius = 0.1;
  float angle = 0.0;
  // board.setUnit( Board::CM );      // Unit for "drawSomething()" functions is 1cm.
  board << Board::CM;

  board << Rectangle( -8, 12, 16, 24, Color::Black, Color::None, 0.1 );

  for ( int i = 0; i < 45; ++i, radius += 0.1 ) {
    angle = i * (2*M_PI/45);
    board.setPenColorRGBf( random_gray(), random_gray(), random_gray() );
    board.drawArrow( 0, 0,
		     radius * cos( angle ),
		     radius * sin( angle ),
		     false );
  }

  board.setPenColorRGBi(255,0,0);
  board.setLineWidth(2);
  board.setFont( Fonts::CourierBold, 12 );
  radius *= 1.2;
  for ( int i = 0; i < 45; ++i ) {
    angle = i * (2*M_PI/45);
    board.drawDot( radius * cos( angle ), radius * sin( angle ) );    
    std::stringstream ss;
    ss << i;
    board.drawText( radius * cos( angle ), radius * sin( angle ), ss.str() );
  }
  
  board.setPenColorRGBi( 0, 0, 0 );
  board.setFont( Fonts::PalatinoRoman, 24 );
  board.drawText( -5, 10, "Arrows" );

  board.setLineWidth( 1 );
  board.drawArrow( -7, 9, 5, 9 );  
  board.setPenColorRGBi( 255, 0, 0 );
  board.drawArrow( -6, -9, -6, 9 );  

  
  Group g;
  for ( int y = 1; y < 10; ++y) 
    g << Rectangle( 0, y * 10 - 1, 10, 9, Color::Black, Color::None, 0.2 );
  
  // Change the unit and add the group.
  board << Board::MM << g;

  // Restore the unit in centimeters.
  board.setUnit( Board::CM );

  std::cout << radius << std::endl;
  board.scale( 1.0 );
  board.saveEPS( "arrows.eps" /*, Board::A4 */ );
  board.saveFIG( "arrows.fig" );
  board.saveSVG( "arrows.svg" /*, Board::A4 */ );
  exit(0);
}
