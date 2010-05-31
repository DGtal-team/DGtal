/**
 * @file   example3.cpp
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
 * @file   example3.cpp
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

  board.clear( Color(0,120,0) );
  board.setPenColorRGBi( 255, 0, 0 );

  board.fillGouraudTriangle( -50, 100, Color( 255, 0, 0 ), 
			     0, 100, Color( 0, 255, 0 ),
			     -30, 130, Color( 0, 120, 255 ) );
  
  board.saveEPS( "draw3.eps", 210, 297 );
  
  board.saveEPS( "draw3_15x10.eps", 210, 297, 25 );
  

  board.saveFIG( "draw3.fig" );
  board.saveFIG( "draw3_A4.fig", Board::A4 ); 

  board.saveSVG( "draw3.svg" );                        // Viewport == BoundingBox
  board.saveSVG( "draw3_A4.svg", Board::A4, 50 );      // Centered on an A4 paper, with a 50mm margin.
  board.scale(10);
  board.saveSVG( "draw3_x10.svg" );                    // Scaled 10 times and saved with Viewport == BoundingBox

  exit(0);
}
