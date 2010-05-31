/**
 * @file   clipping.cpp
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
#include <vector>
using namespace LibBoard;

int main( int, char *[] )
{
  Board board;
  Group g;

  Path clip;
  clip << Point( -10, 5 );
  clip << Point( 0, 5 );
  clip << Point( 10, 0 );
  clip << Point( 0, -5 );
  clip << Point( -10, -5 );
  g.setClippingPath( clip );

  g << Circle( 0, 0, 10, Color::Black, Color::Red, 1.0 );
  
  // board << g;
  // board << g.scaled(0.8,1.5).translated( 20, 0 );

  board.addDuplicates( g, 10, 18, 0, 0.9, 1.05 );
  board.addDuplicates( g, 10, 18, -18, 1.2, 1 );

  Group cross;
  clip.clear();
  clip << Point( 5, 15 );
  clip << Point( 5, 5 );
  clip << Point( 15, 5 );
  clip << Point( 15, -5 );
  clip << Point( 5, -5 );
  clip << Point( 5, -15 );
  clip << Point( -5, -15 );
  clip << Point( -5, -5 );
  clip << Point( -15, -5 );
  clip << Point( -15, 5 );
  clip << Point( -5, 5 );
  clip << Point( -5, 15 );

  cross.setClippingPath( clip );

  Circle cropedC( 0, 0, 10, Color::Black, Color( 100, 255, 100 ), 1.0 );
  cross << cropedC;
  
  //board << cross.translated( 30, -30 ).scaled( 1.2 );
  board << cross.scaled( 3 );

  board.addDuplicates( cross.translated( 0, -60 ).scaled( 2 ),
		       18,
		       0, 0,
		       0.85, 0.85,
		       0.1 );

  board.saveEPS( "clipping.eps", 210, 297 );
  board.saveSVG( "clipping.svg" );                        // Viewport == BoundingBox
  board.saveSVG( "clipping_A4.svg", Board::A4, 50 );      // Centered on an A4 paper, with a 50mm margin.
  board.scale(10);
  board.saveSVG( "clipping_x10.svg" );                    // Scaled 10 times and saved with Viewport == BoundingBox
  exit(0);
}
