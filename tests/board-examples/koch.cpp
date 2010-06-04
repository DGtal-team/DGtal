/**
 * @file   koch.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief   Draw a random complete graph
 *
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 */
/*
 * @file   koch.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * 
 * @brief   Draw a random complete graph
 */
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <utility>
#include "Board/Board.h"
using namespace std;
using namespace LibBoard;

const int HALF_RAND = RAND_MAX/16;
const double dy = 10;

int coordinate( int width ) {
  return 1 + (int) (width * (rand() / (RAND_MAX + 1.0)));
}

void Koch( ShapeList & board, const Color & color,  Point p1, Point p2, int depth ) { 
  if ( depth <= 0 ) return;
  
  Point v = p2 - p1;
  Point a = p1 + ( v / 3.0 );
  Point b = p1 + 2 * ( v / 3.0 );
  Point c = b.rotated( 60 * Board::Degree, a );

  Color col = color;
  
   col.setRGBi( static_cast<unsigned char>( std::floor( col.red() * 0.75 + 0.5 ) ),
		static_cast<unsigned char>( std::floor( col.green() * 0.75 + 0.5 ) ),
		static_cast<unsigned char>( std::floor( col.blue() * 0.75 + 0.5 ) ) );
  
  board << Triangle( a, c, b, Color::None, color, 0.0 );
  Koch( board, col, p1, a, depth-1 );
  Koch( board, col, a, c, depth-1 );
  Koch( board, col, c, b, depth-1 );
  Koch( board, col, b, p2, depth-1 );  
}

int main( int , char *[] )
{
  Board board;
  board.clear( Color(200,255,200) );

  Point a( -10, 0 );
  Point c( 10, 0 );
  Point b = c.rotated( 60 * Board::Degree, a );  

  Group aux;
  Koch( aux, Color::Green, a, b, 5 );
  Koch( aux, Color::Green, b, c, 5 );
  Koch( aux, Color::Green, c, a, 5 );
  aux << Triangle( a, b, c, Color::None, Color::Green, 0 );

  board << aux;

  int n = 4;
  while ( n-- ) {
    board.dup().last().scale( 0.5 );    
    board.last()
      .translate( 0.25 * board.last().bbox().width, board.last().bbox().height * 1.5 )
      .rotate( -20 * Board::Degree );
  }

  board << Line( board.bbox().left, board.last().bbox().top - 1, 
		 board.bbox().left + board.bbox().width, board.last().bbox().top - 1, 
		 Color::Blue, 1 );
  board << Text( -10, board.last().bbox().top + 0.3, "LibBoard Koch's because LibBoard rocks...",
		 Fonts::Helvetica, "NeverExists, 'Monotype Corsiva', Verdana, Arial",  24, Color::Blue );  
  board.dup().last().rotate( -90 * Board::Degree ).translate( 0, -0.5 );

  board.saveFIG( "koch.fig", 200, 200 );
  board.saveSVG( "koch.svg", 300, 600 );  
  board.saveEPS( "koch.eps", Board::A4 );
  exit(0);
}


