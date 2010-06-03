/**
 * @file   graph.cpp
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
 * @file   graph.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
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

#include "Board.h"

using namespace std;
using namespace LibBoard;

const int HALF_RAND = RAND_MAX/16;

const double dy = 10;

int coordinate( int width ) {
  return 1 + (int) (width * (rand() / (RAND_MAX + 1.0)));
}

int main( int , char *[] )
{
  Board board;
  board.clear( Color(200,255,200) );
  srand( static_cast<unsigned int>( time( 0 ) ) );

  vector< pair<int,int>  > points;
  vector< pair<int,int>  >::iterator i1, i2, end;

  int n = 22;
  while ( n-- ) {
    points.push_back( make_pair( coordinate(80), coordinate(60) ) );
  }

  end = points.end();
  i1 = points.begin();
  while ( i1 != end ) {
    i2 = i1;
    Color pen( coordinate(255), coordinate(255), coordinate(255) );
    while ( i2 != end ) {
      if ( i1 != i2 ) {
	board << Arrow( i1->first, i1->second, i2->first, i2->second, pen, pen, 0.1 );
	double mod = sqrt( static_cast<double>( (i2->first-i1->first)*(i2->first-i1->first)
						+(i2->second-i1->second)*(i2->second-i1->second) ) );
	board << Arrow( i1->first,
			i1->second, 
			i1->first + ((i2->first-i1->first)/mod)*((mod>8)?8:mod), 
			i1->second  + ((i2->second-i1->second)/mod)*((mod>8)?8:mod),
			pen, pen, 0.1 ).translated(80,0);
      }
      ++i2;
    }
    ++i1;
  }

  i1 = points.begin();
  while ( i1 != end ) {
    board << Circle( i1->first, i1->second, 0.5, Color::Black, Color::Black, 0 );
    ++i1;
  }

  board.saveEPS( "tree.eps", 100, 100 );
  board.saveFIG( "tree.fig", 100, 100 );
  board.saveSVG( "tree.svg", 100, 100 );

  exit(0);
}
