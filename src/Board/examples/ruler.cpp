/**
 * @file   ruler.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Tue Aug 21
 * 
 * @brief  Sample program to check that arrows are correctly drawn.
 *
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 */
/*
 * @file   ruler.cpp
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

int main( int , char *[] )
{
  Board board;
  board.clear( Color(200,200,255) );
  board.setLineWidth(1);

  board << Board::UCentimeter;
  // board << Rectangle( 0, 27.7, 19, 27.7, Color::Black, Color::None, 0.1 );

  Line tiny( 0, 0, 0.2, 0, Color::Black, 0.1 );
  Line small( 0, 0, 0.5, 0, Color::Black, 0.1 );
  Line large( 0, 0, 1.5, 0, Color::Black, 0.1 );

  // Centimeters
  
  board.addDuplicates( small, 200, 0, 0.1 );
  board.addDuplicates( large, 21, 0, 1  );

  board.drawText( 0, 21, "Centimeters" );

  // Inches

  tiny.translate( 2, 0 );
  small.translate( 2, 0 );
  large.translate( 2, 0 );
  board.addDuplicates( tiny, 40, 0, 2.54/4  );
  board.addDuplicates( small, 20, 0, 2.54/2  );
  board.addDuplicates( large, 11, 0, 2.54  );

  board << Text( 2, 24, "Inches", Fonts::CourierBold, 12, Color::Red );
  // Or : board.drawText( 2, 26, "Inches" );
  board << Board::UPoint
	<< Rectangle( board.last<Text>().boundingBox(), Color::Black, Color::None, 0.1 );

  // Inches with a change of units

  tiny.translate( 0.5, 0 );   // 0.5 inch right.
  small.translate( 0.5, 0 );
  large.translate( 0.5, 0 );

  board << Board::UInche;
  // Equivalent to :   board.setUnit( Board::IN );
  // Equivalent to :   board.setUnit( 1.0, Board::IN );
  // Equivalent to :   board.setUnit( 2.54, Board::CM );
  // Equivalent to :   board.setUnit( 254, Board::MM );
  // Etc.

  board.addDuplicates( tiny, 40, 0, 0.25  );
  board.addDuplicates( small, 20, 0, 0.5  );
  board.addDuplicates( large, 11, 0, 1.0  );
  
  board << Board::UCentimeter 
	<< Text( 6.5, 24, "Inches again", Fonts::PalatinoBold, 14, Color::Red );
  board << Board::UPoint
	<< Rectangle( board.last<Text>().boundingBox(), Color::Black, Color::None, 0.1 );

  board.saveEPS( "ruler.eps" /*, Board::A4 */ );
  board.saveFIG( "ruler.fig" /*, Board::A4 */ );
  board.saveSVG( "ruler.svg" /*, Board::A4 */ );
  exit(0);
}
