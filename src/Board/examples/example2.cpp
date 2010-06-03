/**
 * @file   example2.cpp
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
 * @file   example2.cpp
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

  //board.clear( Color(0,120,0) );
  board.setPenColorRGBi( 255, 0, 0 );

  std::vector<Point> points;
  for ( double x = -1.0; x < 1.0; x += 0.02 ) {
    points.push_back( Point( x, sin(x*6.28) ) );
  }
  board.setLineWidth( 0.5 );
  board.setPenColorRGBi( 0, 0, 255 );
  board.drawPolyline( points );

  board.setPenColorRGBi( 255, 0, 0 );
  for ( double x = -1.0; x < 1.1; x += 0.1 ) {
    board.setFillColorRGBi( 255, 0, 0 );
    board.setPenColorRGBi( 0, 0, 255 );
    board.drawCircle( x, 0.0, 0.1 );
  }
  board.fillCircle( 0.0, 0.0, 0.01 );
  
  board.setPenColorRGBi( 0, 0, 0 );
  board.setFillColor( Color::None );

  board.setFillColorRGBf( 0.5f, 1.0f, 0, 0.25 ).drawEllipse( 0, -1, 0.8, 0.3 );
  
  board.setLineJoin( Shape::RoundJoin );
  board.setLineWidth( 8 ).setPenColor( Color::Black ).setFillColorRGBi( 0, 0, 255, 50 ).drawRectangle( -1, 0, 1, 0.5 );

  board.setLineJoin( Shape::MiterJoin );
  board.setLineWidth( 2 ).setPenColor( Color::Red ).drawTriangle( -0.5, -0.25, 0, 0.25, 0.5, -0.25 );
  board.setLineWidth( 1.5 ).setPenColorRGBi( 255, 100, 0 ).drawLine( -0.5, -0.27, 0.5, -0.27 );


  board.setPenColor( Color::Black ).setFont( Fonts::HelveticaBold, 12.0 ).drawText( 0, 0.5, "Hello world!" );
  board << board.last<Text>().rotated( 45 * Board::Degree );
  board << board.last<Text>().rotated( 45 * Board::Degree );
  board << board.last<Text>().rotated( 45 * Board::Degree );
  board << board.last<Text>().rotated( 45 * Board::Degree );

  board.saveEPS( "draw2_A4.eps", Board::A4 );
  board.saveFIG( "draw2_A4.fig", Board::A4 );
  board.saveSVG( "draw2_A4.svg", Board::A4 );

  exit(0);
}
