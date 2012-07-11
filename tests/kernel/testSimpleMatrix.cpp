/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testSimpleMatrix.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/07/10
 *
 * Functions for testing class SimpleMatrix.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SimpleMatrix.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SimpleMatrix.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleMatrix()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing create ..." );
  
  typedef SimpleMatrix<double,3,4> M34d;
  
  M34d m34d;
  trace.info() << m34d<<std::endl;
  
  m34d.setComponent(1,2, 0.5);
  trace.info() << m34d<<std::endl;
  
  nbok += (m34d(1,2) == 0.5) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  
  M34d matrix;
  bool res=true;

  matrix.constant(12.3);
  trace.info() << matrix;
  for(DGtal::Dimension i = 0; i< 3; ++i)
    for(DGtal::Dimension j = 0; j< 4; ++j)
        res = res && (matrix(i,j) == 12.3);
  nbok += res ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "all equals to 12.3" << std::endl;
  

  trace.endBlock();
  
  return nbok == nb;
}

bool testArithm()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  
  typedef SimpleMatrix<double,3,4> M34d;
  typedef SimpleMatrix<double,4,3> M43d;
  typedef SimpleMatrix<double,3,3> M33d;
  
  M34d m34d, two,four;
  M34d m34dbis, resadd, ressub;
  
  two.constant(2);
  four.constant(4);
  
  for(DGtal::Dimension i = 0; i< 3; ++i)
    for(DGtal::Dimension j = 0; j< 4; ++j)
      {
        m34d.setComponent(i,j,i*j);
        m34dbis.setComponent(i,j,i+j);
        resadd.setComponent(i,j,i*j+i+j);
        ressub.setComponent(i,j,i*j-(double)i-(double)j);
      }
  

  trace.info() << m34d <<std::endl;
  trace.info() << m34dbis<<std::endl;
 
  trace.beginBlock ( "Testing add ..." ); 
  nbok += ((m34d + m34dbis) == resadd) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok" << std::endl;
  nbok += ((m34dbis + m34d) == resadd) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok commutative" << std::endl;
  
  M34d other;
  other += m34d;
  nbok += (other == m34d) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok +=" << std::endl;
  
  trace.endBlock();
  
  trace.beginBlock ( "Testing substraction ..." ); 
  nbok += ((m34d - m34dbis) == ressub) ? 1 : 0; 
  nb++;
  trace.info()<<ressub<<std::endl;
  trace.info()<<m34d - m34dbis<<std::endl;
  
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok simple" << std::endl;
  trace.endBlock();
  
  trace.beginBlock ( "Testing scalar product/divide ..." ); 
  nbok += ( (two*2.0) == four) ? 1 : 0; 
  nb++;
  trace.info()<<ressub<<std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << " [2]*2 == [4]" << std::endl;
 
  nbok += ( two == four/2.0) ? 1 : 0; 
  nb++;
  trace.info()<<ressub<<std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << " [2]= [4]/2" << std::endl;
  trace.endBlock();
  

  trace.beginBlock ( "Testing transpose ..." ); 
  M43d transp = m34d.transpose();
  nbok += (transp.transpose() == m34d) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << "ok idem potent" << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing product ..." ); 
 
  M43d one;
  M33d eight33;

  one.constant(1);
  eight33.constant(8);
  trace.info() << two * one<<std::endl;
  nbok += (two * one  == eight33) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << " [2]*[1] = [8]" << std::endl;
  trace.endBlock();


  
  return nbok == nb;

}

bool testColRow()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  DGtal::SimpleMatrix<double,3,4> mat;
  for(DGtal::Dimension i = 0; i< 3; ++i)
    for(DGtal::Dimension j = 0; j< 4; ++j)
      mat.setComponent(i,j,i+j);
  
  trace.beginBlock("Get Row");
  trace.info() << mat <<std::endl;
  DGtal::SimpleMatrix<double,3,4>::RowVector row;
  row = mat.row(1);
  trace.info() << row << std::endl;
  nbok += (row[1] == 2 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << " row value" << std::endl;
  trace.endBlock();

  trace.beginBlock("Get Col");
  DGtal::SimpleMatrix<double,3,4>::ColumnVector col;
  col = mat.column(1);
  trace.info() << row << std::endl;
  nbok += (col[1] == 2 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << " col value" << std::endl;
  trace.endBlock();

  

  trace.beginBlock("Prod Matrix x Row^t");
  //Row vector is a dim 4 vector
  DGtal::SimpleMatrix<double,3,4>::RowVector r(1,2,3,4);
  DGtal::SimpleMatrix<double,3,4>::ColumnVector c;
  c = mat*r;
  DGtal::SimpleMatrix<double,3,4>::ColumnVector expected(20,30,40);
  
  trace.info() << c << std::endl;
  nbok += (c == expected) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << " mat*row^t" << std::endl;
  trace.endBlock();

  return nbok == nb; 
}

bool testCofactor()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef DGtal::SimpleMatrix<double,3,4> MAT;
  MAT mat;
  for(DGtal::Dimension i = 0; i< 3; ++i)
    for(DGtal::Dimension j = 0; j< 4; ++j)
      mat.setComponent(i,j,i+j);
  
  trace.beginBlock("Cofactor tests...");
  trace.info() << mat<<std::endl;
  trace.info() << mat.cofactor() << std::endl;

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SimpleMatrix" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSimpleMatrix() && testArithm() && testColRow() && testCofactor(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
