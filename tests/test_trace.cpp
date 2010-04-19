/**
 * @file testr_trace.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2009/12/13
 *
 * This file is part of the DGtal library
 */

 /**
   * Description of test_trace' <p>
   * Aim: simple test of \ref Trace
   */


#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>

#include "DGtal/base/Config.h"
#include "DGtal/base/Common.h"


using namespace DGtal;
using namespace std;


void testSimple()
{
  trace.info()<< "This is an Info trace"<<endl;
  trace.warning()<< "This is an warning trace"<<endl;
  trace.error()<< "This is an Error trace"<<endl;
  trace.emphase()<< "This is an Emphased trace"<<endl;
  cerr<<endl;
}

void testIndent()
{
   long tmp=0;

   trace.info()<< "This is an Info trace, level 0"<<endl;
   trace.beginBlock("FirstMethod");
   trace.info()<< "This is an Info trace, level 1"<<endl;
   trace.info()<< "This is an Info trace, level 1"<<endl;
   trace.beginBlock("SecondMethod");
   trace.warning()<< "This is an Warning trace, level 2"<<endl;
   trace.warning()<< "This is an Warning trace, level 2"<<endl;
   trace.info()<< "This is an Info trace, level 2"<<endl;
   trace.error()<< "This is an Error trace, level 2 (followed by a loop)"<<endl;

   for(unsigned int i=0 ; i< 4334450; i++)
    tmp = cos(tmp+i);

   trace.endBlock();
   trace.info()<< "This is an Info trace, level 1 (followed by another loop)"<<endl;

   for(unsigned int i=0 ; i< 4334450; i++)
    tmp = cos(tmp+i);

   trace.endBlock();
   trace.info()<< "This is an Info trace, level 0"<<endl<<endl;
 }

/**
  * We test the Trace class on file stream.
  *
  * a file "example.txt" must have been created with the traces.
  */
void testFileStream()
{
  trace.beginBlock("testFileStream");
  trace.info() << "Checking the filestream output.. Please check the 'example.txt' file"<<endl;

  ofstream myfile;
  myfile.open ("example.txt");

  TraceWriterFile traceWriterFile(myfile);
  Trace t2(traceWriterFile);

  t2.info()<< "This is an Info trace"<<endl;
  t2.warning()<< "This is an warning trace"<<endl;
  
  t2.error()<< "This is an Error trace"<<endl;
  t2.emphase()<< "This is an Emphased trace"<<endl;
  
  t2.beginBlock("FirstMethod");
  t2.info()<< "This is an Info trace, level 1"<<endl;
  t2.info()<< "This is an Info trace, level 1"<<endl;
  t2.endBlock();

  myfile.close();

  trace.endBlock();
}


int main()
{
  testSimple();
  testIndent();
  testFileStream();
  return 0;
}
