/**
 * @file testr_measure.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/03/03
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_trace' <p>
 * Aim: simple test of \ref MeasureOfStraighLines
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/math/MeasureOfStraightLines.h"


using namespace DGtal;
using namespace std;



/**
 * Compute the measure of the unit square [0,1]x[0,1]
 *
 * Expected value : sqrt{2}/2
 **/
void testUnitSquare()
{
    vector<double> a;
    vector<double> b;

    a.push_back(0);
    b.push_back(0);
    a.push_back(1);
    b.push_back(0);
    a.push_back(1);
    b.push_back(1);
    a.push_back(0);
    b.push_back(1);

    MeasureOfStraightLines measure;


    trace.info() << "Measure of the Straight of Lines of the unit square = " << measure.computeMeasure(a,b)<< std::endl;
    trace.emphase() <<"Expected value = 0.707107 (sqrt(2)/2)"<<endl;
}


/**
 * Compute the measure of the unit square [0,1]x[0,1]
 *
 * Expected value : sqrt{2}/2
 **/
void testUnitSquareCentroid()
{
    vector<double> a;
    vector<double> b;

    a.push_back(0);
    b.push_back(0);
    a.push_back(1);
    b.push_back(0);
    a.push_back(1);
    b.push_back(1);
    a.push_back(0);
    b.push_back(1);

    MeasureOfStraightLines measure;

    trace.info() << "Centroid measure of the unit square = (" << measure.computeCentroidA(a,b)
    << ","<<measure.computeCentroidB(a,b)<<")"<<std::endl;
    trace.emphase() <<"Expected value = (0.4142,0.5)"<<endl;
}


int main(int argc, char **argv)
{

    testUnitSquare();
    testUnitSquareCentroid();
    return 0;
}

