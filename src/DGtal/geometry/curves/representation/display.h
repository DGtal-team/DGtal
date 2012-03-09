#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "cone.h"
#include "backpath.h"
using namespace std;

//template <typename T>
//vector <T> operator + (const vector <T > vertexA, const vector <T > vertexB);

//template <typename T>
//vector <T> operator - (const vector <T > vertexA, const vector <T > vertexB);

//bool operator== (const vector <mpq_class > aVertexA, const vector <mpq_class > aVertexB);

//std::ostream& operator << (ostream &of, const mpq_class &rat);

//ostream &operator << (ostream &of, const vector <mpq_class> &vectrat);

//ostream &operator << (ostream &of, const vector <mpz_class> &vectint);

ostream &operator << (ostream &of, const vector <int> &pixel);

template <typename T>
ostream &operator << (ostream &of, const list < vector  <T>  > &preimage);

//ostream &operator << (ostream &of, const vector <list < vector <mpq_class>  > > &preimages);

ostream &operator<< (ostream &of, const vector < vector  <int>  > &p);


//ostream &operator<< (ostream &of, const vector < vector  <mpq_class>  > &p);
ostream &operator << (ostream &of, const vector <double> &vectdouble);

ostream &operator << (ostream &of, const cone &c);

ostream &operator << (ostream &of, const occulter_attributes &occ_at);

ostream &operator << (ostream &of, const backpath &bp);

#endif
