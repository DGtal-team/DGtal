#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <list>
#include <math.h>
#include <assert.h>
#include "cone.h"
#include "backPath.h"

using namespace std;
/************************************* Ostream Operator redefinition*****************/

std::ostream &operator << (ostream &of, const vector <int> &pixel)
{
  //of << "( ";
  copy(pixel.begin(), pixel.end(), ostream_iterator<int>(of, "\n"));
  //of << ")";
  return of;
  
}

std::ostream &operator << (ostream &of, const vector <double> &vectdouble)
{
  //of << "( ";
  copy(vectdouble.begin(), vectdouble.end(), ostream_iterator<double>(of, " "));
  //of << ")";
  return of;
  
}

ostream &operator << (ostream &of, const vector < vector  <int>  > &p)
{
  
  for(unsigned int i=0;i<p.size();i++)
    of << p[i] << "--";
  of << endl;
  //copy(p.begin(), p.end(), ostream_iterator<vector  <int>  >(of, "\n"));
  return of;
}

ostream &operator << (ostream &of, const cone &c)
{
  if(c.inf)
    of << "[Cone infini]" << endl;
  else
    of << "[Cone min = " << c.min << " max = " << c.max << "]" << endl;
  
  return of;
}
 

ostream &operator << (ostream &of, const occulter_attributes &occ_at)
{
  of << "angle min " << occ_at.angle_min << endl;
  of << "angle max " << occ_at.angle_max << endl;

}

ostream &operator << (ostream &of, const backPath &bp)
{
  of << "Occulters" << endl;
  for(occulter_list::const_iterator iter = bp.occulters.begin();iter!=bp.occulters.end();++iter)	
    { 
      of << "Occulter index " << iter->first << endl;
      of << "Occulter attributes " << iter->second << endl;
    }
  
  return of;
}
