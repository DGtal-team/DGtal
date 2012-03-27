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

#include <iostream>

template<typename T>
void f(std::initializer_list<T> init)
{
  std::cout<<"{";
  for (const T *p = init.begin (); p != init.end (); ++p)
    std::cout<<*p<<", ";
  std::cout<<"}"<<std::endl;
}

int main()
{
  f({1,2,3,4});
  f({1.1,2.2,3.3,4.4});
  f({'a','b','c','d'});
  f({"toto","tata","titi"});
  return 1;
}
