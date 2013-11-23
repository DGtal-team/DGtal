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
#include <vector>

using namespace std;

struct T {};
void f( T&& t, int& i ) { i += 1; }
void f( T& t, int& i )  { i += 2; }

struct U {
  U( const std::vector<int> & v ) : myV( v ) {
    former_size = v.size();
  }
  U( std::vector<int> && v ) : myV( std::move( v ) ) {
    former_size = v.size();
  }
  int former_size;
  std::vector<int> myV;
};

int main()
{
  // Checking rvalue reference.
  int i = 0;
  T t;
  f( t, i ); f( T(), i );
  bool ok1 = ( i == 3 );
  std::cout << "rvalue reference is " << ( ok1 ? "ok" : "ko" ) << std::endl;

  // initializer list are not in my clang++ 3.0
  std::vector<int> v;
  v.push_back( 1 ); v.push_back( 1 );
  for ( int j = 0; j < 10; ++j ) v.push_back( v.back() + v[ v.size() - 2 ] );

  // Checking std::move
  U u1( v );
  std::cout << "u1.former_size=" << u1.former_size << " == u1.size()=" << u1.myV.size() << std::endl;
  U u2( std::vector<int>( 5 ) );
  std::cout << "u2.former_size=" << u2.former_size << " != u2.size()=" << u2.myV.size() << std::endl;
  bool ok2 = u2.former_size == 0;
  return ( ok1 && ok2 )? 0 : 1;
}
