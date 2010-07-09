#include <iostream>

template<typename T>
void f(std::initializer_list<T> init)
{
  std::cout<<"{";
  for (const T *p = init.begin (); p != init.end (); ++p)
    std::cout<<*p<<", ";
  std::cout<<"}"<<std::endl;
}
d
int main()
{
  f({1,2,3,4});
  f({1.1,2.2,3.3,4.4});
  f({'a','b','c','d'});
  f({"toto","tata","titi"});
  return 1;
}
