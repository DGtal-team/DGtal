#include <iostream>
#include <vector>

using namespace std;


int main()
{
  auto x = 7;
  auto s= "blam";

  std::vector<int> t(20);

  for (auto p = t.begin(); p!=t.end(); ++p) 
    std::cout << *p << "\n";
  
  return 1;
}
