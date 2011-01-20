#include <iostream>
#include <DGtal/base/Common.h>
#include <boost/program_options/option.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

namespace po = boost::program_options;


int main(int argc, char **argv)
{
  // Declare the supported options.
  po::options_description desc("DGtal Config command line.\n Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("version,v", "get the current DGtal version")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    
  
  if (vm.count("help"))
    {
      cout << desc << "\n";
      return 1;
    }
  
  if (vm.count("version")) 
    {
      cout << DGTAL_VERSION << endl;;
      return 1;
    } 
  
  return 0;
}

