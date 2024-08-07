
#ifndef ROTATIONTABLES_H
#define ROTATIONTABLES_H
#include "GAVector.h"
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
namespace DGtal {
    namespace functions {

        template<typename TSpace, typename TInputValue = typename TSpace::Point>
        std::vector<std::vector<int>> loadOTCTable(const std::string& tableFolderPath,const int rwidth) {
            std::ostringstream ot;
            ot << tableFolderPath <<"OT-" << rwidth << "-circles-l2.txt";
            std::string tname = ot.str();
            std::vector< int> Csize;
            std::vector< std::vector< int > > table;
            std::ifstream tinput( tname.c_str() );
            if(tinput.fail()) {
                std::cerr << "OTC file not found" <<std::endl;
            }else{
                bool size_mode = true;
                int a     = 0;
                int nb_ok = 0;
                while ( ! tinput.eof() )
                {
                    int t;
                    double alpha;
                    std::string str;
                    std::getline( tinput, str );

                    // ignore comments
                    if ( str.empty() || str[ 0 ] == '#' ) continue;
                    if ( size_mode ) {
                        std::istringstream is( str );
                        while ( is >> t ) Csize.push_back( t );
                        // std::cout << "Read #C=" << Csize.size() << std::endl;
                        size_mode = false;
                    } else {
                        std::istringstream is( str );
                        std::vector< int > mapping;
                        is >> alpha;
                        int ra = int( round( alpha*180.0/M_PI ) );
                        if ( a == ra ) nb_ok += 1;
                        mapping.push_back( 0 ); // add 0 circle
                        while ( is >> t ) mapping.push_back( t );
                        // std::cout << " #M=" << mapping.size() << std::endl;
                        table.push_back( mapping );
                        a += 1;
                    }
                }
            }
            tinput.close();

            return table;
        }




        /// parse line
        ///
        // Function to parse a single coordinate from a string
        template<typename TSpace, typename TInputValue = typename TSpace::Point>
        std::vector<GAVector<TSpace>> parseCoordinates(const std::string& coordinatesStr) {
            int x, y;
            std::istringstream iss(coordinatesStr);
            char delim;
            std::vector<GAVector<TSpace>> gavecs;

            iss >> delim;
            // Parse (x1,y1),(x2,y2),...,(xn,yn), n is even
            while (iss >> x >> delim >> y) {
                gavecs.push_back(GAVector<TSpace>({x, y}));
                // std::cout << "x="<<x<<",y="<<y<<std::endl;
                if (iss.peek() == ',' || iss.peek() == ')') {
                    iss >> delim >> delim>> delim;
                } else {
                    break;
                }
            }
            return gavecs;
        }

        template<typename TSpace, typename TInputValue = typename TSpace::Point>
        std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> loadBijectiveRotationTable(std::string fileStr, const size_t n, const size_t kmax) {
        //std::string fileStr = (("../precomputedTables/"+(std::to_string(n))+"reflectionsUnique_kmax_"+std::to_string(kmax)+".txt"));  //"../precomputedTables/2reflectionsUnique_kmax_15.txt";//
        const char* filename =fileStr.c_str();

        // Open bijective file
        int fileDesc = open(filename, O_RDONLY);
        if (fileDesc == -1) {
            std::cerr << "Error opening file: " << strerror(errno) << std::endl;
            return {};
        }

        // Get the size of the file
        struct stat fileInfo;
        if (fstat(fileDesc, &fileInfo) == -1) {
            std::cerr << "file information error : " << strerror(errno) << std::endl;
            close(fileDesc);
            return {};
        }

        // Map the file into memory (file size is approx. 2GB)
        char* fileData = static_cast<char*>(mmap(NULL, fileInfo.st_size, PROT_READ, MAP_PRIVATE, fileDesc, 0));
        if (fileData == MAP_FAILED) {
            std::cerr << "Error mapping file to memory: " << strerror(errno) << std::endl;
            close(fileDesc);
            return {};
        }

        // Read lines from the memory-mapped
        size_t start = 0;
        bool isFirstLine = true;
        int kmaxFile=0;

        std::vector<std::pair<std::vector<GAVector<TSpace>>,GAVector<TSpace>>> vecCompositionBijective;
        for (size_t i = 0; i < fileInfo.st_size; ++i) {
            if (fileData[i] == '\n' || i == fileInfo.st_size - 1) {
                // Extract coordinates from the line
                std::string line(fileData + start, i - start + 1);
                if (isFirstLine) {
                    // Check if it is the first line and extract the number after "kmax="
                    size_t pos = line.find("kmax=");
                    if (pos != std::string::npos) {
                        if (sscanf(line.c_str() + pos + 5, "%d", &kmaxFile) == 1) {
                            std::cout << "kmax: " << kmaxFile << std::endl;
                        }
                    }
                    isFirstLine = false;
                } else {
                    std::vector<GAVector<TSpace>> coordinates;

                    // Parse coordinates from the line
                    std::string coordinateStr;
                    coordinates = functions::parseCoordinates<TSpace>(line);
                    // Store the last coordinate separately
                    GAVector<TSpace> last = coordinates.back();
                    // std::cout << "last=()"<<last.x<<","<<last.y<<")"<<std::endl;
                    coordinates.pop_back();  // Exclude the last coordinate from the main vector
                    vecCompositionBijective.push_back(std::make_pair(coordinates,last));
                }
                // Update the start position for the next line
                start = i + 1;
            }
        }
        // display(vecCompositionBijective);

        // Don't forget to unmap the memory and close the file when you're done
        munmap(fileData, fileInfo.st_size);
        close(fileDesc);
        std::cout << "load done .."<<std::endl;
        return vecCompositionBijective;
        }



    }
}

#endif //ROTATIONTABLES_H
