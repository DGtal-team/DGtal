
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


#pragma once

/**
* @file Rotationtables.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/08
 *
 * This file is part of the DGtal library.
 */

#if defined(ROTATIONTABLES_RECURSES)
#error Recursive header files inclusion detected in Rotationtables.h
#else // defined(ROTATIONTABLES_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ROTATIONTABLES_RECURSES

#if !defined ROTATIONTABLES_h
/** Prevents repeated inclusion of headers. */
#define ROTATIONTABLES_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
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
        /**
         * \brief Rotation table loading : for OTC and RBDR
         * @tparam TSpace a 2 dimensional space.
         * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
        */
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


        template<typename TSpace, typename TInputValue = typename TSpace::Point>
        std::vector<std::string> parselineWithSeparator(const std::string& coordinatesStr,const char delimiter) {
            std::vector<std::string> substrings;
            std::istringstream iss(coordinatesStr);

            std::string substring;
            while(std::getline(iss,substring,delimiter)) {
                std::cout << "substring="<<substring<<std::endl;
                substrings.push_back(substring);
            }
            return substrings;
        }


        /// parse line of CBDR tables
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
        std::vector<std::tuple<std::vector<GAVector<TSpace>>,double,double>> loadFastCBDRTable(std::string fileStr) {
            const char* filename =fileStr.c_str();
            std::cout << "filename="<<filename<<std::endl;
            // Open bijective file
            int fileDesc = open(filename, O_RDONLY);
            if (fileDesc == -1) {
                std::cerr << "Fast table : Error opening file: " << strerror(errno) << std::endl;
                std::cerr << "Fast table : Error file name= " << filename << std::endl;
                return {};
            }

            struct stat fileInfo;
            if (fstat(fileDesc, &fileInfo) == -1) {
                std::cerr << "file information error : " << strerror(errno) << std::endl;
                close(fileDesc);
                return {};
            }
            // Map the file into memory (file size can be ~200MB for kmax=30, n=4)
            char* fileData = static_cast<char*>(mmap(NULL, fileInfo.st_size, PROT_READ, MAP_PRIVATE, fileDesc, 0));
            if (fileData == MAP_FAILED) {
                std::cerr << "Error mapping file to memory: " << strerror(errno) << std::endl;
                close(fileDesc);
                return {};
            }

            // Read lines from the memory-mapped
            size_t start = 0;

            std::vector<std::tuple<std::vector<GAVector<TSpace>>,double,double>> vecCompositionBijective_With_Errors;
            std::cout << "file info size="<<fileInfo.st_size<<std::endl;
            for (size_t i = 0; i < fileInfo.st_size; ++i) {
                std::string linebefore(fileData + start, i - start + 1);
                std::cout << "line before="<<linebefore<<std::endl;
                if (fileData[i] == '\n' || i == fileInfo.st_size - 1) {
                    // Extract coordinates from the line
                    std::string line(fileData + start, i - start + 1);
                    std::cout << "line="<<line<<std::endl;
                    auto vectorsAndLinfLcont = parselineWithSeparator<TSpace,TInputValue>(line,';');
                    std::cout <<"1="<<vectorsAndLinfLcont[0]<<std::endl;
                    std::cout <<"2="<<vectorsAndLinfLcont[1]<<std::endl;
                    std::cout <<"3="<<vectorsAndLinfLcont[2]<<std::endl;

                    std::vector<GAVector<TSpace>> coordinates;

                    // Parse coordinates from the line
                    std::string coordinateStr;
                    coordinates = functions::parseCoordinates<TSpace>(vectorsAndLinfLcont[0]);
                    std::cout <<"fast : coord=";
                    for(auto gavec : coordinates)
                        std::cout <<gavec.my_gavec <<" ";
                    std::cout <<std::endl;
                    std::cout <<"fast two"<<vectorsAndLinfLcont[1]<<std::endl;
                    std::cout <<"fast three"<<vectorsAndLinfLcont[2]<<std::endl;
                    vecCompositionBijective_With_Errors.push_back(std::make_tuple(coordinates,std::stod(vectorsAndLinfLcont[1]),std::stod(vectorsAndLinfLcont[2])));
                    start = i + 1;
                }
            }

            munmap(fileData, fileInfo.st_size);
            close(fileDesc);
            std::cout << "load done .."<<std::endl;

            return vecCompositionBijective_With_Errors;
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

        // Map the file into memory (file size can be ~200MB for kmax=30, n=4)
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
                    coordinates.pop_back();  // Exclude the last coordinate from the main vector
                    vecCompositionBijective.push_back(std::make_pair(coordinates,last));
                }
                start = i + 1;
            }
        }

        munmap(fileData, fileInfo.st_size);
        close(fileDesc);
        std::cout << "load done .."<<std::endl;
        return vecCompositionBijective;
        }

    }
}


#endif //ROTATIONTABLES

#undef ROTATIONTABLES_RECURSES
#endif // else defined(ROTATIONTABLES_RECURSES)
