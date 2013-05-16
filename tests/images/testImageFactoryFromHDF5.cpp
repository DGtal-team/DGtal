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

/**
 * @file testImageFactoryFromHDF5.cpp
 * @ingroup Tests
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/16
 *
 * @brief A test file for ImageFactoryFromHDF5.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

//#define DEBUG_VERBOSE

#include "hdf5.h"
#include "DGtal/images/ImageFactoryFromHDF5.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ImageFactoryFromHDF5.
///////////////////////////////////////////////////////////////////////////////

#define H5FILE_NAME     "testImageFactoryFromHDF5.h5"
#define DATASETNAME     "IntArray0"

#define NX              5       // dataset dimensions
#define NY              6
#define RANK            2

#define NX_SUB          3       // hyperslab dimensions
#define NY_SUB          4

#define NX_OUT          7       // output buffer dimensions
#define NY_OUT          7
#define NZ_OUT          3
#define RANK_OUT        3

bool writeHDF5()
{
    hid_t       file, dataset;         /* file and dataset handles */
    hid_t       datatype, dataspace;   /* handles */
    hsize_t     dimsf[2];              /* dataset dimensions */
    herr_t      status;
    int         data[NX][NY];          /* data to write */
    int         i, j;

    /*
     * Data  and output buffer initialization.
     */
    for(j = 0; j < NX; j++)
        for(i = 0; i < NY; i++)
            data[j][i] = i + j;
    /*
     * 0 1 2 3 4 5
     * 1 2 3 4 5 6
     * 2 3 4 5 6 7
     * 3 4 5 6 7 8
     * 4 5 6 7 8 9
     */

    /*
     * Create a new file using H5F_ACC_TRUNC access,
     * default file creation properties, and default file
     * access properties.
     */
    file = H5Fcreate(H5FILE_NAME, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    /*
     * Describe the size of the array and create the data space for fixed
     * size dataset.
     */
    dimsf[0] = NX;
    dimsf[1] = NY;
    dataspace = H5Screate_simple(RANK, dimsf, NULL);

    /*
     * Define datatype for the data in the file.
     * We will store little endian INT numbers.
     */
    datatype = H5Tcopy(H5T_NATIVE_INT);
    status = H5Tset_order(datatype, H5T_ORDER_LE);

    /*
     * Create a new dataset within the file using defined dataspace and
     * datatype and default dataset creation properties.
     */
    dataset = H5Dcreate2(file, DATASETNAME, datatype, dataspace,
                        H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    /*
     * Write the data to the dataset using default transfer properties.
     */
    status = H5Dwrite(dataset, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    /*
     * Close/release resources.
     */
    H5Sclose(dataspace);
    H5Tclose(datatype);
    H5Dclose(dataset);
    H5Fclose(file);

    return true;
}

bool readHDF5()
{
    hid_t       file, dataset;         /* handles */
    hid_t       datatype, dataspace;
    hid_t       memspace;
    H5T_class_t t_class;                 /* data type class */
    H5T_order_t order;                 /* data order */
    size_t      size;                  /*
                                        * size of the data element
                                        * stored in file
                                        */
    hsize_t     dimsm[3];              /* memory space dimensions */
    hsize_t     dims_out[2];           /* dataset dimensions */
    herr_t      status;

    int         data_out[NX_OUT][NY_OUT][NZ_OUT]; /* output buffer */

    hsize_t      count[2];              /* size of the hyperslab in the file */
    hsize_t      offset[2];             /* hyperslab offset in the file */
    hsize_t      count_out[3];          /* size of the hyperslab in memory */
    hsize_t      offset_out[3];         /* hyperslab offset in memory */
    int          i, j, k, status_n, rank;

    for (j = 0; j < NX_OUT; j++) {
        for (i = 0; i < NY_OUT; i++) {
            for (k = 0; k < NZ_OUT; k++)
                data_out[j][i][k] = 0;
        }
    }

    /*
     * Open the file and the dataset.
     */
    file = H5Fopen(H5FILE_NAME, H5F_ACC_RDONLY, H5P_DEFAULT);
    dataset = H5Dopen2(file, DATASETNAME, H5P_DEFAULT);

    /*
     * Get datatype and dataspace handles and then query
     * dataset class, order, size, rank and dimensions.
     */
    datatype  = H5Dget_type(dataset);     /* datatype handle */
    t_class     = H5Tget_class(datatype);
    if (t_class == H5T_INTEGER) printf("Data set has INTEGER type \n");
    order     = H5Tget_order(datatype);
    if (order == H5T_ORDER_LE) printf("Little endian order \n");

    size  = H5Tget_size(datatype);
    printf(" Data size is %d \n", (int)size);

    dataspace = H5Dget_space(dataset);    /* dataspace handle */
    rank      = H5Sget_simple_extent_ndims(dataspace);
    status_n  = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
    printf("rank %d, dimensions %lu x %lu \n", rank,
           (unsigned long)(dims_out[0]), (unsigned long)(dims_out[1]));

    /*
     * Define hyperslab in the dataset.
     */
    offset[0] = 1;
    offset[1] = 2;
    count[0]  = NX_SUB;
    count[1]  = NY_SUB;
    status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL,
                                 count, NULL);

    /*
     * Define the memory dataspace.
     */
    dimsm[0] = NX_OUT;
    dimsm[1] = NY_OUT;
    dimsm[2] = NZ_OUT;
    memspace = H5Screate_simple(RANK_OUT,dimsm,NULL);

    /*
     * Define memory hyperslab.
     */
    offset_out[0] = 3;
    offset_out[1] = 0;
    offset_out[2] = 0;
    count_out[0]  = NX_SUB;
    count_out[1]  = NY_SUB;
    count_out[2]  = 1;
    status = H5Sselect_hyperslab(memspace, H5S_SELECT_SET, offset_out, NULL,
                                 count_out, NULL);

    /*
     * Read data from hyperslab in the file into the hyperslab in
     * memory and display.
     */
    status = H5Dread(dataset, H5T_NATIVE_INT, memspace, dataspace,
                     H5P_DEFAULT, data_out);
    for (j = 0; j < NX_OUT; j++) {
        for (i = 0; i < NY_OUT; i++) printf("%d ", data_out[j][i][0]);
        printf("\n");
    }
    /*
     * 0 0 0 0 0 0 0
     * 0 0 0 0 0 0 0
     * 0 0 0 0 0 0 0
     * 3 4 5 6 0 0 0
     * 4 5 6 7 0 0 0
     * 5 6 7 8 0 0 0
     * 0 0 0 0 0 0 0
     */

    /*
     * Close/release resources.
     */
    H5Tclose(datatype);
    H5Dclose(dataset);
    H5Sclose(dataspace);
    H5Sclose(memspace);
    H5Fclose(file);

    return true;
}

bool testSimple()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing simple ImageFactoryFromHDF5");
    
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;

    VImage image(Z2i::Domain(Z2i::Point(0,0), Z2i::Point(3,3)));
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;

    trace.info() << "ORIGINAL image: " << image << endl;

    typedef ImageFactoryFromHDF5<VImage > MyImageFactoryFromHDF5;
    MyImageFactoryFromHDF5 factImage(image);
    
    typedef MyImageFactoryFromHDF5::OutputImage OutputImage;
    
    Z2i::Domain domain1(Z2i::Point(0,0), Z2i::Point(1,1));
    OutputImage *image1 = factImage.requestImage(domain1);
    OutputImage::ConstRange r1 = image1->constRange();
    //cout << "image1: "; std::copy( r1.begin(), r1.end(), std::ostream_iterator<int>(cout,", ") ); cout << endl;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
    trace.beginBlock ( "Testing class ImageFactoryFromHDF5" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << endl;

    bool res = writeHDF5() && readHDF5() && testSimple(); // && ... other tests

    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
