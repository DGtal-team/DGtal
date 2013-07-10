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
 * @file exampleImageFactoryFromHDF5.cpp
 * @ingroup Examples
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/25
 *
 * @brief An example file for ImageFactoryFromHDF5.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/readers/HDF5Reader.h"
#include "DGtal/images/ImageFactoryFromHDF5.h"
#include "DGtal/images/ImageCache.h"
#include "DGtal/images/TiledImage.h"

#include "ConfigExamples.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

#define H5FILE_NAME_3D_TILED    "exampleImageFactoryFromHDF5_TILED_3D.h5"

#define DATASETNAME_3D_TILED    "UInt8Array3D_TILED"
#define NX_3D_TILED             500//1980       // dataset dimensions
#define NY_3D_TILED             500//1980
#define NZ_3D_TILED             500//400
#define RANK_3D_TILED           3

bool writeHDF5_3D_TILED()
{
    trace.beginBlock("Example : writeHDF5_3D (3D)");
    
      trace.info() << "begin" << endl;
      
      hid_t               file, dataset;                                  // file and dataset handles
      hid_t               datatype, dataspace;                            // handles
      hsize_t             dimsf[RANK_3D_TILED];                           // dataset dimensions
      herr_t              status;
      //DGtal::uint8_t      data[NZ_3D_TILED][NY_3D_TILED][NX_3D_TILED];    // data to write
      DGtal::uint8_t      *data;
      int                 i, j, k;
      
      data = (DGtal::uint8_t*)malloc(NZ_3D_TILED*NY_3D_TILED*NX_3D_TILED * sizeof(DGtal::uint8_t));

      // Data  and output buffer initialization.
      for(k = 0; k < NZ_3D_TILED; k++)
        for(j = 0; j < NY_3D_TILED; j++)
          for(i = 0; i < NX_3D_TILED; i++)
            if (i>=15 && j>=15 && k>=15 && i<35 && j<35 && k<35)
              data[k*NY_3D_TILED*NX_3D_TILED + j*NX_3D_TILED + i] = 1;
            else
              data[k*NY_3D_TILED*NX_3D_TILED + j*NX_3D_TILED + i] = 0;

      /*
      * Create a new file using H5F_ACC_TRUNC access,
      * default file creation properties, and default file
      * access properties.
      */
      file = H5Fcreate(H5FILE_NAME_3D_TILED, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

      // Describe the size of the array and create the data space for fixed size dataset.
      dimsf[0] = NZ_3D_TILED;
      dimsf[1] = NY_3D_TILED;
      dimsf[2] = NX_3D_TILED;
      dataspace = H5Screate_simple(RANK_3D_TILED, dimsf, NULL);

      /*
      * Define datatype for the data in the file.
      */
      datatype = H5Tcopy(H5T_NATIVE_UINT8);
      status = H5Tset_order(datatype, H5T_ORDER_LE);

      /*
      * Create a new dataset within the file using defined dataspace and
      * datatype and default dataset creation properties.
      */
      dataset = H5Dcreate2(file, DATASETNAME_3D_TILED, datatype, dataspace,
                          H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

      // Write the data to the dataset using default transfer properties.
      status = H5Dwrite(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

      // Close/release resources.
      H5Sclose(dataspace);
      H5Tclose(datatype);
      H5Dclose(dataset);
      H5Fclose(file);
      
      free(data);
      
      trace.info() << "end" << endl;
    
    trace.endBlock();

    return true;
}

bool exampleTiledImage3D()
{
    int i, j, k;

    trace.beginBlock("Example : TiledImage with ImageFactoryFromHDF5 (3D)");
    
      typedef ImageContainerBySTLVector<Z3i::Domain, DGtal::uint8_t> ImageV;
      typedef ImageV::Domain DomainV;
      typedef ImageV::Point PointV;
      DomainV domainV(PointV(0,0,0), PointV(NX_3D_TILED-1, NY_3D_TILED-1, NZ_3D_TILED-1));
      ImageV imageV(domainV);
      
      for(k = 0; k < NZ_3D_TILED; k++)
        for(j = 0; j < NY_3D_TILED; j++)
          for(i = 0; i < NX_3D_TILED; i++)
            if (i>=15 && j>=15 && k>=15 && i<35 && j<35 && k<35)
              imageV.setValue(PointV(i,j,k), 1);
            else
              imageV.setValue(PointV(i,j,k), 0);

      typedef ImageSelector<Z3i::Domain, DGtal::uint8_t>::Type Image;

      typedef ImageFactoryFromHDF5<Image> MyImageFactoryFromHDF5;
      MyImageFactoryFromHDF5 factImage(H5FILE_NAME_3D_TILED, DATASETNAME_3D_TILED);

      typedef MyImageFactoryFromHDF5::OutputImage OutputImage;
      
      //typedef ImageCacheReadPolicyFIFO<OutputImage, MyImageFactoryFromHDF5> MyImageCacheReadPolicyFIFO;
      typedef ImageCacheReadPolicyLAST<OutputImage, MyImageFactoryFromHDF5> MyImageCacheReadPolicyLAST;
      typedef ImageCacheWritePolicyWT<OutputImage, MyImageFactoryFromHDF5> MyImageCacheWritePolicyWT;
      //MyImageCacheReadPolicyFIFO imageCacheReadPolicyFIFO(factImage, 3);
      MyImageCacheReadPolicyLAST imageCacheReadPolicyLAST(factImage);
      MyImageCacheWritePolicyWT imageCacheWritePolicyWT(factImage);
      
      typedef TiledImage<Image, MyImageFactoryFromHDF5, MyImageCacheReadPolicyLAST, MyImageCacheWritePolicyWT> MyTiledImage;
      //BOOST_CONCEPT_ASSERT(( CImage< MyTiledImage > ));
      MyTiledImage tiledImage(factImage, imageCacheReadPolicyLAST, imageCacheWritePolicyWT, 50);
      MyTiledImage tiledImage1block(factImage, imageCacheReadPolicyLAST, imageCacheWritePolicyWT, 1);
      
      typedef MyTiledImage::OutputImage OutputImage;
      OutputImage::Value aValue;
      
      trace.info() << tiledImage1block << std::endl;

      int cpt=0;
      trace.beginBlock("Counting ones in the original image (domain)");
      for(Image::Domain::ConstIterator it = imageV.domain().begin(), itend = imageV.domain().end();
          it != itend; ++it)
        if (imageV( *it ) == 1)
          cpt++;
      trace.info() << "imageV cpt: " << cpt << endl;
      trace.endBlock();

      cpt=0;
      trace.beginBlock("Counting ones in the original image (range)");
      for(Image::Range::ConstIterator it = imageV.range().begin(), itend = imageV.range().end();
          it != itend; ++it)
        if (( *it ) == 1)
          cpt++;
      trace.info() << "imageV cpt: " << cpt << endl;
      trace.endBlock();
          
      cpt=0;
      trace.beginBlock("Counting ones in the tiled image - 1x1x1 blocks - LAST/WT ");
      for(Image::Domain::ConstIterator it = tiledImage1block.domain().begin(), itend = tiledImage1block.domain().end();
          it != itend; ++it)
        if (tiledImage1block(*it) == 1)
          cpt++;         
      trace.info() << "Cpt: " << cpt << endl;
      trace.endBlock();
      
      cpt=0;
      trace.beginBlock("Counting ones in the tiled image - 5x5x5 blocks - LAST/WT ");
         for(Image::Domain::ConstIterator it = tiledImage.domain().begin(), itend = tiledImage.domain().end();
             it != itend; ++it)
           if (tiledImage( *it ) == 1)
                cpt++;         
         trace.info() << "Cpt: " << cpt << endl;
      trace.endBlock();
      
      /* trace.beginBlock("Counting ones in the tiled image - 5x5x5 blocks - FIFO/WT ");
        for(k = 0; k < NZ_3D_TILED; k++)
          for(j = 0; j < NY_3D_TILED; j++)
            for(i = 0; i < NX_3D_TILED; i++)
              if (tiledImage(Z3i::Point(i,j,k)) == 1)
                cpt++;         
        trace.info() << "Cpt: " << cpt << endl;
      trace.endBlock();
      */
      
    
    trace.endBlock();
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
    if (argc==1)
      writeHDF5_3D_TILED();
    
    exampleTiledImage3D();
    
    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
