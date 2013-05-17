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
 * @file ImageFactoryFromHDF5.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/16
 *
 * Header file for module ImageFactoryFromHDF5.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageFactoryFromHDF5_RECURSES)
#error Recursive header files inclusion detected in ImageFactoryFromHDF5.h
#else // defined(ImageFactoryFromHDF5_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageFactoryFromHDF5_RECURSES

#if !defined ImageFactoryFromHDF5_h
/** Prevents repeated inclusion of headers. */
#define ImageFactoryFromHDF5_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/Alias.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // Template class ImageFactoryFromHDF5
  /**
   * Description of template class 'ImageFactoryFromHDF5' <p>
   * \brief Aim: implements a factory from an HDF5 file.
   * 
   * @tparam TImageContainer an image container type (model of CImage).
   * 
   * The factory images production (images are copied, so it's a creation process) is done with the function 'requestImage'
   * so the deletion must be done with the function 'detachImage'.
   * 
   * The update of the original image is done with the function 'flushImage'.
   */
  template <typename TImageContainer>
  class ImageFactoryFromHDF5
  {

    // ----------------------- Types ------------------------------

  public:
    typedef ImageFactoryFromHDF5<TImageContainer> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Value Value;
    
    ///New types
    typedef ImageContainerBySTLVector<Domain, Value> OutputImage;

    // ----------------------- Standard services ------------------------------

  public:

    /**
     * Constructor.
     * @param anImage alias on the underlying image container.
     * @param aFilename HDF5 filename.
     * @param aDataset datasetname.
     */
    ImageFactoryFromHDF5(Alias<ImageContainer> anImage, const std::string & aFilename, const std::string & aDataset):
      myImagePtr(anImage), myFilename(aFilename), myDataset(aDataset)
    {
        /*typename ImageContainer::Point firstPoint;
        typename ImageContainer::Point lastPoint;

        firstPoint = ImageContainer::Point::zero;
        lastPoint[0] = dims_out[1]-1;
        lastPoint[1] = dims_out[0]-1;

        typename ImageContainer::Domain domain(firstPoint,lastPoint);
        myImagePtr->resize(domain.size()); // TODO problème car size OK mais toujours [[PointVector] {0, 0}]x[[PointVector] {0, 0}]*/
    }

    /**
     * Destructor.
     */
    ~ImageFactoryFromHDF5()
    {
    }

    // ----------------------- Interface --------------------------------------
  public:

    /////////////////// Domains //////////////////


    /////////////////// Accessors //////////////////

    
    /////////////////// API //////////////////

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return (myImagePtr->isValid());
    }

    /**
     * Returns a pointer of an OutputImage created with the Domain aDomain.
     * 
     * @param aDomain the domain.
     * 
     * @return an ImagePtr.
     */
    OutputImage * requestImage(const Domain &aDomain)
    {
      // HDF5 handles
      hid_t file, dataset;
      hid_t datatype, dataspace;
    
      H5T_class_t t_class;                  // data type class
      H5T_order_t order;                    // data order
      size_t      size;                     // size of the data element stored in file
      hsize_t     dims_out[2];              // dataset dimensions
      int         status_n, rank;
        
      // Open the file and the dataset.
      file = H5Fopen(myFilename.c_str(), /*H5F_ACC_RDONLY*/H5F_ACC_RDWR, H5P_DEFAULT);
      dataset = H5Dopen2(file, myDataset.c_str(), H5P_DEFAULT);

      // Get datatype and dataspace handles and then query dataset class, order, size, rank and dimensions.
      datatype = H5Dget_type(dataset); // datatype handle
      t_class = H5Tget_class(datatype);
      /*if (t_class == H5T_INTEGER)
        trace.info() << "Data set has INTEGER type" << std::endl;*/
      
      order = H5Tget_order(datatype);
      /*if (order == H5T_ORDER_LE)
        trace.info() << "Little endian order" << std::endl;*/

      size  = H5Tget_size(datatype);
      //trace.info() << "Data size is " << (int)size << std::endl;

      dataspace = H5Dget_space(dataset); // dataspace handle
      rank = H5Sget_simple_extent_ndims(dataspace);
      status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
      //trace.info() << "Rank: " << rank << ", dimensions: " << (unsigned long)(dims_out[0]) << " x " << (unsigned long)(dims_out[1]) << std::endl;
      
      // --
      
      hsize_t offset[2];        // hyperslab offset in the file
      hsize_t count[2];         // size of the hyperslab in the file
      
      herr_t status;
      hsize_t dimsm[2];         // memory space dimensions
      hid_t memspace;
      
      hsize_t offset_out[2];    // hyperslab offset in memory
      hsize_t count_out[2];     // size of the hyperslab in memory
      
      int i, j;
      
      int _NX_SUB = (aDomain.upperBound()[1]-aDomain.lowerBound()[1])+1;
      int _NY_SUB = (aDomain.upperBound()[0]-aDomain.lowerBound()[0])+1;
      
      int data_out[_NX_SUB][_NY_SUB];   // output buffer        // TODO -> int <-> H5T_INTEGER et new
      
      // Define hyperslab in the dataset.
      offset[0] = aDomain.lowerBound()[1];//-myImagePtr->domain().lowerBound()[1];
      offset[1] = aDomain.lowerBound()[0];//-myImagePtr->domain().lowerBound()[0];
      count[0] = _NX_SUB;
      count[1] = _NY_SUB;
      status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);

      // Define the memory dataspace.
      dimsm[0] = _NX_SUB;
      dimsm[1] = _NY_SUB;
      memspace = H5Screate_simple(2,dimsm,NULL);

      // Define memory hyperslab.
      offset_out[0] = 0;
      offset_out[1] = 0;
      count_out[0] = _NX_SUB;
      count_out[1] = _NY_SUB;
      status = H5Sselect_hyperslab(memspace, H5S_SELECT_SET, offset_out, NULL, count_out, NULL);

      // Read data from hyperslab in the file into the hyperslab in memory.
      status = H5Dread(dataset, H5T_NATIVE_INT, memspace, dataspace, H5P_DEFAULT, data_out);
    
      OutputImage* outputImage = new OutputImage(aDomain);

      for (j = 0; j < _NX_SUB; j++)
      {
        for (i = 0; i < _NY_SUB; i++)
        {
          typename OutputImage::Point pt;
          pt[0]=offset[1]+i; pt[1]=offset[0]+j;
          
          outputImage->setValue(pt, data_out[j][i]);
          //trace.info() << data_out[j][i];
        }
        //trace.info() << std::endl;
      }
      
      H5Sclose(memspace);
      
      // Reset the selection for the file dataspace.
      //status = H5Sselect_none(dataspace); // TODO -> utile ?
      
      // --

      // Close/release resources.
      H5Tclose(datatype);
      H5Dclose(dataset);
      H5Sclose(dataspace);
      H5Fclose(file);
        
      return outputImage;
    }
    
    /**
     * Flush (i.e. write/synchronize) an OutputImage.
     * 
     * @param outputImage the OutputImage.
     */
    void flushImage(OutputImage* outputImage)
    {
      // HDF5 handles
      hid_t file, dataset;
      hid_t datatype, dataspace;
    
      H5T_class_t t_class;                  // data type class
      H5T_order_t order;                    // data order
      size_t      size;                     // size of the data element stored in file
      hsize_t     dims_out[2];              // dataset dimensions
      int         status_n, rank;
        
      // Open the file and the dataset.
      file = H5Fopen(myFilename.c_str(), /*H5F_ACC_RDONLY*/H5F_ACC_RDWR, H5P_DEFAULT);
      dataset = H5Dopen2(file, myDataset.c_str(), H5P_DEFAULT);

      // Get datatype and dataspace handles and then query dataset class, order, size, rank and dimensions.
      datatype = H5Dget_type(dataset); // datatype handle
      t_class = H5Tget_class(datatype);
      /*if (t_class == H5T_INTEGER)
        trace.info() << "Data set has INTEGER type" << std::endl;*/
      
      order = H5Tget_order(datatype);
      /*if (order == H5T_ORDER_LE)
        trace.info() << "Little endian order" << std::endl;*/

      size  = H5Tget_size(datatype);
      //trace.info() << "Data size is " << (int)size << std::endl;

      dataspace = H5Dget_space(dataset); // dataspace handle
      rank = H5Sget_simple_extent_ndims(dataspace);
      status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
      //trace.info() << "Rank: " << rank << ", dimensions: " << (unsigned long)(dims_out[0]) << " x " << (unsigned long)(dims_out[1]) << std::endl;
      
      // --
      
      hsize_t offset[2];        // hyperslab offset in the file
      hsize_t count[2];         // size of the hyperslab in the file
      
      herr_t status;
      hsize_t dimsm[2];         // memory space dimensions
      hid_t memspace;
      
      hsize_t offset_in[2];    // hyperslab offset in memory
      hsize_t count_in[2];     // size of the hyperslab in memory
      
      int i, j;
      
      int _NX_SUB = (outputImage->domain().upperBound()[1]-outputImage->domain().lowerBound()[1])+1;
      int _NY_SUB = (outputImage->domain().upperBound()[0]-outputImage->domain().lowerBound()[0])+1;
      
      int data_in[_NX_SUB][_NY_SUB];    // input buffer         // TODO -> int <-> H5T_INTEGER et new
      
      // Define hyperslab in the dataset.
      offset[0] = outputImage->domain().lowerBound()[1];//-myImagePtr->domain().lowerBound()[1];
      offset[1] = outputImage->domain().lowerBound()[0];//-myImagePtr->domain().lowerBound()[0];
      count[0] = _NX_SUB;
      count[1] = _NY_SUB;
      status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
      
      // Define the memory dataspace.
      dimsm[0] = _NX_SUB;
      dimsm[1] = _NY_SUB;
      memspace = H5Screate_simple(2,dimsm,NULL);

      // Define memory hyperslab.
      offset_in[0] = 0;
      offset_in[1] = 0;
      count_in[0] = _NX_SUB;
      count_in[1] = _NY_SUB;
      status = H5Sselect_hyperslab(memspace, H5S_SELECT_SET, offset_in, NULL, count_in, NULL);
      
      /*typename OutputImage::Domain::Iterator it = outputImage->domain().begin();
      typename OutputImage::Domain::Iterator it_end = outputImage->domain().end();
      for (; it != it_end; ++it)
      {
        myImagePtr->setValue(*it, (*outputImage)(*it));
      }*/
      for (j = 0; j < _NX_SUB; j++)
      {
        for (i = 0; i < _NY_SUB; i++)
        {
          typename OutputImage::Point pt;
          pt[0]=offset[1]+i; pt[1]=offset[0]+j;
          
          data_in[j][i] = outputImage->operator()(pt);
          //trace.info() << data_in[j][i];
        }
        //trace.info() << std::endl;
      }
      
      // Write data from hyperslab in memory into the hyperslab in the file.
      status = H5Dwrite(dataset, H5T_NATIVE_INT, memspace, dataspace, H5P_DEFAULT, data_in);
      
      H5Sclose(memspace);
      
      // Reset the selection for the file dataspace.
      //status = H5Sselect_none(dataspace); // TODO -> utile ?
      
      // --

      // Close/release resources.
      H5Tclose(datatype);
      H5Dclose(dataset);
      H5Sclose(dataspace);
      H5Fclose(file);
    }
    
    /**
     * Free (i.e. delete) an OutputImage.
     * 
     * @param outputImage the OutputImage.
     */
    void detachImage(OutputImage* outputImage)
    {
      delete outputImage;
    }

    // ------------------------- Protected Datas ------------------------------
  private:
    /**
     * Default constructor.
     */
    ImageFactoryFromHDF5() {}
    
    // ------------------------- Private Datas --------------------------------
  protected:

    /// Alias on the image container
    ImageContainer * myImagePtr;
    
    /// HDF5 filename and datasetname
    const std::string myFilename;
    const std::string myDataset;

  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ImageFactoryFromHDF5


  /**
   * Overloads 'operator<<' for displaying objects of class 'ImageFactoryFromHDF5'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImageFactoryFromHDF5' to write.
   * @return the output stream after the writing.
   */
  template <typename TImageContainer>
  std::ostream&
  operator<< ( std::ostream & out, const ImageFactoryFromHDF5<TImageContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageFactoryFromHDF5.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageFactoryFromHDF5_h

#undef ImageFactoryFromHDF5_RECURSES
#endif // else defined(ImageFactoryFromHDF5_RECURSES)
