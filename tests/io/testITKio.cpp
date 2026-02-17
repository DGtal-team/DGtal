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
 * @file testITKio.cpp
 * @ingroup Tests
 * @author Pierre Gueth (\c pierre.gueth@gmail.com )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/10/28
 *
 * Functions for testing class ITKio.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "ConfigTest.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/CImage.h"
#include "DGtal/io/writers/ITKWriter.h"
#include "DGtal/io/readers/ITKReader.h"
using namespace DGtal;

#include <string>
using std::endl;
using std::string;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing ITK io
///////////////////////////////////////////////////////////////////////////////
template <typename Image>
bool
test_image(const string& filename)
{
    BOOST_CONCEPT_ASSERT(( concepts::CImage<Image> ));

    typedef typename Image::Domain::Point Point;
    Point point0;
    Point point1;
    for (typename Image::Domain::Dimension kk=0; kk<Image::Domain::dimension; kk++)
    {
        point0[kk] = 5;
        point1[kk] = 10;
    }

    typedef typename Image::Domain Domain;
    const Domain domain(point0, point1);
    Image image(domain);

    typedef typename std::vector<typename Image::Value> Values;
    Values values;
    values.reserve(domain.size());
    for (typename Domain::Size kk=0; kk<domain.size(); kk++)
        values.push_back(rand());

    std::copy(values.begin(), values.end(), image.range().outputIterator());

    trace.info() << image << endl;
    trace.info() << "writing " << filename << endl;
    if (!ITKWriter<Image>::exportITK(filename, image)) return false;

    trace.info() << "reading " << filename << endl;
    Image image_read = ITKReader<Image>::importITK(filename);
    trace.info() << image_read << endl;

    if (image_read.domain().lowerBound() !=  image.domain().lowerBound()) trace.warning() << "lowerBound mismatch!!" << endl;
    if (image_read.domain().upperBound() !=  image.domain().upperBound()) trace.warning() << "upperBound mismatch!!" << endl;

    typename Image::ConstRange::ConstIterator iter_read = image_read.constRange().begin();
    typename Image::ConstRange::ConstIterator iter_read_end = image_read.constRange().end();
    typename Values::const_iterator iter_value = values.begin();
    typename Values::const_iterator iter_value_end = values.end();
    while (iter_value!=iter_value_end && iter_read!=iter_read_end)
    {
        if ((*iter_read)!=(*iter_value)) {
            trace.error() << "values mismatch" << endl;
            return false;
        }
        iter_value++;
        iter_read++;
    }

    return true;
}

bool
testITKSpacingIO()
{
  typedef ImageContainerBySTLVector<Z3i::Domain, int> Image3D;
  typedef ImageContainerByITKImage<Z3i::Domain, int> Image3DITK;
  Image3DITK input = ITKReader<Image3DITK>::importITK("image_3d_int.mha");
  Image3DITK copy(input.domain());
  Image3DITK::ImageSpacing s (0.2, 0.3, 0.4);
  trace.info() << "setting image spacing to 0.2, 0.3, 0.4" << std::endl;
  copy.setImageSpacing(s);
  for (auto p: input.domain() ) {copy.setValue(p, input(p));}
  ITKWriter<Image3DITK>::exportITK("image_3d_intSpace0.2.mha", copy);
  Image3DITK check = ITKReader<Image3DITK>::importITK("image_3d_intSpace0.2.mha");
  s = check.getImageSpacing();
  trace.info() << "reading image spacing after write (should be 0.2, 0.3, 0.4)" << std::endl;
  trace.info() << "spacing: " << s[0] << " " << s[1] << " " << s[2]  << std::endl;
  Image3D img (input.domain());
  ITKWriter<Image3D>::exportITK("imageVect_3d_intSpace0.8.mha", img, Z3i::RealPoint(0.8, 0.9, 1.0));
  Image3DITK check3 = ITKReader<Image3DITK>::importITK("imageVect_3d_intSpace0.8.mha");
  auto s3 = check3.getImageSpacing();
  trace.info() << "reading image spacing after export with spacing mention (should be 0.8, 0.9, 1.0)" << std::endl;
  trace.info() << "spacing: " << s3[0] << " " << s3[1]  << " " << s3[2] <<  std::endl;

  typedef ImageContainerByITKImage<Z2i::Domain, int> Image2DITK;
  Image2DITK input2 = ITKReader<Image2DITK>::importITK("image_2d_int.mha");
  Image2DITK copy2(input2.domain());
  Image2DITK::ImageSpacing s2 (0.2, 0.3);
  trace.info() << "setting image spacing to 0.2, 0.3" << std::endl;
  copy2.setImageSpacing(s2);
  for (auto p: input2.domain() ) {copy2.setValue(p, input2(p));}
  ITKWriter<Image2DITK>::exportITK("image_2d_intSpace0.2.mha", copy2);
  Image2DITK check2 = ITKReader<Image2DITK>::importITK("image_2d_intSpace0.2.mha");
  s2 = check2.getImageSpacing();
  trace.info() << "reading image spacing after write (should be 0.2, 0.3)" << std::endl;
  trace.info() << "spacing: " << s2[0] << " " << s2[1]  << std::endl;


  return s[0] == 0.2 && s[1] == 0.3 && s[2] == 0.4 &&
         s2[0] == 0.2 && s2[1] == 0.3 &&
         s3[0] == 0.8 && s3[1] == 0.9 && s3[2] == 1.0;
}


bool testITKio()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  nb += 8;
  trace.beginBlock ( "Testing 2D ITK image value types ..." );
  nbok += test_image<ImageSelector<Z2i::Domain, int>::Type>("image_2d_int.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, bool>::Type>("image_2d_bool.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned int>::Type>("image_2d_unsigned_int.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned char>::Type>("image_2d_unsigned_char.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned long>::Type>("image_2d_unsigned_long.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, long>::Type>("image_2d_long.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, float>::Type>("image_2d_float.mha");
  nbok += test_image<ImageSelector<Z2i::Domain, double>::Type>("image_2d_double.mha");
  trace.endBlock();

  nb += 8;
  trace.beginBlock ( "Testing 3D ITK image value types ..." );
  nbok += test_image<ImageSelector<Z3i::Domain, int>::Type>("image_3d_int.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, bool>::Type>("image_3d_bool.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, unsigned int>::Type>("image_3d_unsigned_int.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, unsigned char>::Type>("image_3d_unsigned_char.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, unsigned long>::Type>("image_3d_unsigned_long.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, long>::Type>("image_3d_long.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, float>::Type>("image_3d_float.mha");
  nbok += test_image<ImageSelector<Z3i::Domain, double>::Type>("image_3d_double.mha");
  trace.endBlock();

  nb += 3;
  trace.beginBlock ( "Testing 2D ITK image formats ..." );
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned char>::Type>("image_unsigned_char.jpg"); nb--; // jpg is lossy
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned char>::Type>("image_unsigned_char.png");
  nbok += test_image<ImageSelector<Z2i::Domain, unsigned char>::Type>("image_unsigned_char.bmp");
  trace.endBlock();

  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  nb += 1;
  trace.beginBlock ( "Testing 3D ITK image with spacing ..." );
  nbok += testITKSpacingIO();
  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int /*argc*/, char** /*argv*/ )
{
  bool res = testITKio(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
