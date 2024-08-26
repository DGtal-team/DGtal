///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/images/RigidTransformation2D.h"
//////////////////
///
#include "QSH.h"
#include "RDSL.h"
#include "RBC.h"
#include "OTC.h"
#include "CBDR.h"
#include "Rotationtables.h"


using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z2i;

std::vector<std::string> supportedBijectiveRotation = {
    "OTC", "CBDR", "RDSL", "QSH" , "RBC"
};

// Generic function to handle rotation based on type
template <typename TImage,typename TBijectiveRotation>
void performRotation(const TImage& img, TBijectiveRotation& obj) {
    std::string structName = obj.tostring();

    if (std::find(supportedBijectiveRotation.begin(), supportedBijectiveRotation.end(), structName) != supportedBijectiveRotation.end()) {
    trace.beginBlock ( obj.tostring() );
        obj.rotateImage(img);
    trace.endBlock ();
    } else {
        throw std::runtime_error("Unsupported bijective rotation : " + structName);
    }
}

int main() {
    typedef ImageSelector<Domain, unsigned char >::Type Image;
    //! [def]
    typedef ForwardRigidTransformation2D < Space > ForwardTrans;
    typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
    typedef MyDomainTransformer::Bounds Bounds;

    trace.beginBlock ( "Example bijective rotations" );

    Image image = PGMReader<Image>::importPGM ( "church.pgm" );

    double angle = M_PI_4;
    int W=image.domain().myUpperBound[0];
    int H=image.domain().myUpperBound[1];
    std::cout << "w="<<W<<std::endl;
    std::cout << "h="<<H<<std::endl;
    DGtal::Z2i::Point c(W/2, H/2);

    /// QSH
    DGtal::QSH<DGtal::SpaceND< 2, DGtal::int32_t >> rotQSH(angle,c);
    performRotation<Image,QSH<DGtal::SpaceND< 2, DGtal::int32_t >>> (image,rotQSH);

    /// RDSL
    auto linf = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::DSL_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
    DGtal::RotationDSL<DGtal::SpaceND<2, DGtal::int32_t> > rotDSL(angle, c, linf);
    performRotation<Image,RotationDSL<DGtal::SpaceND< 2, DGtal::int32_t >>> (image,rotDSL);

    /// RBC
    DGtal::RBC_vec<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_rbcvec(2*max(W,H));
    rot_rbcvec.setAngle() = angle;
    rot_rbcvec.center() = c;
    DGtal::RBC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_RBC(rot_rbcvec,angle,c);
    performRotation<Image,DGtal::RBC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint>> (image,rot_RBC);


    /// CBDR
    auto linfCBDR = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CBDR_vec<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
    const int n = 3;
    const int kmax=15;
    DGtal::CBDR<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_CBDR(angle,c,n,kmax,linfCBDR,true);
    performRotation<Image,DGtal::CBDR<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint>> (image,rot_CBDR);

    /// OTC
    int rwidth = 2;
    std::vector< std::vector< int > > tableOTC = DGtal::functions::loadOTCTable<DGtal::SpaceND< 2, DGtal::int32_t >>("../tables/",rwidth);
    DGtal::OTC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_OTC( tableOTC, rwidth, c, W, H );
    performRotation<Image,DGtal::OTC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint>> (image,rot_OTC);

    trace.endBlock();




    return 0;
}
