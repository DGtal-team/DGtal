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
 * @file
 * @ingroup Tests
 * @author Stephane Breuils, David Coeurjolly, Jacques-Olivier Lachaud
 * @date 2024/08
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_BijectiveRotation <p>
 * Aim: simple test of Bijective rotation methods (CBDR,RDSL,QSH,OTC,RBC)
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include <iostream>
#include <cmath>
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/writers/GenericWriter.h"
//! [include]
#include "DGtal/images/RigidTransformation2D.h"
//! [include]
//////////////////
///
#include "DGtal/images/bijectiverotations/QSH.h"
#include "DGtal/images/bijectiverotations/CDLR.h"
#include "DGtal/images/bijectiverotations/RBC.h"
#include "DGtal/images/bijectiverotations/OTC.h"
#include "DGtal/images/bijectiverotations/CBDR.h"
#include "DGtal/images/bijectiverotations/Rotationtables.h"


using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z2i;

std::vector<std::string> supportedBijectiveRotation = {
 "OTC", "CBDR", "CDLR", "QSH" , "RBC"
};


template <typename TBijectiveRotation>
bool testBijectiveRotations(TBijectiveRotation& bijectiveRot) {
 typedef ImageSelector<Domain, unsigned char >::Type GrayImage;
 typedef ImageSelector<Domain, DGtal::Color >::Type ColorImage;

 typedef ForwardRigidTransformation2D < Space > ForwardTrans;
 typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
 typedef MyDomainTransformer::Bounds Bounds;

 Point A(0,0);
 Point B(200,200);
 HyperRectDomain<Space> my_domain(A,B);

 GrayImage imgGray( my_domain );
 ColorImage imgColor( my_domain );

 std::string structName = bijectiveRot.tostring();

 if (std::find(supportedBijectiveRotation.begin(), supportedBijectiveRotation.end(), structName) != supportedBijectiveRotation.end()) {
  trace.beginBlock ( "Bijective Rotation : "+ structName);
  trace.info() << bijectiveRot.tostring() << std::endl;

  auto rotatedImgGray  = bijectiveRot.rotateImage(imgGray);
  auto rotatedImgColor = bijectiveRot.rotateImage(imgColor);
  trace.endBlock ();
 } else {
  return false;
 }

  return true;
}

/// check that RDSL with a Linf error results in the same domain as RDSL with a mix of 1*Linf and 0*Lcontinuity
bool testCDLRPolicy(const Point& c, const double angle) {
 typedef ImageSelector<Domain, unsigned char >::Type GrayImage;
 typedef ImageSelector<Domain, DGtal::Color >::Type ColorImage;

 typedef ForwardRigidTransformation2D < Space > ForwardTrans;
 typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
 typedef MyDomainTransformer::Bounds Bounds;


 auto linf = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CDLR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
 auto linfWithMix = std::make_shared<DGtal::MixedPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CDLR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>(0.0,1.0);
 DGtal::CDLR<DGtal::SpaceND<2, DGtal::int32_t> > rotCDLRLinf(angle, c, linf);
 DGtal::CDLR<DGtal::SpaceND<2, DGtal::int32_t> > rotCDLRLinf_withMix(angle, c, linfWithMix);

 Point A(0,0);
 Point B(200,200);
 HyperRectDomain<Space> my_domain(A,B);
 GrayImage imgGray( my_domain );

 bool isSameTransformedDomain = true;
 for (typename Domain::ConstIterator it = imgGray.domain().begin(); it != imgGray.domain().end(); ++it )
 {
  Point cdlrLinf = rotCDLRLinf(*it);
  Point cdlrLinf_withMix = rotCDLRLinf_withMix(*it);
  isSameTransformedDomain *= (cdlrLinf_withMix==cdlrLinf);
 }
 return isSameTransformedDomain;
}

/// check that CBDR with a Linf error results in the same domain as RDSL with a mix of 1*Linf and 0*Lcontinuity
bool testCBDRPolicy(const Point& c, const double angle) {
 typedef ImageSelector<Domain, unsigned char >::Type GrayImage;
 typedef ImageSelector<Domain, DGtal::Color >::Type ColorImage;

 typedef ForwardRigidTransformation2D < Space > ForwardTrans;
 typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
 typedef MyDomainTransformer::Bounds Bounds;
 int kmax = 10; int n = 2;


 auto linf = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CBDR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
 auto linfWithMix = std::make_shared<DGtal::MixedPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CBDR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>(1.0,0.0);
 DGtal::CBDR<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rotCBDRLinf(angle, c,n,kmax, linf);
 DGtal::CBDR<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rotCBDRLinf_withMix(angle, c,n,kmax, linfWithMix);

 Point A(0,0);
 Point B(200,200);
 HyperRectDomain<Space> my_domain(A,B);
 GrayImage imgGray( my_domain );

 bool isSameTransformedDomain = true;
 for (typename Domain::ConstIterator it = imgGray.domain().begin(); it != imgGray.domain().end(); ++it )
 {
  Point cbdrLinf = rotCBDRLinf(*it);
  Point cbdrLinf_withMix = rotCBDRLinf_withMix(*it);
  isSameTransformedDomain *= (cbdrLinf_withMix==cbdrLinf);
 }
 return isSameTransformedDomain;
}


int main() {
 typedef ImageSelector<Domain, unsigned char >::Type Image;
 //! [def]
 typedef ForwardRigidTransformation2D < Space > ForwardTrans;
 typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
 typedef MyDomainTransformer::Bounds Bounds;

 /// init
 double angle = M_PI_4;
 Point c = {100,100};
 const int W=200; const int H=200;
 trace.beginBlock ( "Testing bijective rotations" );

 /// QSH
 DGtal::QSH<DGtal::SpaceND< 2, DGtal::int32_t >> rot_QSH(angle,c);

 /// Linf DSL
 auto linf = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CDLR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
 DGtal::CDLR<DGtal::SpaceND<2, DGtal::int32_t> > rot_RDSL(angle, c, linf);

 /// Linf CBDR
 auto linfCBDR = std::make_shared<DGtal::LinfPolicy<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::HyperRectDomain< DGtal::SpaceND< 2, DGtal::int32_t >>,DGtal::CBDR_naiverotation<DGtal::SpaceND< 2, DGtal::int32_t >>>>();
 const int n = 3;const int kmax=15;
 DGtal::CBDR<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_CBDR(angle,c,n,kmax,linfCBDR);

 /// RBC
 DGtal::RBC_vec<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_rbcvec(2*max(W,H));
 rot_rbcvec.setAngle() = angle;
 rot_rbcvec.center() = c;
 DGtal::RBC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_RBC(rot_rbcvec,angle,c);

 /// OTC
 int rwidth = 2;
 std::vector< std::vector< int > > tableOTC = DGtal::functions::loadOTCTable<DGtal::SpaceND< 2, DGtal::int32_t >>("../tables/",rwidth);
 DGtal::OTC<DGtal::SpaceND< 2, DGtal::int32_t >,DGtal::Z2i::RealPoint> rot_OTC( tableOTC, rwidth, c, W, H );


 bool res = testBijectiveRotations<OTC<DGtal::SpaceND< 2, DGtal::int32_t >>>(rot_OTC) &&
            testBijectiveRotations<RBC<DGtal::SpaceND< 2, DGtal::int32_t >>>(rot_RBC) &&
            testBijectiveRotations<CBDR<DGtal::SpaceND< 2, DGtal::int32_t >>>(rot_CBDR) &&
             testBijectiveRotations<CDLR<DGtal::SpaceND< 2, DGtal::int32_t >>>(rot_RDSL) &&
             testBijectiveRotations<QSH<DGtal::SpaceND< 2, DGtal::int32_t >>>(rot_QSH);


 // equal domain
 res = res&& testCDLRPolicy({100,100}, M_PI_4);
 res = res && testCBDRPolicy({100,100}, M_PI_4);

 trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
 trace.endBlock();


 return 0;
}