s/DGtal::SpaceND<1u, int>/Z1/g
s/DGtal::SpaceND<2u, int>/Z2/g
s/DGtal::SpaceND<3u, int>/Z3/g
s/DGtal::KhalimskySpaceND<1u, int>/K1/g
s/DGtal::KhalimskySpaceND<2u, int>/K2/g
s/DGtal::KhalimskySpaceND<3u, int>/K3/g
s/PointsRange/PtRange/g
s/ConstIterator/CIt/g
s/Iterator/It/g
s/DGtal::HyperRectDomain/BOX/g
s/DGtal::MetricAdjacency<Z3, 2ul, 3ul>/Adj18/g
s/DGtal::MetricAdjacency<Z3, 1ul, 3ul>/Adj6/g
s/DGtal::MetricAdjacency<Z2, 2ul, 2ul>/Adj8/g
s/DGtal::MetricAdjacency<Z2, 1ul, 2ul>/Adj4/g
s/DGtal::DigitalTopology/DT/g
s/DGtal::DomainAdjacency/DAdj/g
s/DGtal::DigitalSetBySTLSet/SSet/g
s/DGtal::DigitalSetBySTLVector/VSet/g
s/DGtal:://g
s/::[a-zA-Z0-9_]*(/ & /g
s/)/ )/g
s/ >/>/g
s/>, />,/g
