set(DGTAL_TESTS_SRC
   testAdjacency
   testKhalimskySpaceND
   testCubicalComplex
   testVoxelComplex
   testDigitalSurface
   testDigitalTopology
   testObject
   testObjectBorder
   testSimpleExpander
   testSCellsFunctor
   testUmbrellaComputer
   testSurfaceHelper
   testDigitalSetToCellularGridConverter
   testNeighborhoodConfigurations
   testParDirCollapse
   testHalfEdgeDataStructure
   testIndexedDigitalSurface
)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
