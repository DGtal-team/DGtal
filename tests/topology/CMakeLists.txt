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
   testSplitFunctions
)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

set(DGTAL_BENCH_SRC
   testObject-benchmark
   testImplicitDigitalSurface-benchmark
   testLightImplicitDigitalSurface-benchmark
)

#Benchmark target
foreach(FILE ${DGTAL_BENCH_SRC})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()
