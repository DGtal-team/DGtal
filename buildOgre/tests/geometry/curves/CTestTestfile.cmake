# CMake generated Testfile for 
# Source directory: /home/anisbenyoub/Libraries/DGtal/tests/geometry/curves
# Build directory: /home/anisbenyoub/Libraries/DGtal/buildOgre/tests/geometry/curves
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(testArithDSS "testArithDSS")
ADD_TEST(testArithDSS3d "testArithDSS3d")
ADD_TEST(testFreemanChain "testFreemanChain")
ADD_TEST(testDecomposition "testDecomposition")
ADD_TEST(testSegmentation "testSegmentation")
ADD_TEST(testMaximalSegments "testMaximalSegments")
ADD_TEST(testFP "testFP")
ADD_TEST(testGridCurve "testGridCurve")
ADD_TEST(testCombinDSS "testCombinDSS")
ADD_TEST(testGeometricalDSS "testGeometricalDSS")
ADD_TEST(testGeometricalDCA "testGeometricalDCA")
ADD_TEST(testBinomialConvolver "testBinomialConvolver")
SUBDIRS(estimation)
