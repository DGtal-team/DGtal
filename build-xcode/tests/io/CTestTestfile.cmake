# CMake generated Testfile for 
# Source directory: /Users/davidcoeurjolly/Sources/DGtal/tests/io
# Build directory: /Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/io
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(testSimpleBoard "testSimpleBoard")
ADD_TEST(testBoard2DCustomStyle "testBoard2DCustomStyle")
ADD_TEST(testLongvol "testLongvol")
ADD_TEST(testArcDrawing "testArcDrawing")
SUBDIRS(viewers)
SUBDIRS(colormaps)
SUBDIRS(readers)
SUBDIRS(writers)
