## Sources associated to the module io
##


##########################################
#### boards
##########################################

set(DGTAL_SRC ${DGTAL_SRC}
  DGtal/io/Color.cpp)


set(DGTALIO_SRC ${DGTALIO_SRC}
  DGtal/io/boards/Board2D.cpp
  DGtal/io/boards/Board3D.cpp)


if( DGTAL_WITH_CAIRO )
  set(DGTALIO_SRC ${DGTALIO_SRC} DGtal/io/boards/Board3DTo2D.cpp)
endif()
