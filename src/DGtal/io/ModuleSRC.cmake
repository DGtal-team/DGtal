## Sources associated to the module io
##


##########################################
#### boards
##########################################

#set(DGTAL_SRC ${DGTAL_SRC})


#set(DGTALIO_SRC ${DGTALIO_SRC})


if( WITH_CAIRO )
  set(DGTALIO_SRC ${DGTALIO_SRC} DGtal/io/boards/Board3DTo2D.cpp)
endif()
