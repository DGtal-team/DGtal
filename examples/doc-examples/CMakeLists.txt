set(DGTAL_DOC_EX_SRC
  demo-kernel-1
  kernelDomain
  range
  labelledMapBestParameters
  exampleCatch
  )

if( WITH_VISU3D_QGLVIEWER  )
  set(DGTAL_DOC_EX_SRC_QGL
     )

foreach(FILE ${DGTAL_DOC_EX_SRC_QGL})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE}  DGtal)
endforeach()

endif()


foreach(FILE ${DGTAL_DOC_EX_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal )
endforeach()

