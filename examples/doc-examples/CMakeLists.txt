set(DGTAL_DOC_EX_SRC
  demo-kernel-1
  kernelDomain
  range
  labelledMapBestParameters
  )

include_directories(${PROJECT_SOURCE_DIR}/tests/)

if (DGTAL_BUILD_TESTS)
  add_executable(exampleCatch exampleCatch.cpp)
  target_link_libraries (exampleCatch DGtal Catch2::Catch2 DGtalCatch)
endif()


if( WITH_VISU3D_QGLVIEWER  )
  set(DGTAL_DOC_EX_SRC_QGL
     )

foreach(FILE ${DGTAL_DOC_EX_SRC_QGL})
   DGtal_add_example(${FILE})
endforeach()

endif()


foreach(FILE ${DGTAL_DOC_EX_SRC})
   DGtal_add_example(${FILE})
endforeach()
