set(DGTAL_DOC_EX_SRC
  demo-kernel-1
  kernelDomain
  range
  labelledMapBestParameters
  )


if (BUILD_TESTING)
     DGtal_add_example(exampleCatch)
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

