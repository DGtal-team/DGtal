SET(DGTAL_DOC_EX_SRC
  demo-kernel-1
  kernelDomain
  range
  ctopo-1
  ctopo-2
  khalimskySpaceScanner
)

if( WITH_VISU3D_QGLVIEWER  )
SET(DGTAL_DOC_EX_SRC_QGL
   ctopo-1-3d
   ctopo-1s-3d
   ctopo-2-3d
)

FOREACH(FILE ${DGTAL_DOC_EX_SRC_QGL})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtalIO DGtal)
ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER )


FOREACH(FILE ${DGTAL_DOC_EX_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
ENDFOREACH(FILE)

