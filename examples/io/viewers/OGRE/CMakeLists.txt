
SET(DGTAL_TESTS_SRC
   testDomainViewer
   testSetViewer
   testPoint1Viewer
   testPoint2Viewer
   testSelectionViewer
   testManipulateViewer
)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtalIO DGtal ${DGtalLibDependencies})
ENDFOREACH(FILE)

