SET(DGTALIV_TESTS_SRC
   testIVViewer
   )

if ( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )

  FOREACH(FILE ${DGTALIV_TESTS_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} ${LIBDGTALIV_NAME} ${LIBDGTAL_NAME}
                            ${Boost_LIBRARIES}
   	                    ${SOQT_LIBRARY} ${COIN3D_LIBRARY} ${QT_LIBRARIES})
    #add_test(${FILE} ${FILE})
    #not a test since IHM
  ENDFOREACH(FILE)

endif ( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )
