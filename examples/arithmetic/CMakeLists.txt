SET(DGTAL_EXAMPLES_SRC
        approximation
        convergents
        fraction
        pattern
)

FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)

ENDFOREACH(FILE)

if ( GMP_FOUND )
   SET( DGTAL_EXAMPLES_GMP_SRC
        convergents-biginteger     
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_GMP_SRC}) 
   add_executable(${FILE} ${FILE})
   target_link_libraries ( ${FILE} DGtal
                           ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( GMP_FOUND )

