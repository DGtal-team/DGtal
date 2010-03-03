# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/include)


##Recursive calls
## each subdir must contain a ModuleSRC.txt file with a set command on the variable ${DGTAL_SRC}
include(base/ModuleSRC.txt)
include(utils/ModuleSRC.txt)


# Create a library called "DGtal".
# The extension is already found. Any number of sources could be listed here.
add_library ( ${LIBDGTAL_NAME}  ${DGTAL_SRC})
target_link_libraries (${LIBDGTAL_NAME} )

message(STATUS "DTGAL_SRCs: "${DGTAL_SRC})

install(TARGETS ${LIBDGTAL_NAME} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
