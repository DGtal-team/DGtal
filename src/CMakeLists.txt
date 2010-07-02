# Make sure the compiler can find include files.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories (${PROJECT_SOURCE_DIR}/src)

##Recursive calls
## each subdir must contain a ModuleSRC.txt file with a set command on the variable ${DGTAL_SRC}
include(DGtal/utils/ModuleSRC.txt)
include(DGtal/base/ModuleSRC.txt)
include(DGtal/math/ModuleSRC.txt)
include(DGtal/kernel/ModuleSRC.txt)
include(DGtal/geometry/ModuleSRC.txt)
include(DGtal/topology/ModuleSRC.txt)

include(Board/ModuleSRC.txt)

# Create a library called "DGtal".
# The extension is already found. Any number of sources could be listed here.
add_library ( ${LIBDGTAL_NAME}  ${DGTAL_SRC} ${BOARD_SRC})

message(STATUS "DTGAL_SRCs: "${DGTAL_SRC})

install(TARGETS ${LIBDGTAL_NAME} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

