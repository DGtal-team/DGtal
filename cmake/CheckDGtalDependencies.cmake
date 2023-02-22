# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------

message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal required dependencies: ")


if (WINDOWS)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
  list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                TLS_VERIFY ON)
  endif()

  include(${CMAKE_BINARY_DIR}/conan.cmake)

  conan_cmake_configure(REQUIRES zlib/1.2.13
                      GENERATORS cmake_find_package)

  conan_cmake_configure(REQUIRES boost/1.81.0
                      OPTIONS boost:header_only=True
                      GENERATORS cmake_find_package)

  conan_cmake_autodetect(settings)
  conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})
ENDIF()

# -----------------------------------------------------------------------------
# Looking for boost
# -----------------------------------------------------------------------------
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_FOUND FALSE)
find_package(Boost 1.81.0 REQUIRED)
target_compile_definitions(DGtal PUBLIC ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
# SYSTEM to avoid warnings from boost.
target_include_directories(DGtal SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------
find_package(ZLIB REQUIRED)
target_link_libraries(DGtal PUBLIC ZLIB::ZLIB)
set(DGtalLibDependencies ${DGtalLibDependencies} ${ZLIB_LIBRARIES})

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT APPLE)
  target_link_libraries(DGtal PUBLIC rt)
  set(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()

# -----------------------------------------------------------------------------
# Eigen (already fetched)
# -----------------------------------------------------------------------------
set(WITH_EIGEN ON)
set(EIGEN_FOUND_DGTAL 1)
target_compile_definitions(DGtal PUBLIC "-DWITH_EIGEN=true")
set(DGtalLibDependencies ${DGtalLibDependencies} Eigen3::Eigen)
