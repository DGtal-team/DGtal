# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------

message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal required dependencies: ")


# -----------------------------------------------------------------------------
# Mandatory and optional deps via conan
# -----------------------------------------------------------------------------
option(ENABLE_CONAN "Enable conan for deps discovery (used for windows CI for instance) features." OFF)


if (ENABLE_CONAN)
  message(STATUS  "Conan enabled for deps")

  if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                 STATUS DOWNLOAD_STATUS
                TLS_VERIFY OFF)
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    # Check if download was successful.
    if(${STATUS_CODE} EQUAL 0)
       message(STATUS "Download completed successfully!")
    else()
     # Exit CMake if the download failed, printing the error message.
       message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
    endif()
  endif()

  include("${CMAKE_BINARY_DIR}/conan.cmake")

  conan_cmake_configure(REQUIRES zlib/1.2.13
                                 boost/1.81.0
                                 gmp/6.2.1
                                 fftw/3.3.9
                                 cairo/1.17.6
                                 libpng/1.6.39 #Explicit fix deps (compat issues)
                                 expat/2.5.0
                                 openssl/1.1.1s
                                 libiconv/1.17
                      OPTIONS boost:header_only=True
                              gmp:enable_cxx=True
                      GENERATORS cmake_find_package)

  conan_cmake_autodetect(settings)
  conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})

  set(FFTW3_DIR ${CONAN_FFTW_ROOT})
  message(STATUS "Setting FFTW3_DIR to ${CONAN_FFTW_ROOT}")
else()
  message(STATUS "Conan disabled")
endif()

# -----------------------------------------------------------------------------
# Looking for boost
# -----------------------------------------------------------------------------
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_FOUND FALSE)
find_package(Boost 1.50.0 REQUIRED)
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
