PROJECT(DGtal-Exercises)

cmake_minimum_required (VERSION 2.4) 
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

# -----------------------------------------------------------------------------
# Check Dependencies
# -----------------------------------------------------------------------------

FIND_PACKAGE(Boost REQUIRED COMPONENTS serialization)
if ( Boost_FOUND )
   message( "-- Boost found. include=${Boost_INCLUDE_DIR} libs=${Boost_LIBRARIES}" )
   include_directories( ${Boost_INCLUDE_DIR} )
endif ( Boost_FOUND )


##########Check if libMagick++ exists
FIND_PACKAGE(Magick)

IF(MAGICK++_FOUND)
  INCLUDE_DIRECTORIES(${MAGICK++_INCLUDE_DIR})
   message( "-- GraphicsMagick++ found." )
ENDIF(MAGICK++_FOUND)



##########Check if DGatl exists
FIND_PACKAGE(DGtal)

IF(DGtal_FOUND)
  INCLUDE_DIRECTORIES(${DGtal_INCLUDE_DIR})
   message( "-- DGtal found." )
ENDIF(DGtal_FOUND)

ADD_SUBDIRECTORY(1_basic_setup)
ADD_SUBDIRECTORY(2_basic_images)
