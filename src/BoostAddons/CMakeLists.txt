set(BOOSTADDONS_SRC
		zlib.cpp)

# @TODO Link to Boost 
add_library(DGTAL_BoostAddons STATIC ${BOOSTADDONS_SRC})
set_property(TARGET DGTAL_BoostAddons PROPERTY POSITION_INDEPENDENT_CODE ON)