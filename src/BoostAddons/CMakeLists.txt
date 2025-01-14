set(BOOSTADDONS_SRC
		zlib.cpp)

# @TODO Link to Boost 
add_library(DGTAL_BoostAddons STATIC ${BOOSTADDONS_SRC})