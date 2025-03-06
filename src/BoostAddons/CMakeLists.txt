set(BOOSTADDONS_SRC
		zlib.cpp)

add_library(DGTAL_BoostAddons STATIC ${BOOSTADDONS_SRC})
target_link_libraries(DGTAL_BoostAddons PRIVATE Boost::headers)
set_property(TARGET DGTAL_BoostAddons PROPERTY POSITION_INDEPENDENT_CODE ON)