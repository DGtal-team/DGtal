set(BOOSTADDONS_SRC
		zlib.cpp)

add_library(DGTAL_BoostAddons STATIC ${BOOSTADDONS_SRC})
target_link_libraries(DGTAL_BoostAddons PRIVATE Boost::boost)
target_include_directories(DGTAL_BoostAddons PUBLIC $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}>)
set_property(TARGET DGTAL_BoostAddons PROPERTY POSITION_INDEPENDENT_CODE ON)
