set(BOOSTADDONS_SRC
		zlib.cpp)

add_library(DGTAL_BoostAddons STATIC ${BOOSTADDONS_SRC})
target_link_libraries(DGTAL_BoostAddons PRIVATE Boost::boost       boost_headers
      boost_throw_exception
      boost_assert
      boost_config
      boost_iostreams
      boost_core
      boost_detail
      boost_function
      boost_static_assert
      boost_integer
      boost_iterator
      boost_mpl
      boost_preprocessor
      boost_random
      boost_range
      boost_regex
      boost_smart_ptr
      boost_utility
      boost_numeric_conversion
      boost_type_traits
      boost_bind
      boost_concept_check
      boost_function_types
      boost_fusion
      boost_optional
      boost_predef
      boost_array
      boost_dynamic_bitset
      boost_io
      boost_system
      boost_container_hash
      boost_conversion
      boost_tuple
      boost_typeof
      boost_move
      boost_functional
      boost_mp11
      boost_variant2
      boost_winapi
      boost_describe)
set_property(TARGET DGTAL_BoostAddons PROPERTY POSITION_INDEPENDENT_CODE ON)