#------------------------------------------------------------------------------
# DGtal Configuration file for LookUpTables
#------------------------------------------------------------------------------

# ------ AT CONFIGURE TIME ------ #
#--- Configuration of the src/topology/tables/NeighborhoodTables.h.in
set(TABLE_DIR ${PROJECT_BINARY_DIR}/src/DGtal/topology/tables)
configure_file(
  ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h.in
  ${PROJECT_BINARY_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h)

#--- Unpack files at cmake configuration time.
file(GLOB DGTAL_TABLES_COMPRESSED
     ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/*.tar.gz)
set(unzip_folder ${PROJECT_BINARY_DIR}/src/DGtal/topology/tables/)
message(STATUS "Decompressing look up tables to: ${unzip_folder}")
foreach(zip_table ${DGTAL_TABLES_COMPRESSED})
  execute_process(
    COMMAND           ${CMAKE_COMMAND} -E tar xzf ${zip_table}
    WORKING_DIRECTORY ${unzip_folder}
  )
endforeach()

# ------ AT INSTALLATION TIME ------ #
#--- Configuration of the src/topology/tables/NeighborhoodTables.h.in
install(CODE "
set(TABLE_DIR ${INSTALL_INCLUDE_DIR}/DGtal/topology/tables)
configure_file(
  ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h.in
  ${INSTALL_INCLUDE_DIR}/DGtal/topology/tables/NeighborhoodTables.h) " )

#--- specific install for uncompressed tables.
set(unzip_folder_install ${INSTALL_INCLUDE_DIR}/DGtal/topology/tables)
install(DIRECTORY "${unzip_folder}"
        DESTINATION "${unzip_folder_install}"
        FILES_MATCHING PATTERN "*.txt")

