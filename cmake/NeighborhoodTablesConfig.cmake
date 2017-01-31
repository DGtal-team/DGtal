#------------------------------------------------------------------------------
# DGtal Configuration file for LookUpTables
#------------------------------------------------------------------------------

# TABLE_DIR is the variable that NeighborhoodTables.h.in read.

# ------ Build Tree ------ #
#--- Configuration of the src/topology/tables/NeighborhoodTables.h.in
set(TABLE_DIR ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables)
configure_file(
  ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h.in
  ${PROJECT_BINARY_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h)

# ------ Install Tree ------ #
#--- Configuration of the src/topology/tables/NeighborhoodTables.h.in for the install tree. Save to tmp file.
# dev note: we escape \$ENV{DESTDIR} because we want DESTDIR to be read at
#  install time, not at configure time. The same with TABLE_DIR inside configure_file.
#  Read more: https://cmake.org/pipermail/cmake-developers/2013-January/017810.html
install(CODE "
set(TABLE_DIR \$ENV{DESTDIR}${INSTALL_INCLUDE_DIR}/DGtal/topology/tables)
configure_file(
  ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/NeighborhoodTables.h.in
  \${TABLE_DIR}/NeighborhoodTables.h @ONLY)")

#--- Install compressed tables and the header pointing to them ---#
set(table_folder_install ${INSTALL_INCLUDE_DIR}/DGtal/topology/tables)
#--- Select all the tables and install ---#
install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/"
        DESTINATION "${table_folder_install}"
        FILES_MATCHING PATTERN "*.zlib")
