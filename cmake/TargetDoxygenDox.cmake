#make dox target

find_package(Doxygen)


if (DOXYGEN_FOUND)

  # click+jump in Emacs and Visual Studio (for doxy.config) (jw)
  if    (CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")
    set(DOXY_WARN_FORMAT "\"$file($line) : $text \"")
  else()
    set(DOXY_WARN_FORMAT "\"$file:$line: $text \"")
  endif()

  # we need latex for doxygen because of the formulas
  find_package(LATEX)
  if    (NOT LATEX_COMPILER)
    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user interaction on doxy run.")
  endif()
  if    (NOT MAKEINDEX_COMPILER)
    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  endif()
  if    (NOT DVIPS_CONVERTER)
    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  endif()

  if   (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox.in")
    MESSAGE(STATUS "configured ${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox.in --> ${CMAKE_CURRENT_BINARY_DIR}/doxy.config.dox")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox.in
      ${CMAKE_CURRENT_BINARY_DIR}/doxy.config.dox
      @ONLY )
    # use (configured) doxy.config from (out of place) BUILD tree:
    set(DOXY_CONFIG_DOX "${CMAKE_CURRENT_BINARY_DIR}/doxy.config.dox")
  else()
    # use static hand-edited doxy.config from SOURCE tree:
    set(DOXY_CONFIG_DOX "${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox")
    if   (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox")
      MESSAGE(STATUS "WARNING: using existing ${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox instead of configuring from doxy.config.dox.in file.")
    else()
      if   (EXISTS "${CMAKE_MODULE_PATH}/doc/doxy.config.dox.in")
        # using template doxy.config.dox.in
        MESSAGE(STATUS "config.doxured ${CMAKE_CMAKE_MODULE_PATH}/doc/doxy.config.dox.in --> ${CMAKE_CURRENT_BINARY_DIR}/doc/doxy.config.dox")
        CONFIGURE_FILE(${CMAKE_MODULE_PATH}/doc/doxy.config.dox.in
          ${CMAKE_CURRENT_BINARY_DIR}/doxy.config.dox
          @ONLY )
        set(DOXY_CONFIG_DOX "${CMAKE_CURRENT_BINARY_DIR}/doxy.config.dox")
      else()
        # failed completely...
        MESSAGE(SEND_ERROR "Please create ${CMAKE_CURRENT_SOURCE_DIR}/doc/doxy.config.dox.in (or doxy.config.dox as fallback)")
      endif()

    endif()
  endif()


  add_custom_target(dox ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG_DOX})


  # create a windows help .chm file using hhc.exe
  # HTMLHelp DLL must be in path!
  # fallback: use hhw.exe interactively
  if    (WIN32)
    find_package(HTMLHelp)
    if   (HTML_HELP_COMPILER)
      set (TMP "${CMAKE_CURRENT_BINARY_DIR}\\Doc\\html\\index.hhp")
      string(REGEX REPLACE "[/]" "\\\\" HHP_FILE ${TMP} )
      # MESSAGE(SEND_ERROR "DBG  HHP_FILE=${HHP_FILE}")
      add_custom_target(winhelp-dox ${HTML_HELP_COMPILER} ${HHP_FILE})
      add_dependencies (winhelp-dox dox)

      if (NOT TARGET_DOC_SKIP_INSTALL)
      # install windows help?
      # determine useful name for output file
      # should be project and version unique to allow installing
      # multiple projects into one global directory
      if   (EXISTS "${PROJECT_BINARY_DIR}/Doc/html/index.chm")
        if   (PROJECT_NAME)
          set(OUT "${PROJECT_NAME}")
        else()
          set(OUT "Documentation") # default
        endif()
        if   (${PROJECT_NAME}_VERSION_MAJOR)
          set(OUT "${OUT}-${${PROJECT_NAME}_VERSION_MAJOR}")
          if   (${PROJECT_NAME}_VERSION_MINOR)
            set(OUT  "${OUT}.${${PROJECT_NAME}_VERSION_MINOR}")
            if   (${PROJECT_NAME}_VERSION_PATCH)
              set(OUT "${OUT}.${${PROJECT_NAME}_VERSION_PATCH}")
            endif()
          endif()
        endif()
        # keep suffix
        set(OUT  "${OUT}.chm")

        #MESSAGE("DBG ${PROJECT_BINARY_DIR}/Doc/html/index.chm \n${OUT}")
        # create target used by install and package commands
        INSTALL(FILES "${PROJECT_BINARY_DIR}/Doc/html/index.chm"
          DESTINATION "doc"
          RENAME "${OUT}"
        )
      endif()
      endif()

    endif()
    # MESSAGE(SEND_ERROR "HTML_HELP_COMPILER=${HTML_HELP_COMPILER}")
  endif()

endif()
