#
# Copyright 2019 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#
if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party (external): creating target 'igl::core'")

option(LIBIGL_INSTALL "Enable installation of libigl targets" ON)

include(CPM)
CPMAddPackage(
    NAME libigl
    GITHUB_REPOSITORY libigl/libigl
    VERSION 2.5.0
    OPTIONS LIBIGL_WITH_CGAL
)

# set_target_properties(igl_core PROPERTIES FOLDER third_party/libigl)

export (TARGETS igl_core FILE IGLConfig.cmake)
