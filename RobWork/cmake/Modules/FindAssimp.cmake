# * Try to find Assimp 3.0 Once done this will define Assimp_FOUND - System has Assimp
#   ASSIMP_INCLUDE_DIRS - The Assimp include directories ASSIMP_LIBRARY_DIRS - The library
#   directories needed to use Assimp ASSIMP_LIBRARIES    - The libraries needed to use Assimp

if(ASSIMP_INCLUDE_DIR)
    # in cache already
    set(ASSIMP_FIND_QUIETLY TRUE)
endif(ASSIMP_INCLUDE_DIR)

find_path(
    ASSIMP_INCLUDE_DIR
    NAMES assimp/Importer.hpp
    PATHS "$ENV{Assimp_ROOT}/include" "/usr/include"
)

if(MSVC12)
    set(ASSIMP_MSVC_VERSION "vc120")
elseif(MSVC14)
    set(ASSIMP_MSVC_VERSION "vc140")
endif(MSVC12)

find_library(
    ASSIMP_LIBRARY assimp
    PATHS "$ENV{Assimp_ROOT}/lib/"
          "$ENV{ASSIMP_ROOT}/lib/"
          "/usr/lib"
          "C:/Local/Assimp/lib"
          "C:/Program Files/Assimp/lib"
          "C:/Program Files (x86)/Assimp/lib"
          "C:/Local/assimp/lib"
          "C:/Program Files/assimp/lib"
          "C:/Program Files (x86)/assimp/lib"
)
find_library(
    ASSIMP_LIBRARY assimp-${ASSIMP_MSVC_VERSION}-mt
    PATHS "$ENV{Assimp_ROOT}/lib/"
          "$ENV{ASSIMP_ROOT}/lib/"
          "/usr/lib"
          "C:/Local/Assimp/lib"
          "C:/Program Files/Assimp/lib"
          "C:/Program Files (x86)/Assimp/lib"
          "C:/Local/assimp/lib"
          "C:/Program Files/assimp/lib"
          "C:/Program Files (x86)/assimp/lib"
)

set(ASSIMP_INCLUDE_DIRS "${ASSIMP_INCLUDE_DIR}")
set(ASSIMP_LIBRARIES "${ASSIMP_LIBRARY}")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ASSIMP_FOUND to TRUE if all listed variables are
# TRUE
find_package_handle_standard_args(Assimp DEFAULT_MSG ASSIMP_LIBRARY ASSIMP_INCLUDE_DIR)

mark_as_advanced(ASSIMP_INCLUDE_DIR ASSIMP_LIBRARY)
