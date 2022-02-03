
get_filename_component(MOD_ROOT ${CMAKE_CURRENT_LIST_FILE} PATH)
include("${MOD_ROOT}/Find_QT5_DIR.cmake")
find_qt_package(Qt5Core)