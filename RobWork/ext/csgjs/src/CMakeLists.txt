
# Order object files by frequency of execution.  Small files at end.
set(
    csgjs_src
        csgjs/csgjs.cpp
)

file(GLOB csgjs_hdr csgjs/*.cpp)

rw_add_library(sdurw_csgjs STATIC ${csgjs_src})
set_property(TARGET sdurw_csgjs PROPERTY OUTPUT_NAME "sdurw_csgjs")
target_include_directories(sdurw_csgjs INTERFACE $<BUILD_INTERFACE:${CSGJS_INCLUDE_DIRS}> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/ext/csgjs/src>)
rw_add_includes(sdurw_csgjs /ext/csgjs/src ${csgjs_hdr})
