configure_file("${RW_ROOT}/src/rwlibs/swig/ext_i/os.i.in"
               "${RW_ROOT}/src/rwlibs/swig/ext_i/os.i" IMMEDIATE @ONLY
)

add_subdirectory(java)
add_subdirectory(python)
add_subdirectory(lua)


rw_add_includes(swig "rwlibs/swig" ScriptTypes.hpp)
