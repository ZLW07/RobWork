if(NOT DEFINED RWP_ROOT)
    unset(SLIB_COMPILE_BUFFER CACHE) # this is used in macro RW_ADD_SWIG
endif()

add_subdirectory(lua)
add_subdirectory(java)
add_subdirectory(python)