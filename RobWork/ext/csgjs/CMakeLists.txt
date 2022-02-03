cmake_minimum_required(VERSION 3.10  )

if(POLICY CMP0003)
  cmake_policy(SET CMP0003 NEW)
endif()

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(csgjs)

add_subdirectory(src)

