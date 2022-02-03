
#INCLUDE(ExternalProject)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Subdirectories to process.
IF(RW_ENABLE_INTERNAL_PQP_TARGET)
    add_subdirectory(rwpqp)
ENDIF()

IF(RW_ENABLE_INTERNAL_YAOBI_TARGET)

    add_subdirectory(rwyaobi)
ENDIF()

if(RW_USE_CSGJS)
    add_subdirectory(csgjs)
endif()
