cmake_minimum_required(VERSION 3.10  )

project(PQP)

SET(PQP_HPP_FILES
    Build.h
    BV.h
    BVTQ.h
    GetTime.h
    MatVec.h
    OBB_Disjoint.h
    PQP_Compile.h
    PQP_Internal.h
    PQP.h
    RectDist.h
    Tri.h
    TriDist.h    
)


add_library(pqp
  PQP.cpp
  Build.cpp
  BV.cpp
  TriDist.cpp
  )

target_include_directories(pqp INTERFACE $<BUILD_INTERFACE:${PQP_INCLUDE_DIR}> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/ext/rwpqp/>)

SET_TARGET_PROPERTIES(pqp PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
set_target_properties(pqp PROPERTIES VERSION 1 SOVERSION 1.0)
  
install(TARGETS pqp
  EXPORT ${PROJECT_PREFIX}Targets       
  RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT pqp
  LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT pqp
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT pqp
  )
install(FILES ${PQP_HPP_FILES} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/rwpqp/" COMPONENT pqp_dev)
