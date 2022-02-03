set(SUBSYS_NAME sdurw_task)
set(SUBSYS_DESC "Generic Task description")
set(SUBSYS_DEPS sdurw_core sdurw)

set(DEFAULT TRUE)
set(REASON)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    # The camera file and include path to use.
    set(
        SRC_FILES
        ./Action.cpp
        ./Entity.cpp
        ./Motion.cpp
        ./Target.cpp
        ./Task.cpp
        ./TaskUtils.cpp
        ./TypeRepository.cpp
        ./GraspTask.cpp
        ./GraspTarget.cpp
        ./GraspResult.cpp
        ./GraspSubTask.cpp
        ./loader/DOMTaskFormat.cpp
        ./loader/DOMTaskLoader.cpp
        ./loader/DOMTaskSaver.cpp
        ./loader/TaskLoader.cpp
        ./loader/TaskSaver.cpp
    )

    set(
        SRC_FILES_HPP
        ./Action.hpp
        ./Entity.hpp
        ./Motion.hpp
        ./Target.hpp
        ./Task.hpp
        ./TaskUtils.hpp
        ./TypeRepository.hpp
        ./GraspTask.hpp
        ./GraspTarget.hpp
        ./GraspResult.hpp
        ./GraspSubTask.hpp
        ./loader/DOMTaskFormat.hpp
        ./loader/DOMTaskLoader.hpp
        ./loader/DOMTaskSaver.hpp
        ./loader/TaskLoader.hpp
        ./loader/TaskSaver.hpp
    )

    if(RW_HAVE_XERCES)
        set(SRC_FILES ${SRC_FILES} ./loader/XMLTaskFormat.cpp ./loader/XMLTaskLoader.cpp
                      ./loader/XMLTaskSaver.cpp)
        set(LOADER_SRC_FILES_HPP ./loader/XMLTaskFormat.hpp ./loader/XMLTaskLoader.hpp
                                 ./loader/XMLTaskSaver.hpp)
    endif()

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP} ${LOADER_SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/task" ${SRC_FILES_HPP})
    rw_add_include_dirs(${SUBSYS_NAME} "rwlibs/task" loader)
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
