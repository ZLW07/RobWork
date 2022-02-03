set(SUBSYS_NAME sdurw_common)
set(SUBSYS_DESC
    "This is some commone functinality used by most of the other robwork packages"
)
set(SUBSYS_DEPS sdurw_core)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        Archive.cpp
        BINArchive.cpp
        Cache.cpp
        ConcatVectorIterator.cpp
        FileCache.cpp
        INIArchive.cpp
        InputArchive.cpp
        LogBufferedChar.cpp
        LogBufferedMsg.cpp
        LogFileWriter.cpp
        LogMultiWriter.cpp
        OutputArchive.cpp
        PairMap.cpp
        ProgramOptions.cpp
        Serializable.cpp
        ThreadPool.cpp
        ThreadSafeQueue.cpp
        ThreadSafeStack.cpp
        ThreadSafeVariable.cpp
        ThreadTask.cpp
        Timer.cpp
        TimerUtil.cpp
        VectorIterator.cpp
    )

    set(FILES_HPP
        AnyPtr.hpp
        Archive.hpp
        BINArchive.hpp
        BoostXMLParser.hpp
        Cache.hpp
        ConcatVectorIterator.hpp
        DOMElem.hpp
        DOMParser.hpp
        Event.hpp
        Exception.hpp
        Extension.hpp
        ExtensionPoint.hpp
        ExtensionRegistry.hpp
        FileCache.hpp
        INIArchive.hpp
        IOUtil.hpp
        InputArchive.hpp
        Log.hpp
        LogBufferedChar.hpp
        LogBufferedMsg.hpp
        LogFileWriter.hpp
        LogMultiWriter.hpp
        LogStreamWriter.hpp
        LogWriter.hpp
        macros.hpp
        Message.hpp
        os.hpp
        OutputArchive.hpp
        PairMap.hpp
        Plugin.hpp
        ProgramOptions.hpp
        Property.hpp
        PropertyBase.hpp
        PropertyMap.hpp
        PropertyType.hpp
        Ptr.hpp
        ScopedTimer.hpp
        Serializable.hpp
        StringUtil.hpp
        ThreadPool.hpp
        ThreadSafeQueue.hpp
        ThreadSafeStack.hpp
        ThreadSafeVariable.hpp
        ThreadTask.hpp
        Timer.hpp
        TimerUtil.hpp
        Traits.hpp
        types.hpp
        VectorIterator.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/common" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} sdurw_core)

    rw_include_eigen(${SUBSYS_NAME})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../common.hpp)
endif()
