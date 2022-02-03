set(SUBSYS_NAME sdurw_core)
set(SUBSYS_DESC
    "This is the core of the RobWork FrameWork, providing fonctinality for logging and plugins"
)
set(SUBSYS_DEPS)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(FILES_CPP
        Extension.cpp
        ExtensionPoint.cpp
        ExtensionRegistry.cpp
        Log.cpp
        LogStreamWriter.cpp
        Message.cpp
        RobWork.cpp
        Property.cpp
        PropertyBase.cpp
        PropertyMap.cpp
        PropertyValue.cpp
        PropertyValueBase.cpp
        AnyPtr.cpp
        PropertyType.cpp
        Event.cpp
        Exception.cpp
        LogWriter.cpp
        IOUtil.cpp
        Plugin.cpp
        StringUtil.cpp
        DOMElem.cpp
        DOMParser.cpp
        DOMCorePropertyMapLoader.cpp
        DOMCorePropertyMapSaver.cpp
        DOMCoreBasisTypes.cpp
        DOMPropertyMapFormat.cpp
        BoostXMLParser.cpp
    )

    set(FILES_HPP
        Extension.hpp
        ExtensionPoint.hpp
        ExtensionRegistry.hpp
        Log.hpp
        LogStreamWriter.hpp
        Message.hpp
        Ptr.hpp
        RobWork.hpp
        macros.hpp
        math_fwd.hpp
        os.hpp
        PairIterator.hpp
        Property.hpp
        PropertyBase.hpp
        PropertyMap.hpp
        PropertyValue.hpp
        PropertyValueBase.hpp
        AnyPtr.hpp
        PropertyType.hpp
        Event.hpp
        Exception.hpp
        LogWriter.hpp
        IOUtil.hpp
        Plugin.hpp
        StringUtil.hpp
        DOMElem.hpp
        DOMParser.hpp
        DOMCorePropertyMapLoader.hpp
        DOMCorePropertyMapSaver.hpp
        DOMCoreBasisTypes.hpp
        DOMPropertyMapFormat.hpp
        BoostXMLParser.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/core" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../core.hpp)
    
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} PRIVATE ${CMAKE_DL_LIBS})

    target_include_directories(${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR})


    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    

endif()
