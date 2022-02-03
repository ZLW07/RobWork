set(SUBSYS_NAME sdurw_mathematica)
set(SUBSYS_DESC "Mathematica interface for RobWork")
set(SUBSYS_DEPS sdurw)

set(build ON)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RW_HAVE_MATHEMATICA} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(
        SRC_FILES
        Mathematica.cpp
        # Packets
        EnterExpressionPacket.cpp
        EnterTextPacket.cpp
        EvaluatePacket.cpp
        InputNamePacket.cpp
        MessagePacket.cpp
        OutputNamePacket.cpp
        ReturnExpressionPacket.cpp
        ReturnPacket.cpp
        ReturnTextPacket.cpp
        TextPacket.cpp
        # Functions
        FactorInteger.cpp
        Image.cpp
        List.cpp
        ListPlot.cpp
        RawArray.cpp
        Rule.cpp
        ToExpression.cpp
    )

    set(
        SRC_FILES_HPP
        Mathematica.hpp
        # Packets
        EnterExpressionPacket.hpp
        EnterTextPacket.hpp
        EvaluatePacket.hpp
        InputNamePacket.hpp
        MessagePacket.hpp
        OutputNamePacket.hpp
        ReturnExpressionPacket.hpp
        ReturnPacket.hpp
        ReturnTextPacket.hpp
        TextPacket.hpp
        # Functions
        FactorInteger.hpp
        Image.cpp
        List.hpp
        ListPlot.hpp
        RawArray.hpp
        Rule.hpp
        ToExpression.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/mathematica" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} PRIVATE ${Mathematica_WSTP_LIBRARIES})
    target_include_directories(${SUBSYS_NAME}
    	PRIVATE
    	${Mathematica_WSTP_INCLUDE_DIR}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
