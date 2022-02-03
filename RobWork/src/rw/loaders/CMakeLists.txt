set(SUBSYS_NAME sdurw_loaders)
set(SUBSYS_DESC "A collection of fileloading algorithms!")
set(SUBSYS_DEPS
    sdurw_core
    sdurw_common
    sdurw_math
    sdurw_kinematics
    sdurw_geometry
    sdurw_sensor
    sdurw_models
    sdurw_proximity
    sdurw_graphics
    sdurw_trajectory
)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        WorkCellLoader.cpp
        WorkCellFactory.cpp
        # model3d
        model3d/LoaderAC3D.cpp
        model3d/Loader3DS.cpp
        model3d/Model3DS.cpp
        model3d/LoaderOBJ.cpp
        model3d/LoaderTRI.cpp
        model3d/STLFile.cpp
        Model3DFactory.cpp
        Model3DLoader.cpp
        GeometryFactory.cpp
        # path
        path/PathLoader.cpp
        path/PathLoaderCSV.cpp
        # rwxml
        rwxml/DependencyGraph.cpp
        rwxml/MultipleFileIterator.cpp
        rwxml/XMLParserUtil.cpp
        rwxml/XMLRWPreParser.cpp
        rwxml/XMLRWParser.cpp
        rwxml/XMLRWLoader.cpp
        rwxml/XMLErrorHandler.cpp
        rwxml/XMLParser.cpp
        rwxml/XML.cpp
        # dom
        dom/DOMBasisTypes.cpp
        dom/DOMPropertyMapFormat.cpp
        dom/DOMPropertyMapLoader.cpp
        dom/DOMPropertyMapSaver.cpp
        dom/DOMPathLoader.cpp
        dom/DOMPathSaver.cpp
        dom/DOMTrajectoryLoader.cpp
        dom/DOMTrajectorySaver.cpp
        dom/DOMProximitySetupLoader.cpp
        dom/DOMProximitySetupSaver.cpp
        dom/DOMWorkCellSaver.cpp
        # collision
        colsetup/CollisionSetupLoader.cpp
        # images
        image/PGMLoader.cpp
        image/PPMLoader.cpp
        image/RGBLoader.cpp
        ImageLoader.cpp
    )

    set(FILES_HPP WorkCellLoader.hpp WorkCellFactory.hpp Model3DFactory.hpp Model3DLoader.hpp
                  GeometryFactory.hpp
    )
    set(FILES_HPP_model3d model3d/LoaderAC3D.hpp model3d/Loader3DS.hpp model3d/Model3DS.hpp
                          model3d/LoaderOBJ.hpp model3d/LoaderTRI.hpp model3d/STLFile.hpp
    )
    set(FILES_HPP_path path/PathLoader.hpp path/PathLoaderCSV.hpp)
    set(FILES_HPP_rwxml
        rwxml/DependencyGraph.hpp
        rwxml/MultipleFileIterator.hpp
        rwxml/XMLParserUtil.hpp
        rwxml/XMLRWPreParser.hpp
        rwxml/XMLRWParser.hpp
        rwxml/XMLRWLoader.hpp
        rwxml/XMLErrorHandler.hpp
        rwxml/XMLParser.hpp
        rwxml/XML.hpp
    )
    set(FILES_HPP_dom
        dom/DOMBasisTypes.hpp
        dom/DOMPropertyMapFormat.hpp
        dom/DOMPropertyMapLoader.hpp
        dom/DOMPropertyMapSaver.hpp
        dom/DOMPathLoader.hpp
        dom/DOMPathSaver.hpp
        dom/DOMTrajectoryLoader.hpp
        dom/DOMTrajectorySaver.hpp
        dom/DOMProximitySetupLoader.hpp
        dom/DOMProximitySetupSaver.hpp
        dom/DOMWorkCellSaver.hpp
    )
    set(FILES_HPP_colsetup colsetup/CollisionSetupLoader.hpp)
    set(FILES_HPP_image image/PGMLoader.hpp image/PPMLoader.hpp image/RGBLoader.hpp ImageLoader.hpp)

    if(RW_HAVE_ASSIMP)
        set(FILES_CPP ${FILES_CPP} model3d/LoaderAssimp.cpp)

        set(FILES_HPP_model3d ${FILES_HPP_model3d} model3d/LoaderAssimp.hpp)
    endif()

    set(FILES_HPP_xml)
    if(RW_HAVE_XERCES)
        set(FILES_CPP
            ${FILES_CPP}
            xml/XercesErrorHandler.cpp
            xml/XercesUtils.cpp
            xml/XMLBasisTypes.cpp
            xml/XMLPathFormat.cpp
            xml/XMLPathLoader.cpp
            xml/XMLPathSaver.cpp
            xml/XMLTrajectoryFormat.cpp
            xml/XMLTrajectoryLoader.cpp
            xml/XMLTrajectorySaver.cpp
            xml/XMLPropertyFormat.cpp
            xml/XMLPropertyLoader.cpp
            xml/XMLPropertySaver.cpp
            xml/XMLProximitySetupLoader.cpp
        )

        set(FILES_HPP_xml
            xml/XercesErrorHandler.hpp
            xml/XercesUtils.hpp
            xml/XMLBasisTypes.hpp
            xml/XMLPathFormat.hpp
            xml/XMLPathLoader.hpp
            xml/XMLPathSaver.hpp
            xml/XMLTrajectoryFormat.hpp
            xml/XMLTrajectoryLoader.hpp
            xml/XMLTrajectorySaver.hpp
            xml/XMLPropertyFormat.hpp
            xml/XMLPropertyLoader.hpp
            xml/XMLPropertySaver.hpp
            xml/XMLProximitySetupLoader.hpp
        )
    endif()

    rw_add_library(
        ${SUBSYS_NAME}
        ${FILES_CPP}
        ${FILES_HPP}
        ${FILES_HPP_colsetup}
        ${FILES_HPP_dom}
        ${FILES_HPP_image}
        ${FILES_HPP_model3d}
        ${FILES_HPP_path}
        ${FILES_HPP_rwxml}
        ${FILES_HPP_xml}
    )
    rw_add_includes(${SUBSYS_NAME} "rw/loaders" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/colsetup" ${FILES_HPP_colsetup})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/dom" ${FILES_HPP_dom})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/image" ${FILES_HPP_image})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/model3d" ${FILES_HPP_model3d})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/path" ${FILES_HPP_path})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/rwxml" ${FILES_HPP_rwxml})
    rw_add_includes(${SUBSYS_NAME} "rw/loaders/xml" ${FILES_HPP_xml})
    rw_add_includes(${SUBSYS_NAME} "rw" ../loaders.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})
    target_include_directories(
        ${SUBSYS_NAME}
        PUBLIC ${Boost_INCLUDE_DIR} ${XERCESC_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
               $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    if(RW_HAVE_XERCES)
        target_link_libraries(${SUBSYS_NAME} PUBLIC ${XERCESC_LIBRARIES})
    endif()
    if(RW_HAVE_ASSIMP)
        target_link_libraries(${SUBSYS_NAME} PRIVATE ${ASSIMP_LIBRARIES})
    endif()

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
