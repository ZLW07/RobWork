
SET(FILES_CPP
    bstrap/core/Brain.cpp
    bstrap/core/BrainState.cpp
    bstrap/core/Condition.cpp
    bstrap/core/MotorProgram.cpp
    bstrap/core/Predictor.cpp
    bstrap/core/Schema.cpp
    bstrap/core/Abstraction.cpp
    bstrap/core/ActionGoal.cpp
    bstrap/core/ActionTree.cpp
    bstrap/core/Memory.cpp
    
    bstrap/abstractions/VisionSensor.cpp
    
    bstrap/mp/GraspMotorProgram.cpp
)

SET(FILES_HPP
    bstrap/core/Brain.hpp
    bstrap/core/BrainState.hpp
    bstrap/core/Condition.hpp
    bstrap/core/MotorProgram.hpp
    bstrap/core/Predictor.hpp
    bstrap/core/Schema.hpp
    bstrap/core/Abstraction.hpp
    bstrap/core/ActionGoal.hpp
    bstrap/core/ActionTree.hpp
    bstrap/core/Memory.hpp
    
    bstrap/abstractions/VisionSensor.hpp
    
    bstrap/mp/GraspMotorProgram.hpp
)

if(DEFINED Qt6Core_VERSION)
    qt_wrap_cpp(MocSrcFiles bstrap/core/Brain.hpp)
else()
    qt5_wrap_cpp(MocSrcFiles bstrap/core/Brain.hpp)
endif()

add_library(bstrap ${FILES_CPP} ${MocSrcFiles})
target_link_libraries(bstrap ${QT_LIBRARIES})
