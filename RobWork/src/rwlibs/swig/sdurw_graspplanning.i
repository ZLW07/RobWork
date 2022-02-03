%module sdurw_graspplanning

%include <rwlibs/swig/swig_macros.i>

%include <stl.i>
%include <std_vector.i>



%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>

%{
    #include <rw/geometry/IndexedTriArray.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}

%{
    #include <rw/graspplanning/GraspQualityMeasure3D.hpp>
%}
%include <rw/graspplanning/GraspQualityMeasure3D.hpp>

%{
    #include <rw/graspplanning/ApproachMeasure3D.hpp>
%}
%include <rw/graspplanning/ApproachMeasure3D.hpp>

%{
    #include <rw/graspplanning/CMDistCCPMeasure3D.hpp>
%}
%include <rw/graspplanning/CMDistCCPMeasure3D.hpp>

%{
    #include <rw/graspplanning/ContactValidateFilter.hpp>
%}
%include <rw/graspplanning/ContactValidateFilter.hpp>

%{
    #include <rw/graspplanning/GraspValidateFilter.hpp>
%}
%include <rw/graspplanning/GraspValidateFilter.hpp>


%{
    #include <rw/graspplanning/CompositeContactFilter.hpp>
%}
%include <rw/graspplanning/CompositeContactFilter.hpp>

%{
    #include <rw/graspplanning/CompositeGraspFilter.hpp>
%}
%include <rw/graspplanning/CompositeGraspFilter.hpp>

%{
    #include <rw/graspplanning/ContactDistThresFilter.hpp>
%}
%include <rw/graspplanning/ContactDistThresFilter.hpp>

%{
    #include <rw/graspplanning/Contour2DInfoMap.hpp>
%}
%include <rw/graspplanning/Contour2DInfoMap.hpp>

%{
    #include <rw/graspplanning/CurvatureThresFilter.hpp>
%}
%include <rw/graspplanning/CurvatureThresFilter.hpp>

%{
    #include <rw/graspplanning/DiceContactG3D.hpp>
%}
%include <rw/graspplanning/DiceContactG3D.hpp>

%{
    #include <rw/graspplanning/Grasp2D.hpp>
%}
%include <rw/graspplanning/Grasp2D.hpp>

%{
    #include <rw/graspplanning/Grasp3D.hpp>
%}
%include <rw/graspplanning/Grasp3D.hpp>

%{
    #include <rw/graspplanning/GraspTable.hpp>
%}
%include <rw/graspplanning/GraspTable.hpp>

%{
    #include <rw/graspplanning/PlaneClearanceFilter.hpp>
%}
%include <rw/graspplanning/PlaneClearanceFilter.hpp>

%{
    #include <rw/graspplanning/SemiForceClosureFilter.hpp>
%}
%include <rw/graspplanning/SemiForceClosureFilter.hpp>

%{
    #include <rw/graspplanning/WrenchMeasure3D.hpp>
%}
%include <rw/graspplanning/WrenchMeasure3D.hpp>
