%module sdurw_proximity

%include <stl.i>
%include <std_vector.i>
%include <rwlibs/swig/swig_macros.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_models.i>

%import <rwlibs/swig/ext_i/std.i>

%{
    #include <rw/models/Joint.hpp>
    #include <rw/models/PrismaticSphericalJoint.hpp>
    #include <rw/models/SphericalJoint.hpp>
    #include <rw/models/UniversalJoint.hpp>
    #include <rw/models/VirtualJoint.hpp>
    #include <rw/models/DependentJoint.hpp>
    #include <rw/models/DependentRevoluteJoint.hpp>
    #include <rw/models/RevoluteJoint.hpp>
    #include <rw/models/DependentPrismaticJoint.hpp>
    #include <rw/models/PrismaticJoint.hpp>
    #include <rw/models/PrismaticUniversalJoint.hpp>
    #include <rw/models/Object.hpp>
    #include <rw/models/WorkCell.hpp>
    #include <rw/kinematics/MovableFrame.hpp>
    #include <rw/kinematics/FixedFrame.hpp>
    #include <rw/geometry/IndexedTriMesh.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
%}

%nodefaultctor ProximityFilterStrategy;
%{
    #include <rw/proximity/ProximityFilterStrategy.hpp>
%}
%include <rw/proximity/ProximityFilterStrategy.hpp>
NAMED_OWNEDPTR(ProximityFilterStrategy,rw::proximity::ProximityFilterStrategy)

%{
    #include <rw/proximity/BasicFilterStrategy.hpp>
%}
%include <rw/proximity/BasicFilterStrategy.hpp>


%{
    #include <rw/proximity/ProximityCalculator.hpp>
    
    #include <rw/proximity/DistanceStrategy.hpp>
    #include <rw/proximity/DistanceMultiStrategy.hpp>
    #include <rw/proximity/CollisionStrategy.hpp>
%}
%include <rw/proximity/ProximityCalculator.hpp>
NAMED_OWNEDPTR(ProximityCalculatorDistanceMulti,rw::proximity::ProximityCalculator<rw::proximity::DistanceMultiStrategy>);

%nodefaultctor CollisionDetectorQueryType;
%inline %{
    //! @brief types of collision query
    struct CollisionDetectorQueryType {
        static const int AllContactsFullInfo = 0;
        static const int AllContactsNoInfo = 1;
        static const int FirstContactFullInfo = 2;
        static const int FirstContactNoInfo = 3;
    };
%}

%rename(CollisionDetectorQueryResult) rw::proximity::CollisionDetector::QueryResult;
%nodefaultctor CollisionDetector;
%{
    #include <rw/proximity/ProximityData.hpp>
    #include <rw/proximity/CollisionDetector.hpp>
%}
%include <rw/proximity/CollisionDetector.hpp>
NAMED_OWNEDPTR(CollisionDetector,rw::proximity::CollisionDetector);
NAMED_OWNEDPTR(ProximityCalculatorCollision,rw::proximity::ProximityCalculator<rw::proximity::CollisionStrategy>);
%extend rw::proximity::CollisionDetector::QueryResult {
    /**
     * @brief get a copy of the items presented in \b _fullInfo as a pointer
     * @param index the index in the vector
     * @return a pointer to a copy of the ProximityStrategyData
     */
    rw::core::Ptr<rw::proximity::ProximityStrategyData> getFullInfo(unsigned int i){
        return rw::core::ownedPtr(new rw::proximity::ProximityStrategyData($self->_fullInfo[i]));
    }

}

%ignore rw::proximity::CollisionSetup::merge(rw::proximity::CollisionSetup const &,rw::proximity::CollisionSetup const &);
%{
    #include <rw/proximity/CollisionSetup.hpp>
%}
%include <rw/proximity/CollisionSetup.hpp>

%nodefaultctor ProximityStrategy;
%rename (ProximityStrategyFactory) rw::proximity::ProximityStrategy::Factory;
%{
    #include <rw/proximity/ProximityStrategy.hpp>
%}
%template (ExtensionPointProximityStrategy) rw::core::ExtensionPoint<rw::proximity::ProximityStrategy>;
%include <rw/proximity/ProximityStrategy.hpp>
NAMED_OWNEDPTR(ProximityStrategy,rw::proximity::ProximityStrategy);

%rename (CollisionStrategyFactory) rw::proximity::CollisionStrategy::Factory;
%rename(CollisionStrategyResult) rw::proximity::CollisionStrategy::Result;
%inline %{
    //! @brief types of collision query
    struct CollisionStrategyQueryType {
        static const int FirstContact = 0; 
        static const int AllContacts = 0;
    };
%}
%nodefaultctor CollisionStrategy;
%{
    #include <rw/proximity/CollisionStrategy.hpp>
%}
%template (ExtensionPointCollisionStrategy) rw::core::ExtensionPoint<rw::proximity::CollisionStrategy>;
%include <rw/proximity/CollisionStrategy.hpp>
NAMED_OWNEDPTR(CollisionStrategy,rw::proximity::CollisionStrategy);
%template(CollisionStrategyCollisionPairVector) std::vector<rw::proximity::CollisionStrategy::Result::CollisionPair>;


%rename (CollisionToleranceStrategyFactory) rw::proximity::CollisionToleranceStrategy::Factory;
%nodefaultctor CollisionToleranceStrategy;
%{
    #include <rw/proximity/CollisionToleranceStrategy.hpp>
%}
%template (ExtensionPointCollisionToleranceStrategy) rw::core::ExtensionPoint<rw::proximity::CollisionToleranceStrategy>;
%include <rw/proximity/CollisionToleranceStrategy.hpp>
NAMED_OWNEDPTR(CollisionToleranceStrategy,rw::proximity::CollisionToleranceStrategy);

%{
    #include <rw/proximity/DistanceCalculator.hpp>
%}
%include <rw/proximity/DistanceCalculator.hpp>
NAMED_OWNEDPTR(ProximityCalculatorDistance,rw::proximity::ProximityCalculator<rw::proximity::DistanceStrategy>);
NAMED_OWNEDPTR(DistanceCalculator,rw::proximity::DistanceCalculator);

%rename (DistanceMultiStrategyFactory) rw::proximity::DistanceMultiStrategy::Factory;
%rename(DistanceMultiStrategyResult) rw::proximity::DistanceMultiStrategy::Result;
%nodefaultctor DistanceMultiStrategy;
%{
    #include <rw/proximity/DistanceMultiStrategy.hpp>
%}
%template (ExtensionPointDistanceMultiStrategy) rw::core::ExtensionPoint<rw::proximity::DistanceMultiStrategy>;
%include <rw/proximity/DistanceMultiStrategy.hpp>
NAMED_OWNEDPTR(DistanceMultiStrategy,rw::proximity::DistanceMultiStrategy);
%template(DistanceMultiStrategyResultVector) std::vector<rw::proximity::DistanceMultiStrategy::Result>;


%rename (DistanceStrategyFactory) rw::proximity::DistanceStrategy::Factory;
%rename(DistanceStrategyResult) rw::proximity::DistanceStrategy::Result;
%nodefaultctor DistanceStrategy;
%{
    #include <rw/proximity/DistanceStrategy.hpp>
%}
%template (ExtensionPointDistanceStrategy) rw::core::ExtensionPoint<rw::proximity::DistanceStrategy>;
%include <rw/proximity/DistanceStrategy.hpp>
NAMED_OWNEDPTR(DistanceStrategy,rw::proximity::DistanceStrategy);
%template(DistanceStrategyResultVector) std::vector<rw::proximity::DistanceStrategy::Result>;


%typemap(in) (void *value) {
        $1 = (void *) $input;
};
%{
    #include <rw/proximity/ProximityCache.hpp>
%}
%include <rw/proximity/ProximityCache.hpp>
NAMED_OWNEDPTR(ProximityCache,rw::proximity::ProximityCache);

%{
    #include <rw/proximity/ProximityData.hpp>
%}
%include <rw/proximity/ProximityData.hpp>
NAMED_OWNEDPTR(ProximityData,rw::proximity::ProximityData);

%{
    #include <rw/proximity/ProximityFilter.hpp>
%}
%include <rw/proximity/ProximityFilter.hpp>
NAMED_OWNEDPTR(ProximityFilter,rw::proximity::ProximityFilter);

%{
    #include <rw/proximity/ProximityModel.hpp>
%}
%include <rw/proximity/ProximityModel.hpp>
NAMED_OWNEDPTR(ProximityModel,rw::proximity::ProximityModel);

%{
    #include <rw/proximity/ProximitySetup.hpp>
%}
%include <rw/proximity/ProximitySetup.hpp>

%nodefaultctor ProximitySetupRule;
%{
    #include <rw/proximity/ProximitySetupRule.hpp>
%}
%include <rw/proximity/ProximitySetupRule.hpp>
%template(ProximitySetupRuleVector) std::vector<rw::proximity::ProximitySetupRule>;

%nodefaultctor ProximityStrategyData;
%{
    #include <rw/proximity/ProximityStrategyData.hpp>
%}
%include <rw/proximity/ProximityStrategyData.hpp>
NAMED_OWNEDPTR(ProximityStrategyData,rw::proximity::ProximityStrategyData);
%template (VectorProximityStrategyData) std::vector<rw::proximity::ProximityStrategyData>;
%template (VectorProximityStrategyDataPtr) std::vector<rw::core::Ptr<rw::proximity::ProximityStrategyData> >;


%rename(RaycasterQueryResult) rw::proximity::Raycaster::QueryResult;
%{
    #include <rw/proximity/Raycaster.hpp>
%}
%include <rw/proximity/Raycaster.hpp>

#if !defined(WIN32)
%{
    #include <rw/proximity/rwstrategy/BVTree.hpp>
%}
%include <rw/proximity/rwstrategy/BVTree.hpp>
%template(BVTreeBinaryBVTreeOBB) rw::proximity::BVTree< rw::proximity::BinaryBVTree< rw::geometry::OBB< double >,rw::geometry::Triangle< double > > >;

%{
    #include <rw/proximity/rwstrategy/BinaryBVTree.hpp>
%}
%include <rw/proximity/rwstrategy/BinaryBVTree.hpp>
%template(BinaryBVTreeOBBTr) rw::proximity::BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double>>;
%template(TraitBinaryTreeOBBTr)  rw::Traits<rw::proximity::BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double>>>;


%{
    #include <rw/proximity/rwstrategy/BVTreeCollider.hpp>
%}
%include <rw/proximity/rwstrategy/BVTreeCollider.hpp>
%template(BVTreeColliderBinaryOBBTr) rw::proximity::BVTreeCollider<rw::proximity::BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double>>>;
%template(BVTreeColliderBinaryOBBTrPtr) rw::core::Ptr<rw::proximity::BVTreeCollider<rw::proximity::BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double>>>>;

#if defined(WIN32)
%ignore rw::proximity::BVTreeCollider<rw::proximity::BinaryBVTree<rw::geometry::OBB<double>, rw::geometry::Triangle<double>>>::colides;
#endif 

%{
    #include <rw/proximity/rwstrategy/BVTreeColliderFactory.hpp>
%}
%include <rw/proximity/rwstrategy/BVTreeColliderFactory.hpp>

%{
    #include <rw/proximity/rwstrategy/BVTreeFactory.hpp>
%}
%include <rw/proximity/rwstrategy/BVTreeFactory.hpp>

%{
    #include <rw/proximity/rwstrategy/BVTreeToleranceCollider.hpp>
%}
%include <rw/proximity/rwstrategy/BVTreeToleranceCollider.hpp>

%{
    #include <rw/proximity/rwstrategy/BinaryIdxBVTree.hpp>
%}
%include <rw/proximity/rwstrategy/BinaryIdxBVTree.hpp>

%{
    #include <rw/proximity/rwstrategy/OBVTreeDFSCollider.hpp>
%}
%include <rw/proximity/rwstrategy/OBVTreeDFSCollider.hpp>

%{
    #include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>
%}
%include <rw/proximity/rwstrategy/ProximityStrategyRW.hpp>

#endif 
