/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWS_SWIG_REMOTETYPES_HPP_
#define RWS_SWIG_REMOTETYPES_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
/*
#ifdef __cplusplus
extern "C" {
#endif
    int luaopen_rws(struct lua_State* L); // declare the wrapped module
#ifdef __cplusplus
}
#endif
*/
namespace rws { namespace swig {

    template< typename T > std::string toString (const T& x)
    {
        std::ostringstream buf;
        buf << x;
        return buf.str ();
    }

    typedef rws::RobWorkStudio RobWorkStudio;
    typedef rws::RobWorkStudioPlugin RobWorkStudioPlugin;

    typedef rws::RWStudioView3D RWStudioView3D;

    // for now we add all static functions here
    RobWorkStudio* getRobWorkStudio ();
    RobWorkStudio* getRobWorkStudioFromQt ();

    /**
     * @brief set current RobWorkStudio instance
     */
    void setRobWorkStudio (RobWorkStudio* rwstudio);

    /// These functions all work on the current RobWorkStudio state

    #if defined(SWIGPYTHON)
        rw::core::Ptr< RobWorkStudio > getRobWorkStudioInstance (const std::string& args = "RobWorkStudio --exclude-plugins libsdurws_pythoneditor.so");
    #else
        rw::core::Ptr< RobWorkStudio > getRobWorkStudioInstance (const std::string& args = "RobWorkStudio");
    #endif
    void closeRobWorkStudio ();

    bool isRunning ();

    const rw::kinematics::State& getState ();
    void setState (rw::kinematics::State& state);
    rw::core::Ptr< rw::models::Device > findDevice (const std::string& name);
    rw::core::Ptr< rw::models::JointDevice > findJointDevice (const std::string& name);
    rw::core::Ptr< rw::models::SerialDevice > findSerialDevice (const std::string& name);
    rw::core::Ptr< rw::models::TreeDevice > findTreeDevice (const std::string& name);
    rw::core::Ptr< rw::models::ParallelDevice > findParallelDevice (const std::string& name);
    rw::kinematics::Frame* findFrame (const std::string& name);
    rw::kinematics::MovableFrame* findMovableFrame (const std::string& name);
    rw::kinematics::FixedFrame* findFixedFrame (const std::string& name);

    void moveTo (rw::kinematics::MovableFrame* mframe, rw::math::Transform3D< double > wTframe);
    void moveTo (rw::kinematics::Frame* frame, rw::kinematics::MovableFrame* mframe,
                 rw::math::Transform3D< double > wTtcp);
    void moveTo (const std::string& fname, const std::string& mname,
                 rw::math::Transform3D< double > wTframe);

    // utility functions for
    rw::math::Q getQ (rw::core::Ptr< rw::models::Device > dev);
    void setQ (rw::core::Ptr< rw::models::Device > dev, rw::math::Q);

    void setTransform (rw::kinematics::Frame* mframe, rw::math::Transform3D< double > wTframe);

    rw::math::Transform3D< double > wTf (rw::kinematics::Frame* frame);
    rw::math::Transform3D< double > wTf (const std::string& name);
    rw::math::Transform3D< double > fTf (rw::kinematics::Frame* frame, rw::kinematics::Frame* to);
    rw::math::Transform3D< double > fTf (const std::string& from, const std::string& to);

    /**
     * @brief add geometry to an existing frame or object with name objName
     * @param frameName
     * @param geom
     */
    void addGeometry (const std::string& objName, rw::geometry::Geometry::Ptr geom);

    /**
     * @brief adds an rigid object to the scene. If a frame with the objName allready exist then
     * the object will not be created.
     * @param objName
     * @param geom
     */
    void addObject (const std::string& objName, rw::geometry::Geometry::Ptr geom);
    void removeObject (const std::string& objName);

}}    // namespace rws::swig

#endif /* REMOTETYPES_HPP_ */
