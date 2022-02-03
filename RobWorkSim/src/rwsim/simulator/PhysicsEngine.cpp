#include "PhysicsEngine.hpp"

#include <rwsim/rwphysics/RWSimulator.hpp>

using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rw::core;
using namespace rw::core;

PhysicsEngine::Factory::Factory () :
    ExtensionPoint< Dispatcher > ("rwsim.simulator.PhysicsEngine", "Example extension point")
{}

std::vector< std::string > PhysicsEngine::Factory::getEngineIDs ()
{
    std::vector< std::string > ids;
    PhysicsEngine::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        ids.push_back (ext.getProperties ().get ("engineID", ext.name));
    }
    ids.push_back ("RWPhysics");
    return ids;
}

bool PhysicsEngine::Factory::hasEngineID (const std::string& engineID)
{
    if (engineID == "RWPhysics") {
        return true;
    }
    PhysicsEngine::Factory ep;
    std::vector< Extension::Descriptor > exts = ep.getExtensionDescriptors ();
    for (Extension::Descriptor& ext : exts) {
        if (ext.getProperties ().get ("engineID", ext.name) == engineID)
            return true;
    }
    return false;
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine (rw::core::Ptr< DynamicWorkCell > dwc)
{
    // select an engine ID
    std::vector< std::string > ids = getEngineIDs ();

    if (ids.size () == 0)
        RW_THROW ("No available physicsengines!");
    return makePhysicsEngine (ids.front (), dwc);
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine (const std::string& engineID)
{
    if (engineID == "RWPhysics") {
        rwsim::simulator::RWSimulator::Ptr rwphys =
            rw::core::ownedPtr (new rwsim::simulator::RWSimulator ());
        return rwphys;
    }

    PhysicsEngine::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr& ext : exts) {
        if (ext->getProperties ().get ("engineID", ext->getName ()) == engineID) {
            const rw::core::Ptr< const Dispatcher > dispatch =
                ext->getObject ().cast< Dispatcher > ();
            // optionally add any properties options...
            return dispatch->makePhysicsEngine ();
        }
    }
    return NULL;
}

PhysicsEngine::Ptr PhysicsEngine::Factory::makePhysicsEngine (const std::string& engineID,
                                                              rw::core::Ptr< DynamicWorkCell > dwc)
{
    if (engineID == "RWPhysics") {
        rwsim::simulator::RWSimulator::Ptr rwphys =
            rw::core::ownedPtr (new rwsim::simulator::RWSimulator (dwc));
        return rwphys;
    }

    PhysicsEngine::Factory ep;
    std::vector< Extension::Ptr > exts = ep.getExtensions ();
    for (Extension::Ptr& ext : exts) {
        if (ext->getProperties ().get ("engineID", ext->getName ()) == engineID) {
            const rw::core::Ptr< const Dispatcher > dispatch =
                ext->getObject ().cast< Dispatcher > ();
            PhysicsEngine::Ptr engine = dispatch->makePhysicsEngine ();
            engine->load (dwc);
            // optionally add any properties options...
            return engine;
        }
    }
    return NULL;
}
