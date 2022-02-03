
#include "ScriptTypes.hpp"

#include <map>

using namespace rwsim::swig;

rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc_internal;

std::map< std::string, rwsim::simulator::ThreadSimulator::Ptr > sim_instances_internal;

rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > rwsim::swig::getDynamicWorkCell ()
{
    return dwc_internal;
}

void rwsim::swig::setDynamicWorkCell (rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc)
{
    dwc_internal = dwc;
}

void rwsim::swig::addSimulatorInstance (rw::core::Ptr< ThreadSimulator > sim, const std::string& id)
{
    sim_instances_internal[id] = sim;
}

rw::core::Ptr< ThreadSimulator > rwsim::swig::getSimulatorInstance (const std::string& id)
{
    return sim_instances_internal[id];
}

void rwsim::swig::removeSimulatorInstance (const std::string& id)
{
    sim_instances_internal[id] = NULL;
}

std::vector< std::string > rwsim::swig::getSimulatorInstances ()
{
    typedef std::map< std::string, rw::core::Ptr< ThreadSimulator > >::value_type PairVals;
    std::vector< std::string > result;
    for (PairVals pair : sim_instances_internal) {
        if (pair.second != NULL)
            result.push_back (pair.first);
    }
    return result;
}

rw::core::Ptr< ThreadSimulator > rwsim::swig::getSimulatorInstance ()
{
    typedef std::map< std::string, rw::core::Ptr< ThreadSimulator > >::value_type PairVals;
    for (PairVals pair : sim_instances_internal) {
        if (pair.second != NULL)
            return pair.second;
    }
    return NULL;
}
