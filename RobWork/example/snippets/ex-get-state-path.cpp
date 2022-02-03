#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>

#include <vector>

using rw::math::Q;
using rw::models::Device;
using rw::kinematics::State;

std::vector<State> getStatePath(
    const Device& device,
    const std::vector<Q>& path,
    const State& common_state)
{
    State state = common_state;

    std::vector<State> result;
    for(const Q& q : path) {
        device.setQ(q, state);
        result.push_back(state);
    }
    return result;
}
