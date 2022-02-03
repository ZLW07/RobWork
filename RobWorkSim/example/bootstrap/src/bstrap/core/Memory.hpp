

#ifndef MEMORY_HPP_
#define MEMORY_HPP_

#include <rw/core/Ptr.hpp>

class BrainState;

/**
 * @brief something that computes abstract knowledge and puts this into the state
 */
class Memory{
public:
    typedef rw::core::Ptr<Memory> Ptr;


    /**
     * @brief adds a brain state to the memory.
     * @param state
     */
    void addState(BrainState& state);
};

#endif
