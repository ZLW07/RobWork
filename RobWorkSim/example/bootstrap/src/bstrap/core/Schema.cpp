#include "Schema.hpp"

void Schema::executeMotorProgram( rw::core::Ptr<rw::core::PropertyMap> parameters, const BrainState& bstate  ){
    _mp->execute( parameters, bstate);
}

