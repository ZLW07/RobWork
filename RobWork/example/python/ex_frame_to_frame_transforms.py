from sdurw import *

def frameToFrameTransforms(a, b, tree_structure, states):
    fk = FKRange(a, b, tree_structure);
    
    result = Transform3DdVector();
    for state in states:
        result.push_back(fk.get(state));
    return result;
