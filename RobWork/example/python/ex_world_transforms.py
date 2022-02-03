from sdurw import *

def worldTransforms(frames, state):
    fk = FKTable(state);
    
    result = Transform3DdVector();
    for frame in frames:
        result.push_back(fk.get(frame));
    return result;