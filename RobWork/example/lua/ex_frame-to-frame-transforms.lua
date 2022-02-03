function frameToFrameTransforms(a, b, tree_structure, states)
    fk = FKRange(a, b, tree_structure);

    result = Transform3DdVector()
    for i = 0,states:size()-1 do
        result:push_back(fk:get(states[i]))
    end
    return result
end