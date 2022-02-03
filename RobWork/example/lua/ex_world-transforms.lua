function worldTransforms(frames, state)
    fk = FKTable(state);

    result = Transform3DdVector()
    for i = 0,frames:size()-1 do
        result:push_back(fk:get(frames[i]))
    end
    return result
end