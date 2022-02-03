function frameToFrameTransform(a, b, state)
    fk = FKRange(a, b, state)
    return fk:get(state)
end