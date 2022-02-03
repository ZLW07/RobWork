function printKinematicTree(frame, state, parentTransform, level)
    transform = parentTransform * frame:getTransform(state);

    indent = ""
    for i=1,level do
        indent = indent .. " "
    end

    print(indent .. frame:getName() .. " at " .. tostring(transform:P()))

    local children = frame:getChildren(state)
    if not children:empty() then
        for i = 0,children:size()-1 do
            child = children[i]
            printKinematicTree(child, state, transform, level + 1);
        end
    end
end

function printDefaultWorkCellStructure(workcell)
    printKinematicTree(
        workcell:getWorldFrame(),
        workcell:getDefaultState(),
        Transform3Dd.identity(),
        0);
end