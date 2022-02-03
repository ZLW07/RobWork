from sdurw import *

def printKinematicTree(frame, state, parentTransform, level):
    transform = parentTransform * frame.getTransform(state);

    for i in range(level): 
        print(' ', end = '')

    print(frame.getName() + " at " + str(transform.P()))

    for child in frame.getChildren(state):
        printKinematicTree(child, state, transform, level + 1);

def printDefaultWorkCellStructure(workcell):
    printKinematicTree(
        workcell.getWorldFrame(),
        workcell.getDefaultState(),
        Transform3Dd.identity(),
        0);
