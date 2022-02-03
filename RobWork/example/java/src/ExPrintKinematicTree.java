import org.robwork.sdurw.*;

public class ExPrintKinematicTree {
    public static void printKinematicTree(
            Frame frame,
            State state,
            Transform3Dd parentTransform,
            int level)
    {
        final Transform3Dd transform = parentTransform.multiply(frame.getTransform(state));

        for (int i = 0; i < level; i++) {
            System.out.print(" ");
        }
        System.out.println(frame.getName() + " at " + transform.P());

        FrameVector children = frame.getChildren(state);
        for (int i = 0; i < children.size(); i++) {
            printKinematicTree(children.get(i), state, transform, level + 1);
        }
    }

    public static void printDefaultWorkCellStructure(WorkCellPtr workcell)
    {
        printKinematicTree(
                workcell.getWorldFrame(),
                workcell.getDefaultState(),
                Transform3Dd.identity(),
                0);
    }

}
