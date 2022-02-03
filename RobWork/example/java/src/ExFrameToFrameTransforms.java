import java.util.Iterator;

import org.robwork.sdurw.*;

public class ExFrameToFrameTransforms {
    public static Transform3DdVector frameToFrameTransforms(
            Frame a, Frame b, State tree_structure, StateVector states)
    {
        FKRange fk = new FKRange(a, b, tree_structure);

        Transform3DdVector result = new Transform3DdVector();
        Iterator<State> iterator = states.iterator();
        while(iterator.hasNext()) {
            State state = iterator.next();
            result.add(fk.get(state));
        }
        return result;
    }
}