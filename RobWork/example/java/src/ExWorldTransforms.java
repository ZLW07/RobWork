import java.util.Iterator;

import org.robwork.sdurw.*;

public class ExWorldTransforms {
    public static Transform3DdVector worldTransforms(
            FrameVector frames, State state)
    {
        FKTable fk = new FKTable(state);

        Transform3DdVector result = new Transform3DdVector();
        Iterator<Frame> iterator = frames.iterator();
        while(iterator.hasNext()) {
            Frame frame = iterator.next();
            result.add(fk.get(frame));
        }
        return result;
    }
}