import org.robwork.LoaderRW;
import org.robwork.sdurw.*;

public class ExLoadWorkCell {

    public static void main(String[] args) throws Exception {
        LoaderRW.load("sdurw");

        if (args.length != 1) {
            System.out.print("Usage: java " + ExLoadWorkCell.class.getSimpleName());
            System.out.println(" <workcell>");
            System.exit(1);
        }

        WorkCellPtr workcell = WorkCellLoaderFactory.load(args[0]);
        if (workcell.isNull()) {
            System.out.println("WorkCell could not be loaded.");
            System.exit(1);
        }

        System.out.print("Workcell " + workcell.getName());
        System.out.println(" successfully loaded.");
    }

}
