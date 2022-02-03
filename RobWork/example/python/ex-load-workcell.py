from sdurw import *
import sys

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 " + sys.argv[0] + " <workcell>")
        sys.exit(1)

    workcell = WorkCellLoaderFactory.load(sys.argv[1]);
    if workcell.isNull():
        print("WorkCell could not be loaded.")
        sys.exit(1)

    print("Workcell " + workcell.getName() + " successfully loaded.")
    sys.exit(0)