from controlCenter import ControlCenter
from solution.solution import *


def main():
    in_path = "../instances/instances_01/uniform/medium_free_016_50x50_70_1750.instance.json"
    out_path = "../solutions/instances_01/uniform/medium_free_016_50x50_70_1750/"
    paths = [in_path, out_path]
    controlCenter = ControlCenter(paths, 10000, 10000)
    controlCenter.run_all()

if __name__ == "__main__":
    main()
    print("Done!")