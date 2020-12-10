from controlCenter import ControlCenter
from solution.solution import *


def main():
    in_path = "/Users/yonatan/Documents/SoCG/code/instances/instances_01/uniform/small_free_000_10x10_30_30.instance.json"
    out_path = "/Users/yonatan/Documents/SoCG/code/solutions/instances_01/uniform/small_free_000_10x10_30_30"
    paths = [in_path, out_path]
    controlCenter = ControlCenter(paths, 1000, 1000)
    controlCenter.run_all()


if __name__ == "__main__":
    main()
    print("Done!")