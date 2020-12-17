from controlCenter import ControlCenter
from solution.solution import *


def main():
    in_path = "../instances/instances_01/uniform/small_019_20x20_90_329.instance.json"
    out_path = "../solutions/instances_01/uniform/small_019_20x20_90_329/"
    paths = [in_path, out_path]
    controlCenter = ControlCenter(paths, 10000, 10000)
    controlCenter.run_all()

if __name__ == "__main__":
    main()
    print("Done!")