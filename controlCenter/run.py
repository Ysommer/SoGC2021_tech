from controlCenter import ControlCenter
from solution.solution import *


def main():
    in_path = "C:\\SoCG\\SoGC2021_tech\\instances\\instances_01\\uniform\\small_free_000_10x10_30_30.instance.json"
    out_path = "C:\\SoCG\\SoGC2021_tech\\solutions\\instances_01\\uniform\\small_free_000_10x10_30_30"
    paths = [in_path, out_path]
    controlCenter = ControlCenter(paths)
    print("Hello World!")


if __name__ == "__main__":
    main()