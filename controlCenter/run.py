from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *

def main():
    instances_id = [143]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        out_path = "../solutions/" + instance.name + "/"
        controlCenter = ControlCenter(instance, out_path, 30000, 30000)
        controlCenter.run_all()

if __name__ == "__main__":
    main()
    print("Done!")