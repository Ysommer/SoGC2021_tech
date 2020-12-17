from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *


def main():
    instances_id = [i for i in range(161, 170)]
    instances = load_all_instances()
    analyzed = []

    for id in instances_id:
        instance = instances[id]
        print("Start instance: ", instance.name, "(number:"+str(id)+")")
        out_path = "../solutions/" + instance.name + "/"
        controlCenter = ControlCenter(instance, out_path, 50000, 500000)
        analyzed.append(controlCenter.run_all())

    print(analyzed)

if __name__ == "__main__":
    main()
    print("Done!")