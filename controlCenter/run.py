from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *
from cgshop2021_pyutils import Instance
import json

def main():
    instances_id = [i for i in range(170, 180)]
    instances = load_all_instances()
    f = open("analyzed_data.json", "r")
    analyzed_data = json.load(f)
    f.close()
    f = open("analyzed_data.json", "w")

    for id in instances_id:
        instance = instances[id]
        print("Start instance: ", instance.name, "(number:"+str(id)+")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = 40 * num_of_robots
        max_sum = 10 * max_makespan

        controlCenter = ControlCenter(instance, out_path, max_makespan, max_sum)
        analyzed_data[id] = controlCenter.run_all()


    json.dump(analyzed_data, f)
    f.close()

if __name__ == "__main__":
    main()
    print("Done!")