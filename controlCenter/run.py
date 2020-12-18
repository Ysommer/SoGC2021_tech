from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *
from cgshop2021_pyutils import Instance
import json
from Utils.compress_solutions_and_validate import compress_solutions_and_validate


def main():
    instances_id = [i for i in range(161, 180)]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        print("Start instance: ", instance.name, "(number:"+str(id)+")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = 40 * num_of_robots
        max_sum = 10 * max_makespan

        controlCenter = ControlCenter(instance, out_path, max_makespan, max_sum)
        controlCenter.run_all()




def make_a_zip():
    compress_solutions_and_validate()


def analyze():
    out_file = "analyzed_data.csv"



if __name__ == "__main__":
    main()
    print("Done!")