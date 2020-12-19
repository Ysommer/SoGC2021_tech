from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *
from cgshop2021_pyutils import Instance
import json
from Utils.compress_solutions_and_validate import compress_solutions_and_validate
from Utils.solution_analyzer import analyze_solutions


def main():
    instances_id = [i for i in range(161, 175)]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        print("Start instance: ", instance.name, "(number:"+str(id)+")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = 40 * num_of_robots
        max_sum = 10 * max_makespan

        control_center = ControlCenter(instance, out_path, max_makespan, max_sum)
        control_center.run_all()


def make_a_zip():
    compress_solutions_and_validate()


def analyze(to_console=True, to_file=False):
    data = analyze_solutions(False)
    if to_console:
        for i in data:
            print(i)

    if to_file:
        out_file_str = "analyzed_data.csv"
        out_file = open(out_file_str, "w")
        data = analyze_solutions(False)
        for i in data:
            print(i, file=out_file)


if __name__ == "__main__":
    main()
    analyze()
    print("Done!")