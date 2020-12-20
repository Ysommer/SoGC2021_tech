from cgshop2021_pyutils import Instance
from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *
from Utils.compress_solutions_and_validate import compress_solutions_and_validate
from Utils.solution_analyzer import analyze_solutions
from Utils.validator import validate_solution_zip
from Utils.compress_solutions_and_validate import compress_solutions, clean_bad_solutions
import json

# Algos
from algos.init_algos.LeftPillar import *
from algos.init_algos.OutAndInBFS import *
from algos.init_algos.BFS import *


def main():
    instances_id = [i for i in range(0, 20)]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        print("Start instance: ", instance.name, "(number:"+str(id)+")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = 35 * num_of_robots
        max_sum = 10 * max_makespan

        control_center = jj_control_center_initiate(instance, out_path, max_makespan, max_sum)

        control_center.run_all(print_only_success=True, stop_on_success=True)


def make_a_zip():
    compress_solutions_and_validate()


def jj_control_center_initiate(instance, out_path, max_makespan, max_sum):
    print_info = True
    data_bundle = None
    control_center = ControlCenter(instance, out_path, -1, -1)
    control_center.add_init_algo(OutAndInBFS, name="_default", print_info=print_info, data_bundle=data_bundle)

    return control_center


def ys_control_center_initiate(instance, out_path , max_makespan, max_sum):
    control_center = ControlCenter(instance, out_path, max_makespan, max_sum)
    for i in range(150):
        control_center.add_init_algo(BFS, name="_"+str(i))
    return control_center


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
    # clean_bad_solutions()
    main()
    # analyze()
    print("Done!")
