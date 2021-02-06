import sys

sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")

import functools

print = functools.partial(print, flush=True)

import traceback

from cgshop2021_pyutils import Instance
from controlCenter import ControlCenter
from solution.solution import *
from Utils.loadInstances import *
from Utils.compress_solutions_and_validate import *
from Utils.solution_analyzer import analyze_solutions
from Utils.validator import validate_solution_zip
from Utils.compress_solutions_and_validate import compress_solutions, clean_bad_solutions
import json
from WishList import *

# Algos
from algos.init_algos.LeftPillar import *
from algos.init_algos.OutAndInBFS import *
from algos.init_algos.OutAndInByPercentage import *
from algos.init_algos.Chill import *
from algos.init_algos.BFS import *
from algos.optimization_algos.BFS_in_time import *
from algos.optimization_algos.IterSum import *

import numpy


def main():
    instances_id = [i for i in range(141, 180)]
    #instances_id = [149]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        print("====================")
        print("Start instance: ", instance.name, "(number:" + str(id) + ")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = 30 * num_of_robots
        max_sum = 10 * max_makespan

        try:
            control_center = jj_control_center_initiate(instance, out_path, max_makespan, max_sum)
            control_center.run_all(print_only_success=False, stop_on_success=False, validate=False, opt_iters=3, pick_best_sum=2)
        except Exception as e:
            print(e)
            traceback.print_exc()
        print()


def make_a_zip():
    compress_solutions_and_validate()


def jj_control_center_initiate(instance, out_path, max_makespan, max_sum):
    print_info = True
    data_bundle = None
    control_center = ControlCenter(instance, out_path, -1, -1, print_init_sol=True)
    # control_center.add_init_algo(OutAndInByPercentage, print_info=print_info, data_bundle=data_bundle)
    # control_center.add_init_algo(OutAndInByPercentage, print_info=print_info, data_bundle={"sync_insertion": False})
    # control_center.add_opt_algo(BFS_in_time, data_bundle={})
    # for i in range(0, 11):
    #    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False})
    """
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "rand"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid",
                                              "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target",
                                              "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS",
                                              "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": ""})"""
    control_center.add_init_algo(Chill, data_bundle={"dynamic_percent_to_leave_inside": True, "factor_on_binary_search_result": 1}, print_info=False)
    control_center.add_init_algo(Chill, data_bundle={"dynamic_percent_to_leave_inside": True, "factor_on_binary_search_result": 1, "empty_spots_to_move_in_pillar": 2}, print_info=False)
    control_center.add_init_algo(Chill, data_bundle={"dynamic_percent_to_leave_inside": True, "factor_on_binary_search_result": 1, "empty_spots_to_jump_pillar": 3}, print_info=False)
    control_center.add_init_algo(Chill, data_bundle={"dynamic_percent_to_leave_inside": True, "factor_on_binary_search_result": 0.8}, print_info=False)
    control_center.add_init_algo(Chill, data_bundle={"dynamic_percent_to_leave_inside": True, "factor_on_binary_search_result": 0.5}, print_info=False)


    #control_center.add_opt_algo(IterSum)
    return control_center


def ys_control_center_initiate(instance, out_path, max_makespan, max_sum):
    Sol_name = "the_king_94_OutAndInBFS_default_SUCCESS_MSPAN3038_SUM4339.json"
    path = "../solutions/" + instance.name + "/" + Sol_name
    # sol = load_solutions([path])
    max_makespan = -1
    max_sum = -1
    data_bundle = {"sync_insertion": False}
    control_center = ControlCenter(instance, out_path, max_makespan, max_sum)
    control_center.add_init_algo(OutAndInByPercentage, name="_sea_level_", print_info=False, data_bundle=data_bundle)
    control_center.add_opt_algo(BFS_in_time, data_bundle={"noise": 2})
    # for i in range(1):
    #   control_center.add_init_algo(BFS, name="_"+str(i), print_info=True)
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


def analyze_algo_based_on_makespan(path="../Solutions/", to_console=True, to_file=False):
    hist = {
        "OutAndInByPercentage_BIT": 0,
        "dist_from_grid_BIT": 0,
        "dist_from_grid_desc_BIT": 0,
        "dist_from_target_BIT": 0,
        "dist_from_target_desc_BIT": 0,
        "dist_BFS_BIT": 0,
        "dist_BFS_desc_BIT": 0,
        "rand_BIT": 0
    }

    counter = 0

    for root, dirs, files in os.walk(path):
        min_makespan = -1
        makespan_files = []

        for file in files:
            if file == '.DS_Store' or file == 'not_empty.txt':
                continue
            if "SUCCESS" in file:
                left_index = file.find(MAKESPAN_TAG) + len(MAKESPAN_TAG)
                right_index = file[left_index:].find("_") + left_index
                temp_makespan = int(file[left_index:right_index])

                if temp_makespan == min_makespan:
                    makespan_files.append(file)

                if temp_makespan < min_makespan or min_makespan is -1:
                    min_makespan = temp_makespan
                    makespan_files = [file]

        if len(makespan_files) is 0:
            continue

        for algo in hist:
            for file in makespan_files:
                if algo in file:
                    hist[algo] += 1
                    counter += 1
                    break

    if to_console:
        for algo in hist:
            print(algo, ":", str(hist[algo]) + "/" + str(counter), "(" + str((100 * hist[algo]) // counter) + "%)")

    if to_file:
        out_file_str = "analyzed_data.csv"
        out_file = open(out_file_str, "w")
        data = analyze_solutions(False)
        for i in data:
            print(i, file=out_file)

    return hist


def load_solutions(paths: list):
    sols = []
    for path in paths:
        file = open(path, "r")
        sol_json = json.load(file)
        sols.append(
            Solution(sol_json["instance"], sol_json["algo_name"], int(sol_json["makespan"]), int(sol_json["sum"]),
                     sol_json["result"], sol_json["steps"], sol_json["extra"]))
    return sols


def generator_test():
    obs = set()
    obs.add((0, 0))
    obs.add((1, 0))
    obs.add((0, 1))
    x_axis = dict()

    x_axis[-3] = (0, 0)
    x_axis[-2] = (0, 2)
    x_axis[-1] = (0, 2)
    x_axis[0] = (-2, 7)
    x_axis[1] = (0, 4)
    x_axis[2] = (-3, 4)
    x_axis[3] = (0, 6)
    x_axis[4] = (-2, 4)
    x_axis[5] = (2, 2)
    x_axis[6] = (2, 2)
    x_axis[7] = (2, 2)
    x_axis[8] = (2, 2)

    mat = Generator.get_valid_directions_matrix((1, 1), obs, x_axis, 1)
    print(mat)


if __name__ == "__main__":
    clean_bad_solutions()
    main()
    # compress_best_and_send()

    # analyze()
    # analyze_algo_based_on_makespan()
    # compress_solutions_and_validate()

    # WishList.farm_instances(WishListPackagesTypes.TINY, 0)

    """packages = InstancesPackage.get_instances_packages()

    for p in packages:
        print(len(packages[p]))
        print(p,":", packages[p])"""

    print("Done!")
