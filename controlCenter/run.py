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
from Utils.compress_solutions_and_validate import compress_solutions_and_validate
from Utils.solution_analyzer import analyze_solutions
from Utils.validator import validate_solution_zip
from Utils.compress_solutions_and_validate import compress_solutions, clean_bad_solutions
import json
from WishList import *

# Algos
from algos.init_algos.LeftPillar import *
from algos.init_algos.OutAndInBFS import *
from algos.init_algos.OutAndInByPercentage import *
from algos.init_algos.BFS import *
from algos.optimization_algos.BFS_in_time import *


def main():
    # instances_id = [i for i in range(81, 121)]

    instances_id = [172]
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
            control_center.run_all(print_only_success=False, stop_on_success=False, validate=False)
        except Exception as e:
            print(e)
            traceback.print_exc()
        print()


def make_a_zip():
    compress_solutions_and_validate()


def jj_control_center_initiate(instance, out_path, max_makespan, max_sum):
    print_info = True
    data_bundle = None
    control_center = ControlCenter(instance, out_path, -1, -1, print_init_sol=False)
    # control_center.add_init_algo(OutAndInByPercentage, print_info=print_info, data_bundle=data_bundle)
    #control_center.add_init_algo(OutAndInByPercentage, print_info=print_info, data_bundle={"sync_insertion": False})
    #control_center.add_opt_algo(BFS_in_time, data_bundle={})
    #for i in range(0, 11):
    #    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False})

    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "rand"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_from_grid"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_from_grid", "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_from_target"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_from_target", "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_BFS"})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : "dist_BFS", "descending_order": True})
    control_center.add_init_algo(OutAndInByPercentage, print_info=False, data_bundle={"sync_insertion": False, "secondary_order" : ""})
    control_center.add_opt_algo(BFS_in_time, data_bundle={})
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
    control_center.add_opt_algo(BFS_in_time, data_bundle={"num_to_improve": 30})
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
    # clean_bad_solutions()
    # main()


    # analyze()
    # compress_solutions_and_validate()


    WishList.farm_instances(WishListPackagesTypes.TINY, 1)
    """packages = InstancesPackage.get_instances_packages()

    for p in packages:
        print(len(packages[p]))
        print(p,":", packages[p])"""
    print("Done!")
