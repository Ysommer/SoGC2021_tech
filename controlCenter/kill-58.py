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
    instances_id = [58]
    instances = load_all_instances()

    for id in instances_id:
        instance = instances[id]
        print("====================")
        print("Start instance: ", instance.name, "(number:" + str(id) + ")")
        out_path = "../solutions/" + instance.name + "/"

        try:
            control_center = jj_control_center_initiate(instance, out_path, sys.argv[1])
            control_center.run_all(print_only_success=False, stop_on_success=False, validate=False, opt_iters=1)
        except Exception as e:
            print(e)
            traceback.print_exc()
        print()

def jj_control_center_initiate(instance, out_path, algo_id):
    control_center = ControlCenter(instance, out_path, -1, -1, print_init_sol=True)
    control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                 data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS",
                                              "descending_order": True})
    data_bundles = [
        {"no_bs": False, "source_min": 135, "grid_limit": 35000, "goal_raise": 128},
        {"no_bs": False, "source_min": 844, "grid_limit": 35000, "goal_raise": 32},
        {"no_bs": True, "source_min": 844, "grid_limit": 35000, "goal_raise": 32},
        {"no_bs": True, "source_min": 844, "grid_limit": 35000, "goal_raise": 64}
    ]

    control_center.add_opt_algo(BFS_in_time, data_bundle=data_bundles[algo_id])
    return control_center

if __name__ == "__main__":
    main()

    print("Done!")
