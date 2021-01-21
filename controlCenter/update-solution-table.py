import sys
import os
import json
import pandas as pd

sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")


MAKESPAN_TAG = "MSPAN"
SUM_TAG = "SUM"

algos = {
        "OutAndInByPercentage_BIT": 0,
        "dist_from_grid_BIT": 0,
        "dist_from_grid_desc_BIT": 0,
        "dist_from_target_BIT": 0,
        "dist_from_target_desc_BIT": 0,
        "dist_BFS_BIT": 0,
        "dist_BFS_desc_BIT": 0,
        "rand_BIT": 0
    }


def analyze_solutions():
    sol_path = "../solutions/"
    sol_path = os.path.join("..", "solutions")
    results_table_json_path = "../results_table.json"

    results_f = open(results_table_json_path, "r")
    results_json = json.load(results_f)
    results_f.close()

    for (root, dirs, files) in os.walk(sol_path):
        for dir in dirs:
            instance_result = results_json.get(dir, {"min_makespan": -1,
                                   "min_sum": -1})




            for files in os.walk(os.path.join(root, dir)):
                for file in files[2]:
                    if file == '.DS_Store' or file == 'not_empty.txt':
                        continue
                    if "SUCCESS" in file:

                        left_index = file.find(MAKESPAN_TAG) + len(MAKESPAN_TAG)
                        right_index = file[left_index:].find("_") + left_index
                        temp_makespan = int(file[left_index:right_index])

                        if temp_makespan < instance_result["min_makespan"] or instance_result["min_makespan"] is -1:
                            instance_result["min_makespan"] = temp_makespan

                        left_index = file.find(SUM_TAG) + len(SUM_TAG)
                        right_index = file[left_index:].find(".") + left_index
                        temp_sum = int(file[left_index:right_index])

                        if temp_sum < instance_result["min_sum"] or instance_result["min_sum"] is -1:
                            instance_result["min_sum"] = temp_sum

                        for algo in algos:
                            if algo in file:
                                algo_results = instance_result.get(algo, {"algo_name": algo, "makespan": -1, "sum": -1})
                                if temp_makespan < algo_results["makespan"] or algo_results["makespan"] is -1:
                                    algo_results["makespan"] = temp_makespan
                                if temp_sum < algo_results["sum"] or algo_results["sum"] is -1:
                                    algo_results["sum"] = temp_sum
                                instance_result[algo] = algo_results
                                break
            results_json[dir] = instance_result

    results_f = open(results_table_json_path, "w")
    json.dump(results_json, results_f)
    results_f.close()

    return results_json


def analyze(to_console=False, to_file=True):
    data = analyze_solutions()
    if to_console:
        for i in data:
            print(i)

    if to_file:
        out_file_str = "../results_table.csv"
        df = pd.read_json('../results_table.json')
        df.to_csv(out_file_str, index=None)

analyze()