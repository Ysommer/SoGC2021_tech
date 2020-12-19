import os
from solution.solution import SolutionResult
import json

INSTANCES_PATH = "../instances/instances-zip/instances.zip"
INIT_ALGOS_PATH = "../controlCenter/algos/init_algos"
SOLUTIONS_PATH = "../Solutions/"
SOLUTION_ZIP_NAME = "solutions.zip"
SOLUTIONS_ZIP_PATH = ""

MAKESPAN_TAG = "MSPAN"
SUM_TAG = "SUM"


class AlgoAnalyzedData:
    def __init__(self, path: str, name: str):
        self.path = path
        self.name = name
        self.num_of_successes = 0
        self.num_of_failures = 0
        self.solution_results_hist = [0] * (len(SolutionResult))

        self.makespan = -1
        self.sum = -1

    def increment_solution_result(self, sol_name, solution_result: SolutionResult):
        self.solution_results_hist[solution_result.value] += 1
        if solution_result == SolutionResult.SUCCESS:
            self.num_of_successes += 1

            left_index = sol_name.find(MAKESPAN_TAG) + len(MAKESPAN_TAG)
            right_index = sol_name[left_index:].find("_") + left_index
            temp_makespan = int(sol_name[left_index:right_index])

            if temp_makespan < self.makespan or self.makespan is -1:
                self.makespan = temp_makespan

            left_index = sol_name.find(SUM_TAG) + len(SUM_TAG)
            right_index = sol_name[left_index:].find(".") + left_index
            temp_sum = int(sol_name[left_index:right_index])

            if temp_sum < self.sum or self.sum is -1:
                self.sum = temp_sum
        else:
            self.num_of_failures += 1



    def __str__(self):
        out = "<" + self.name \
                + ": Successes: " + str(self.num_of_successes) \
                + "| Failures: " + str(self.num_of_failures) \
                + "| Makespan: " + str(self.makespan) \
                + "| Sum: " + str(self.sum) + ">"

        return out

    def __repr__(self):
        return str(self)


class InstanceAnalyzedData:
    def __init__(self, path, name: str = "", list_of_init_algos: list = None):
        self.name = name
        self.path = path
        self.data = {}
        self.min_makespan = -1
        self.min_sum = -1

        if list_of_init_algos is None:
            list_of_init_algos = []
            for algo in os.listdir(INIT_ALGOS_PATH):
                if algo[0] == "_":
                    continue
                list_of_init_algos.append(algo[:(-1) * len(".py")])

        for algo in list_of_init_algos:
            self.data[algo] = AlgoAnalyzedData(self.path, algo)

    def run(self):
        for sol in os.listdir(self.path):
            for algo in self.data:
                if algo in sol:
                    for sol_res in SolutionResult:
                        if sol_res.name in sol:
                            self.data[algo].increment_solution_result(sol, sol_res)

    def print(self):
        print(str(self.data))

    def output(self, json_path):
        json_file = open(json_path, "w+")
        json.dump(self.data, json_file)

    def __str__(self):
        out = self.name + ": "
        for i in self.data:
            out += str(self.data[i]) + ", "

        return out[:(-1)*len(", ")]

    def __repr__(self):
        return str(self)


def analyze_solutions(to_print: bool = True):
    instances = []
    for ins in os.listdir(SOLUTIONS_PATH):
        instances.append(InstanceAnalyzedData(SOLUTIONS_PATH+ins, ins))
        instances[-1].run()
        if(to_print):
            instances[-1].print()

    return instances