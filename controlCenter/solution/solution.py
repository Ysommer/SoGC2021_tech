from defines import SolutionResult
import json


class Solution:
    def __init__(self, instance_name: str, algo_name: str):
        self.out = {"instance": instance_name,
                    "steps": [],
                    "result": SolutionResult.RUNNING.name,
                    "makespan": 0,
                    "sum": 0,
                    "algo_name": algo_name}

    def put_result(self, result: SolutionResult, makespan: int, sum: int):
        self.out["result"] = result.name
        self.out["makespan"] = makespan
        self.out["sum"] = sum

    def update_robot(self, robot_id: int, direction: chr, current_turn: int):
        self.out["steps"][current_turn][str(robot_id)] = direction

    def output(self, out_path, name):
        out_path += name
        out_path += "_" + self.out["algo_name"]
        out_path += "_" + self.out["result"]
        out_path += "_MSPAN" + str(self.out["makespan"])
        out_path += "_SUM" + str(self.out["sum"])
        out_path += ".json"

        with open(out_path, 'w') as f:
            json.dump(self.out, f)

    def print(self):
        print('###################')
        print("result: ", self.out["result"])
        print("makespan: ", self.out["makespan"])
        print("sum: ", self.out["sum"])
        print('\n')
