from defines import SolutionResult
import json


class Solution:
    def __init__(self, instance_name: str):
        self.out = {"instance": instance_name,
                    "steps": [],
                    "result": SolutionResult.RUNNING,
                    "makespan": 0,
                    "sum": 0}

    def put_result(self, result: SolutionResult, makespan: int, sum: int):
        self.out["result"] = result
        self.out["makespan"] = makespan
        self.out["sum"] = sum

    def update_robot(self, robot_id: int, direction: chr, current_turn: int):
        self.out["steps"][current_turn][str(robot_id)] = direction

    def output(self, out_path):
        with open(out_path, 'w') as f:
            json.dump(self.out, f)
