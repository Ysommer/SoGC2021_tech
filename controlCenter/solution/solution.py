from defines import SolutionResult
import json


class Solution:
    def __init__(self, instance_name: str, algo_name: str, makespan: int = 0, sum: int = 0,
                 result: str = SolutionResult.RUNNING.name, steps: list = None, extra_data= None):
        if steps is None:
            steps = []

        if extra_data is None:
            extra_data = {}

        self.out = {"instance": instance_name,
                    "steps": steps,
                    "result": result,
                    "makespan": makespan,
                    "sum": sum,
                    "algo_name": algo_name,
                    "extra": extra_data}

    def put_result(self, result: SolutionResult, makespan: int, sum: int):
        self.out["result"] = result.name
        self.out["makespan"] = makespan
        self.out["sum"] = sum

    def update_robot(self, robot_id: int, direction: chr, current_turn: int, allow_override: bool = False):
        if not allow_override:
            assert str(robot_id) not in self.out["steps"][current_turn], "You moved the same robot " + str(robot_id) + " twice you stupid ass"
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

    def __str__(self):
        out = '<Solution = ('
        out += "result: " + str(self.out["result"])
        out += ", makespan: " + str(self.out["makespan"])
        out += ", sum: " + str(self.out["sum"]) + ")>"
        return out
