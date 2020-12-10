from defines import SolutionResult
import json


class Solution:
    def __init__(self, instance_name: str):
        self.out = {"instance": instance_name,
                    "steps": []}
        self.result = SolutionResult.RUNNING

    def update_robot(self, robot_id: int, direction: chr, current_turn: int):
        self.out["steps"][current_turn][str(robot_id)] = direction

    def print(self, out_path):
        with open(out_path, 'w') as f:
            json.dump(self.out, f)
