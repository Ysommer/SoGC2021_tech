from ..defines import SolutionResult


class Solution:
    def __init__(self, instance_name: str):
        self.out = {"instance": instance_name,
                    "steps": []}
        self.result = SolutionResult.RUNNING

    def update_robot(self, robot_id: int, direction: chr, current_turn: int):
        self.out["steps"][current_turn][str(robot_id)] = direction