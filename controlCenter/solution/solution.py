from ..defines import SolutionResult


class Solution:
    def __init__(self, instance_name: str):
        self.out = {"instance": instance_name,
                    "steps": []}
        self.result = SolutionResult.RUNNING


