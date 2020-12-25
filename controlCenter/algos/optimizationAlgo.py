from solution.solution import Solution
from copy import deepcopy

class OptimizationAlgo:
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, obstacles: list, preprocess=None, name="", print_info=True):
        self.instance_name = instance_name
        self.solution = solution
        self.robots = deepcopy(robots)
        self.targets = targets
        self.obs = obstacles
        self.preprocess = preprocess
        self.name = name
        self.print_info = print_info



