from solution.solution import Solution
from copy import deepcopy
import abc

class OptimizationAlgo(abc.ABC):
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, obstacles: list, preprocess=None, name="", print_info=True):
        self.instance_name = instance_name
        self.solution = Solution(solution.out["instance"],
                                 solution.out["algo_name"] + name,
                                 steps=solution.out["steps"])
        self.robots = deepcopy(robots)
        self.targets = targets
        self.obs = obstacles
        self.preprocess = preprocess
        self.name = name
        self.print_info = print_info
        self.solution.out["extra"] = {}

    @abc.abstractmethod
    def run(self):
        ...




