from solution.solution import Solution
from copy import deepcopy
import abc

class OptimizationAlgo(abc.ABC):
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, obstacles: list, size: int, preprocess=None, name="", print_info=True, data_bundle=None):
        self.instance_name = instance_name
        self.solution = deepcopy(solution)
        self.robots = deepcopy(robots)
        self.targets = targets
        self.obs = obstacles
        self.size = size
        self.preprocess = preprocess
        self.name = name
        self.print_info = print_info
        self.solution.out["algo_name"] += name

    @abc.abstractmethod
    def run(self):
        ...


