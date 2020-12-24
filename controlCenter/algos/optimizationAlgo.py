from solution.solution import Solution

class OptimizationAlgo:
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, preprocess=None, name="", print_info=True):
        self.instance_name = instance_name
        self.solution = solution
        self.robots = robots
        self.targets = targets
        self.preprocess = preprocess
        self.name = name
        self.print_info = print_info



