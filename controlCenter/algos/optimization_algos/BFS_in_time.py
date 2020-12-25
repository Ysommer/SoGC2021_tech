from algos.optimizationAlgo import OptimizationAlgo

class BFS_in_time(OptimizationAlgo):
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, preprocess=None, name="", print_info=True):
        super().__init__(instance_name, solution, robots, targets, preprocess, name, print_info)