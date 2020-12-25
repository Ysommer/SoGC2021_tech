from algos.optimizationAlgo import OptimizationAlgo
from infrastructure.solGrid import SolGrid
from dataCollection.Generator import *

class BFS_in_time(OptimizationAlgo):
    def __init__(self, instance_name: str, solution: Solution, robots: list,
                 targets: list, obstacles:list, preprocess=None, name="", print_info=True):
        super().__init__(instance_name, solution, robots, targets, preprocess, name, print_info)
        self.sol_grid = SolGrid(self.robots, self.obs, solution)
        self.boundaries = {"N": self.sol_grid.max_y,
                           "S": self.sol_grid.min_y,
                           "W": self.sol_grid.min_x,
                           "E": self.sol_grid.max_x}
        self.robots_to_improve = self.sol_grid.get_last_moving_robots()

    def calc_new_path(self, robot_id):
        return Generator.calc_a_star_path_in_time(self.sol_grid,
                                                  self.boundaries,
                                                  self.robots[robot_id],
                                                  self.targets[robot_id],
                                                  robot_id,
                                                  self.sol_grid.find_last_step_on_location(robot_id,self.targets[robot_id]),
                                                  self.sol_grid.max_time)

    def run(self):
        for robot_id in self.robots_to_improve:
            new_path = self.calc_new_path(robot_id)
            pass