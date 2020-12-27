from algos.optimizationAlgo import OptimizationAlgo
from infrastructure.solGrid import SolGrid
from dataCollection.Generator import *
from defines import *

"""
BIT run modes:
    1 : improve last stepping robots
    2 : 
"""

class BFS_in_time(OptimizationAlgo):
    def __init__(self, instance_name: str,
                 solution: Solution,
                 robots: list,
                 targets: list,
                 obstacles: list,
                 num_to_improve: int,
                 preprocess=None,
                 name="",
                 print_info=True):
        super().__init__(instance_name,
                         solution,
                         robots,
                         targets,
                         preprocess,
                         "_BIT" + name,
                         print_info)
        self.sol_grid = SolGrid(self.robots, self.obs, solution)
        self.boundaries = {"N": self.sol_grid.max_y,
                           "S": self.sol_grid.min_y,
                           "W": self.sol_grid.min_x,
                           "E": self.sol_grid.max_x}
        self.num_to_improve = num_to_improve
        if self.solution.out["algo_name"].find("BIT_BIT") != -1:
            self.solution.out["algo_name"] = self.solution.out["algo_name"][:(-1)*len("_BIT")]
            self.solution.out["extra"]["BIT_runs"] += 1
        else:
            self.solution.out["extra"]["BIT_runs"] = 1
        self.solution.out["extra"]["old_makespan"] = solution.out["makespan"]
        self.solution.out["extra"]["old_sum"] = solution.out["sum"]
        self.solution.out["extra"]["improved"] = ""
        self.sum = solution.out["sum"]

    def calc_new_path(self, robot_id):
        return Generator.calc_a_star_path_in_time(self.sol_grid,
                                                  self.boundaries,
                                                  self.robots[robot_id],
                                                  self.targets[robot_id],
                                                  robot_id,
                                                  self.sol_grid.find_last_step_on_location(robot_id,self.targets[robot_id]),
                                                  self.sol_grid.max_time)

    def update_solution(self, robot_id, path: list):
        to_delete = []
        for step in range(len(self.solution.out["steps"])):
            if step <= len(path):
                if path[step] != "X":
                    self.solution.out["steps"][step][robot_id] = path[step]
                    self.solution.out["sum"] += 1
                else:
                    if self.solution.out["steps"][step].pop(robot_id, None) is not None:
                        self.solution.out["sum"] -= 1
                        if len(self.solution.out["steps"][step]) == 0:
                            to_delete.append(step)

        for i in to_delete:
            del self.solution.out["steps"][i]


    def run(self):
        improved = []
        to_improve = []
        while len(improved) < self.num_to_improve:
            if len(to_improve) == 0:
                to_improve = self.sol_grid.get_last_moving_robots()
            next_robot_id = to_improve.pop()
            if next_robot_id in improved:
                break
            new_path = self.calc_new_path(next_robot_id)
            assert path is not None, "BIT: a_star failed to find legal path" # should at worst find old path
            self.sol_grid.update_robot_path(next_robot_id, new_path)
            self.update_solution(next_robot_id, new_path)

        self.solution.out["extra"]["improved"] += str(improved) + ","
        self.solution.put_result(SolutionResult.SUCCESS, len(self.solution.out["steps"]), self.sum)
        return self.solution

