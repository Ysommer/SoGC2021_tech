from algos.optimizationAlgo import OptimizationAlgo
from infrastructure.solGrid import SolGrid
from solution.solution import Solution
from dataCollection.Generator import *
from defines import *

"""
BIT run modes:
    1 : improve last stepping robots
    2 : improve robots by order they arrived on target
"""


class BFS_in_time(OptimizationAlgo):
    def __init__(self, instance_name: str,
                 solution: Solution,
                 robots: list,
                 targets: list,
                 obstacles: list,
                 preprocess=None,
                 name="",
                 print_info=True,
                 data_bundle=None):
        super().__init__(instance_name,
                         solution,
                         robots,
                         targets,
                         obstacles,
                         preprocess,
                         "_BIT" + name,
                         print_info,
                         data_bundle)
        self.sol_grid = SolGrid(self.robots, self.obs, self.solution)
        self.boundaries = {"N": self.sol_grid.max_y,
                           "S": self.sol_grid.min_y,
                           "W": self.sol_grid.min_x,
                           "E": self.sol_grid.max_x}
        self.num_to_improve = data_bundle.get("num_to_improve", 1)
        if self.solution.out["algo_name"].find("BIT_BIT") != -1:
            self.solution.out["algo_name"] = self.solution.out["algo_name"][:(-1) * len("_BIT")]
            self.solution.out["extra"]["BIT_runs"] += 1
        else:
            self.solution.out["extra"]["BIT_runs"] = 1
        self.solution.out["extra"]["old_makespan"] = solution.out["makespan"]
        self.solution.out["extra"]["old_sum"] = solution.out["sum"]
        self.solution.out["extra"]["improved"] = ""
        self.sum = solution.out["sum"]

    def calc_new_path(self, robot_id, offset_from_last_step: int = 2, offset_to_start_before_goal: int = 110, calc_tries: int = 20):
        time_arrived = self.sol_grid.get_time_arrived()[robot_id]
        last_step_on_loc = self.sol_grid.find_last_step_on_location(robot_id, self.targets[robot_id], time_arrived)[0]
        goal_time = min(last_step_on_loc + offset_from_last_step, self.sol_grid.max_time)
        start_time = max(goal_time - offset_to_start_before_goal, 0)
        source_pos = self.sol_grid.get_robot_location(robot_id, start_time)
        source_pos_t = (source_pos[0], source_pos[1], start_time)
        dest_pos = self.targets[robot_id]
        dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)

        new_path = None
        counter = 0
        while new_path is None and counter < calc_tries:
            new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                          self.boundaries,
                                                          source_pos_t,
                                                          dest_pos_t,
                                                          robot_id,
                                                          self.sol_grid.max_time)
            offset_from_last_step = min(offset_from_last_step+10, time_arrived)
            goal_time = min(last_step_on_loc + offset_from_last_step, self.sol_grid.max_time)
            dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)
            counter += 1

        # assert new_path is not None, "BIT: a_star failed to find legal path"  # should at worst find old path
        if new_path is None:
            return None
        self.sol_grid.update_robot_path(robot_id, source_pos, new_path, start_time, time_arrived)
        self.update_solution(robot_id, new_path, start_time, time_arrived)
        return new_path

    def update_solution(self, robot_id, path: list, start_time: int, end_time: int):
        """

        :param end_time:
        :param robot_id:
        :param path:
        :param start_time:
        """
        counter = 0
        for step in range(start_time, end_time):
            if self.solution.out["steps"][step].pop(str(robot_id), None) is not None:
                self.sum -= 1
            if counter < len(path):
                if path[counter] != "X":
                    self.solution.out["steps"][step][str(robot_id)] = path[counter]
                    self.sum += 1
            counter += 1

    def run(self):
        # self.improve_last_moving()
        self.improve_all_by_arrival_order()
        self.improve_last_by_arrival_order(min(len(self.robots)//20, 5))
        self.solution.clean_solution()
        self.solution.put_result(SolutionResult.SUCCESS, len(self.solution.out["steps"]), self.sum)
        return self.solution

    def improve_last_moving(self):
        improved = []
        to_improve = []
        while len(improved) < self.num_to_improve:
            if len(to_improve) == 0:
                to_improve = self.sol_grid.get_last_moving_robots()
            next_robot_id = to_improve.pop()
            if next_robot_id in improved:
                break
            if self.calc_new_path(next_robot_id) is None:
                break
            improved.append(next_robot_id)

        self.solution.out["extra"]["improved"] += str(improved) + ","

    def improve_all_by_arrival_order(self):
        improved = []
        arrival_order = self.sol_grid.get_arrival_order()
        for robot_id in arrival_order:
            if self.calc_new_path(robot_id) is not None:
                improved.append(robot_id)
        self.solution.out["extra"]["improved"] += str(improved) + ","

    def improve_last_by_arrival_order(self, num: int = 10):
        improved = []
        arrival_order = self.sol_grid.get_arrival_order()
        for i in range(-num, 0):
            robot_id = arrival_order[i]
            if self.calc_new_path(robot_id) is not None:
                improved.append(robot_id)
        self.solution.out["extra"]["improved"] += str(improved) + ","
