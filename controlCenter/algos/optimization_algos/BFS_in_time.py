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
        self.max_grid_len = data_bundle.get("grid_limit", 1000)
        if self.max_grid_len != -1:
            self.solution.out["steps"] = self.solution.out["steps"][:self.max_grid_len]
        self.sol_grid = SolGrid(self.robots, self.obs, self.solution, max_grid_len=self.max_grid_len)
        self.boundaries = {"N": self.sol_grid.max_y,
                           "S": self.sol_grid.min_y,
                           "W": self.sol_grid.min_x,
                           "E": self.sol_grid.max_x}
        self.num_to_improve = data_bundle.get("num_to_improve", 1)
        self.goal_raise = data_bundle.get("gaol_raise", 32)
        if self.solution.out["algo_name"].find("BIT_BIT") != -1:
            self.solution.out["algo_name"] = self.solution.out["algo_name"][:(-1) * len("_BIT")]
            self.solution.out["extra"]["BIT_runs"] += 1
        else:
            self.solution.out["extra"]["BIT_runs"] = 1
        self.solution.out["extra"]["old_makespan"] = solution.out["makespan"]
        self.solution.out["extra"]["old_sum"] = solution.out["sum"]
        self.solution.out["extra"]["improved"] = ""
        self.sum = solution.out["sum"]

    def calc_new_path(self, robot_id, offset_from_last_step: int = 1, offset_to_start_before_goal: int = 110,
                      calc_tries: int = 20, goal_time_raise: int = 10):
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
        while new_path is None and counter < calc_tries and goal_time < time_arrived:
            new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                          self.boundaries,
                                                          source_pos_t,
                                                          dest_pos_t,
                                                          robot_id,
                                                          self.sol_grid.max_time,
                                                          last_step_on_loc)
            if counter > 0:
                offset_from_last_step += goal_time_raise
            elif offset_from_last_step == 1:
                offset_from_last_step += 1
            goal_time = min(last_step_on_loc + offset_from_last_step, time_arrived)
            dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)
            counter += 1

        if counter == 1:
            print("ran 1")
        elif counter == 2:
            print("ran 2")
        if new_path is None:
            return None
        self.sol_grid.update_robot_path(robot_id, source_pos, new_path, start_time, time_arrived)
        self.update_solution(robot_id, new_path, start_time, time_arrived)
        return new_path

    def calc_new_path_bs(self, robot_id, offset_to_start_before_goal: int = 110,
                         calc_tries: int = 20, goal_time_raise: int = 8,
                         skip_last_plus_one: bool = False):
        time_arrived = self.sol_grid.get_time_arrived()[robot_id]
        last_step_on_loc = self.sol_grid.find_last_step_on_location(robot_id, self.targets[robot_id], time_arrived)[0]
        goal_time = min(last_step_on_loc + 1, self.sol_grid.max_time)
        start_time = max(goal_time - offset_to_start_before_goal, 0)
        source_pos = self.sol_grid.get_robot_location(robot_id, start_time)
        source_pos_t = (source_pos[0], source_pos[1], start_time)
        dest_pos = self.targets[robot_id]
        dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)

        # first try 1 and 2 steps from last step on target
        counter = 0
        if not skip_last_plus_one:
            new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                          self.boundaries,
                                                          source_pos_t,
                                                          dest_pos_t,
                                                          robot_id,
                                                          self.sol_grid.max_time,
                                                          last_step_on_loc)
            if new_path is not None:
                self.sol_grid.update_robot_path(robot_id, source_pos, new_path, start_time, time_arrived)
                self.update_solution(robot_id, new_path, start_time, time_arrived)
                return new_path
            counter += 1
        goal_time = min(last_step_on_loc + 2, time_arrived)
        dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)
        new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                      self.boundaries,
                                                      source_pos_t,
                                                      dest_pos_t,
                                                      robot_id,
                                                      self.sol_grid.max_time,
                                                      last_step_on_loc)
        if new_path is not None:
            self.sol_grid.update_robot_path(robot_id, source_pos, new_path, start_time, time_arrived)
            self.update_solution(robot_id, new_path, start_time, time_arrived)
            return new_path
        counter += 1

        # find first successful astar run
        while new_path is None and counter < calc_tries and goal_time < time_arrived:
            new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                          self.boundaries,
                                                          source_pos_t,
                                                          dest_pos_t,
                                                          robot_id,
                                                          self.sol_grid.max_time,
                                                          last_step_on_loc)
            goal_time = min(goal_time + goal_time_raise, time_arrived)
            dest_pos_t = (dest_pos[0], dest_pos[1], goal_time)
            counter += 1
        if new_path is None:
            return None
        low = max(goal_time - 2*goal_time_raise + 1, 1)
        high = start_time + len(new_path)
        other_new_path = None
        other = True  # True when last successful run was in new_path

        while high > low:
            mid = (high + low) // 2
            dest_pos_t = (dest_pos[0], dest_pos[1], mid)
            if other:
                other_new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                                    self.boundaries,
                                                                    source_pos_t,
                                                                    dest_pos_t,
                                                                    robot_id,
                                                                    self.sol_grid.max_time,
                                                                    last_step_on_loc)
                if other_new_path is None:
                    low = mid + 1
                else:
                    other = False
                    high = mid
            else:
                new_path = Generator.calc_a_star_path_in_time(self.sol_grid,
                                                              self.boundaries,
                                                              source_pos_t,
                                                              dest_pos_t,
                                                              robot_id,
                                                              self.sol_grid.max_time,
                                                              last_step_on_loc)
                if new_path is None:
                    low = mid + 1
                else:
                    other = True
                    high = mid

        if not other and other_new_path is not None:
            new_path = other_new_path
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
        # self.improve_last_by_arrival_order(min(len(self.robots)//20, 5))
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
        robots_remaining = len(self.robots)
        goal_raise = self.goal_raise
        offset_from_last_step = 1
        calc_tries = 300
        i = 0
        for robot_id in arrival_order:
            if robots_remaining == -1:
                offset_from_last_step = 1
                goal_raise = 1
            if self.calc_new_path_bs(robot_id,
                                     goal_time_raise=goal_raise,
                                     calc_tries=calc_tries) is not None:
                improved.append(robot_id)
                i += 1
            robots_remaining -= 1
        print("improved:", i)
        self.solution.out["extra"]["improved"] += str(improved) + ","
        self.solution.out["extra"]["goal_raise_for_bs"] = goal_raise

    def improve_last_by_arrival_order(self, num: int = 10):
        improved = []
        arrival_order = self.sol_grid.get_arrival_order()
        for i in range(-num, 0):
            robot_id = arrival_order[i]
            if self.calc_new_path(robot_id) is not None:
                improved.append(robot_id)
        self.solution.out["extra"]["improved"] += str(improved) + ","
