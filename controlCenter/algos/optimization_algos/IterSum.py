import functools

print = functools.partial(print, flush=True)

from algos.optimizationAlgo import OptimizationAlgo
from solution.solution import Solution
from dataCollection.Generator import *
from defines import *
import time as ttt
from copy import copy


class IterSum(OptimizationAlgo):
    def __init__(self, instance_name: str,
                 solution: Solution,
                 robots: list,
                 targets: list,
                 obstacles: list,
                 size,
                 preprocess=None,
                 name="",
                 print_info=True,
                 data_bundle=None):
        super().__init__(instance_name,
                         solution,
                         robots,
                         targets,
                         obstacles,
                         size,
                         preprocess,
                         "_IterSum" + name,
                         print_info,
                         data_bundle)

        if data_bundle is None:
            data_bundle = {}
        self.max_jump = data_bundle.get("max_jump", 1)

        if self.solution.out["algo_name"].find("IterSum__IterSum") != -1:
            self.solution.out["algo_name"] = self.solution.out["algo_name"][:(-1) * len("__IterSum")]

        self.robots_pos = [self.robots[i].pos for i in range(len(self.robots))]
        self.grid = {self.robots_pos[i]: i for i in range(len(self.robots_pos))}
        self.blocked_grid = {}
        for obs in obstacles:
            self.grid[tuple(obs)] = -1
            self.blocked_grid[tuple(obs)] = -1

        self.last_step_on_target = self.solution.out["extra"].get("last_step_on_target", None)
        self.sum_per_robot = self.solution.out["extra"].get("sum_per_robot", None)
        self.time_arrived = self.solution.out["extra"].get("time_arrived", None)
        self.arrival_order = solution.out["extra"].get("arrival_order", None)
        self.boundaries = solution.out["extra"].get("boundaries", None)
        if self.last_step_on_target is None or self.sum_per_robot is None or self.time_arrived is None or \
                self.arrival_order is None or self.boundaries is None:
            self.last_step_on_target, self.sum_per_robot, self.time_arrived, self.arrival_order, self.boundaries = self.calc_extra_stuff()

        self.target_dict = {tuple(targets[i]): i for i in range(len(targets))}

        self.waiting_for_improvement = sorted([i for i in range(len(self.robots))], key=lambda x: self.last_step_on_target[x], reverse=True)
        self.to_improve = set()
        self.improved = []
        self.arrival_order_index = 0

        self.time = 0
        self.offset = 0
        self.original_makespan = solution.out["makespan"]

        self.future_sum_per_robot = [0 for i in range(len(self.robots))]
        self.future_last_step_grid = {}
        self.future_time_arrived = [0 for i in range(len(self.robots))]

    # calcs last_step_grid & sum_list & time_arrived & arrival_order
    def calc_extra_stuff(self) -> tuple:
        last_step_grid = {}
        sum_list = [0 for i in range(len(self.robots))]
        time_arrived = [0 for i in range(len(self.robots))]
        boundaries = {"E": 10, "W": 0, "N": 10, "S": 0}
        robot_pos = self.robots_pos.copy()
        t = 1
        for step in self.solution.out["steps"]:
            for robot_id, d in step.items():
                robot_id = int(robot_id)
                old_pos = robot_pos[robot_id]
                new_pos = sum_tuples(old_pos, directions_to_coords[d])
                robot_pos[robot_id] = new_pos
                sum_list[robot_id] += 1
                if old_pos != tuple(self.targets[robot_id]):
                    last_step_grid[old_pos] = t - 1
                time_arrived[robot_id] = t
                boundaries["E"] = max(boundaries["E"], new_pos[0])
                boundaries["W"] = min(boundaries["W"], new_pos[0])
                boundaries["N"] = max(boundaries["N"], new_pos[1])
                boundaries["S"] = min(boundaries["S"], new_pos[1])
            t += 1

        arrival_order = sorted(range(len(self.time_arrived)), key=lambda x: self.time_arrived[x])
        last_step_on_target = [last_step_grid.get(tuple(self.targets[i]), -1) for i in range(len(self.robots))]
        boundaries["E"] += 1
        boundaries["W"] -= 1
        boundaries["N"] += 1
        boundaries["S"] -= 1
        return last_step_on_target, sum_list, time_arrived, arrival_order, boundaries

    def step_grid(self):
        t = self.time + self.offset
        robot_reached_target = False
        step = self.solution.out["steps"][t]
        for robot_id, direction in step.items():
            robot_id = int(robot_id)
            old_pos = self.robots_pos[robot_id]
            new_pos = sum_tuples(old_pos, directions_to_coords[direction])
            assert self.grid[old_pos] == robot_id
            self.grid.pop(old_pos)
            self.grid[new_pos] = robot_id
            self.robots_pos[robot_id] = new_pos
            self.sum_per_robot[robot_id] -= 1

            self.future_sum_per_robot[robot_id] += 1
            if old_pos != tuple(self.targets[robot_id]):
                self.future_last_step_grid[old_pos] = t
            self.future_time_arrived[robot_id] = t + 1

            if new_pos == self.targets[robot_id] and self.time - 1 <= self.time_arrived[robot_id] <= self.time + 1:
                self.blocked_grid[self.targets[robot_id]] = robot_id
                robot_reached_target = True

        return robot_reached_target

    def update_solution(self, robot_id, path):
        t = self.time + self.offset
        to_insert = []
        for direction in path:
            to_insert.append({str(robot_id): direction})
        self.solution.out["steps"][t:t] = to_insert
        self.offset += len(path)
        for i in range(t + len(path), min(self.time_arrived[robot_id] + self.offset + 1, len(self.solution.out["steps"]))):
            self.solution.out["steps"][i].pop(str(robot_id), None)
        self.future_sum_per_robot[robot_id] += len(path)
        self.future_time_arrived[robot_id] = t + len(path)
        pos = self.targets[robot_id]
        t += len(path) - 1
        for d in path[::-1]:
            pos = sub_tuples(pos, directions_to_coords[d])
            self.future_last_step_grid[pos] = t
            t -= 1

    def move_robot_to_target(self, robot_id):
        self.grid.pop(self.robots_pos[robot_id])
        self.grid[tuple(self.targets[robot_id])] = robot_id
        self.robots_pos[robot_id] = self.targets[robot_id]

    def choose_and_improve(self) -> dict:
        to_improve = list(self.to_improve.copy())
        improved = {}  # robot_id: path
        while len(to_improve) > 0:
            can_improve = {}  # robot_id: path
            blocked = set()  # robot_id of blocked targets
            astar_result = []  # (robot_id, blocking_targets)
            for robot_id in to_improve:
                path, blocking_targets = Generator.calc_a_star_path_itersum(self.grid,
                                                                            self.boundaries,
                                                                            robot_id,
                                                                            self.robots_pos[robot_id],
                                                                            self.targets[robot_id],
                                                                            self.sum_per_robot[robot_id] - 1,
                                                                            self.target_dict)
                if path is not None and len(path) < self.sum_per_robot[robot_id]:
                    can_improve[robot_id] = path
                    astar_result.append((robot_id, blocking_targets))
            astar_result.sort(reverse=True, key=lambda x: len(x[1]))
            i = 0
            improve = True
            if len(can_improve) == 0:
                return improved
            while len(can_improve) > 0:
                r_id, blocking_targets = astar_result[i]
                for robot_id in blocking_targets:
                    if robot_id in blocked:
                        improve = False
                        break
                if not improve:
                    improve = True
                else:
                    improved[r_id] = can_improve[r_id]
                    blocked.add(r_id)
                    to_improve.remove(r_id)
                    self.move_robot_to_target(r_id)
                can_improve.pop(r_id)
                i += 1
        return improved

    def work(self):
        time_jump = 1
        last_print_index = -1
        while self.time < self.original_makespan:
            # add robots who's target's last step passed
            while len(self.waiting_for_improvement) > 0 and self.last_step_on_target[self.waiting_for_improvement[-1]] <= self.time-1:
                self.to_improve.add(self.waiting_for_improvement.pop())
            # remove robots who arrived before current time
            while self.arrival_order_index < len(self.arrival_order) and self.time_arrived[self.arrival_order[self.arrival_order_index]] <= self.time:
                self.to_improve.discard(self.arrival_order[self.arrival_order_index])
                self.arrival_order_index += 1
            # improve robots - func returns list of (robot_id, path)
            improved = self.choose_and_improve()
            # if len(improved) > 0:
            #    print("num_improved:", len(improved), "out of", len(self.to_improve), "time is", self.time)
            for robot_id, path in improved.items():
                self.to_improve.remove(robot_id)
                self.improved.append(robot_id)
                self.update_solution(robot_id, path)
            if len(improved) == 0 and time_jump < 32 and self.time > 10:
                time_jump *= 2
            else:
                time_jump = max(1, time_jump//4)

            if self.time // 1000 > last_print_index:
                last_print_index += 1
                percent = int((self.time / self.original_makespan) * 100)
                print(self.time, "makespan done,", self.original_makespan - self.time, "remaining (", percent, "% )", ttt.strftime("%d/%m/%Y-%H:%M:%S"))

            # update grid to next step
            if self.time + time_jump >= self.original_makespan:
                break

            clean = False
            for i in range(time_jump):
                if self.step_grid():
                    clean = True
                self.time += 1
            if clean and self.time > 50:
                to_remove = []
                for r in self.to_improve:
                    if Generator.calc_a_star_path_itersum(self.grid,
                                                          self.boundaries,
                                                          r,
                                                          self.robots_pos[r],
                                                          self.targets[r],
                                                          self.sum_per_robot[r]-1,
                                                          self.target_dict)[0] is None:
                        to_remove.append(r)
                for r in to_remove:
                    self.to_improve.remove(r)

        print("all done! ( 100 % )", ttt.strftime("%d/%m/%Y-%H:%M:%S"))


    def run(self):
        self.work()
        self.solution.clean_solution()
        mysum = sum(x for x in self.future_sum_per_robot)
        self.solution.put_result(SolutionResult.SUCCESS, len(self.solution.out["steps"]), mysum)
        self.solution.out["extra"]["sum_per_robot"] = self.future_sum_per_robot
        self.solution.out["extra"]["last_step_on_target"] = [self.future_last_step_grid.get(self.targets[i], -1) for i in range(len(self.robots))]
        self.solution.out["extra"]["time_arrived"] = self.future_time_arrived
        arrival_order = sorted(range(len(self.future_time_arrived)), key=lambda x: self.time_arrived[x])
        self.solution.out["extra"]["arrival_order"] = arrival_order
        return self.solution
