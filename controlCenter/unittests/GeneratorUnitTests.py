from dataCollection.Generator import Generator
from unittests.basic_tests import BasicTests
from infrastructure.grid import Grid


class GeneratorUnitTests(BasicTests):
    def __init__(self):
        self.sizes = [5, 10, 20, 50]
        self.robots_lists = [[] for i in range(len(self.sizes))]
        self.obs_lists = [[] for i in range(len(self.sizes))]
        self.grids = []
        for i in range(len(self.sizes)):
            self.grids.append(Grid(
                size=self.sizes[i],
                robots=self.robots_lists[i],
                obstacles=self.obs_lists[i]
            ))

    def run(self):
        print(self.__class__.__name__, "start:")
        self.test_calc_bfs_map_no_obs()
        self.test_calc_bfs_map_with_obs_no_robots()

        print(self.__class__.__name__, "passed!")

    def test_calc_bfs_map_no_obs(self):
        self.obs_lists = [[] for i in range(len(self.sizes))]

        sources = [
            [(0, 0)],
            [(5, 0)],
            [(0, 5)],
            [(0, -1)],
            [(0, 0), (4, 0), (0, 4), (4, 4)],
            [(i, i) for i in range(4)]
        ]

        for s in sources:
            for i in range(len(self.sizes)):
                grid = self.grids[i]
                Generator.calc_bfs_map(
                    sources=s,
                    grid=grid,
                    boundaries=self.__get_standard_boundaries(self.sizes[i]))
                for x in range(0, self.sizes[i]):
                    for y in range(0, self.sizes[i]):
                        x_dist = abs(x-s[0][0])
                        y_dist = abs(y-s[0][1])
                        dist = x_dist + y_dist

                        res_dist = grid.get_cell_distance((x, y))
                        for pos in s:
                            x_dist_temp = abs(x - pos[0])
                            y_dist_temp = abs(y - pos[1])
                            if x_dist_temp+y_dist_temp < dist:
                                x_dist = x_dist_temp
                                y_dist = y_dist_temp
                                dist = x_dist + y_dist

                        if dist != res_dist:
                            self.__print_failure(self.__class__.test_calc_bfs_map_no_obs.__name__,
                                                 "dist != res_dist",
                                                 {
                                                     "source": s,
                                                     "pos": pos,
                                                     "x,y": (x,y),
                                                     "dist": dist,
                                                     "res_dist": res_dist
                                                 })
                            assert 0

        print("\t", self.__class__.test_calc_bfs_map_no_obs.__name__, "passed")

    def test_calc_bfs_map_with_obs_no_robots(self):
        size = 50
        obs_list = [[x, 10] for x in range(-5, size+5)]
        sources = [
            [(0, 0)],
            [(5, 0)],
            [(5, 5)],
            [(0, 5)],
            [(-1, 5)],
            [(5, -1)],
            [(0, -1)]
        ]

        for s in sources:
            grid = Grid(size, [], obs_list)
            Generator.calc_bfs_map(
                sources=s,
                grid=grid,
                boundaries=self.__get_standard_boundaries(size))
            for x in range(0, size):
                for y in range(0, size):
                    if s[0][1] < 10 < y or y < 10 < s[0][1]:
                        assert grid.get_cell_distance((x, y)) == -1
                        continue

                    if y == 10:
                        assert grid.get_cell_distance((x, y)) == -1
                        continue


                    x_dist = abs(x-s[0][0])
                    y_dist = abs(y-s[0][1])
                    dist = x_dist + y_dist
                    res_dist = grid.get_cell_distance((x, y))

                    for pos in s:
                        x_dist_temp = abs(x - pos[0])
                        y_dist_temp = abs(y - pos[1])
                        if x_dist_temp+y_dist_temp < dist:
                            x_dist = abs(x - pos[0])
                            y_dist = abs(y - pos[1])
                            dist = x_dist + y_dist

                    if dist != res_dist:
                        self.__print_failure(self.__class__.test_calc_bfs_map_no_obs.__name__,
                                             "dist != res_dist",
                                             {
                                                 "source": s,
                                                 "pos": pos,
                                                 "x,y": (x,y),
                                                 "dist": dist,
                                                 "res_dist": res_dist
                                             })
                        assert 0

        print("\t", self.__class__.test_calc_bfs_map_with_obs_no_robots.__name__, "passed")

    @staticmethod
    def __print_failure(function, cause, data):
        print("Failure in", function,":", cause)
        for key in data.keys():
            print(key, ":", data[key])

    @staticmethod
    def __get_standard_boundaries(size) -> dict:
        return {
            "N": size-1,
            "E": size-1,
            "S": 0,
            "W": 0
        }