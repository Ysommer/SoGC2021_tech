from infrastructure.robot import Robot


class Preprocess:
    def __init__(self, data, robots: list):
        self.robots = robots
        self.data = data
        self.sorted_R_x = []
        self.sorted_R_y = []
        self.sorted_R_x_y = []
        self.sorted_R_x_Y = []
        self.sorted_R_y_x = []
        self.sorted_R_y_X = []
        self.sorted_robots_by_y = []
        self.sorted_robots_by_x_than_by_y = []
        self.sorted_robots_by_y_than_by_x = []
        self.sorted_robots_by_target_x = []
        self.arrays_dict = {
            'R_x': self.sorted_R_x,
            'R_y': self.sorted_R_y
        }

    # Sort by robot's x
    def sort_R_x(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_x

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_x", robots)

        return res.copy()

    def sort_R_X(self, robots: list = None):
        res = self.sort_R_x(robots)
        res.reverse()
        return res

    def sort_R_x_y(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_x_y

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_x_y", robots)

        return res.copy()

    def sort_R_X_y(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_x_Y

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_x_Y", robots)

        res = res.copy()
        res.reverse()
        return res

    def sort_R_x_Y(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_x_Y

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_x_Y", robots)

        return res.copy()

    def sort_R_X_Y(self, robots: list = None):
        res = self.sort_R_x_y(robots)
        res.reverse()
        return res

    # Sort by robot's Y
    def sort_R_y(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_y

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_y", robots)

        return res.copy()

    def sort_R_Y(self, robots: list = None):
        res = self.sort_R_y(robots)
        res.reverse()
        return res

    def sort_R_y_x(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_y_x

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_y_x", robots)

        return res.copy()

    def sort_R_y_X(self, robots: list = None):
        res = []
        if robots is None:
            res = self.sorted_R_y_X

        if len(res) == 0:
            self.__generic_robots_sort(res, "R_y_X", robots)

        return res.copy()

    def sort_R_Y_x(self, robots: list = None):
        res = self.sort_R_y_X(robots)
        res.reverse()
        return res

    def sort_R_Y_X(self, robots: list = None):
        res = self.sort_R_y_x(robots)
        res.reverse()
        return res

    def __generic_robots_sort(self, res, code: str, robots: list = None):
        # A list of tuples ( data_to_sort_by , robot_id)
        data_to_sort_by_index = 0
        robot_id_index = 1
        data_to_sort_by = []
        for r in self.robots:
            data_to_sort_by.append((self.__sort_code_to_value(self, code, r), r.robot_id))

        # sort
        data_to_sort_by.sort(key=lambda x: x[data_to_sort_by_index])

        # sort robot id's
        for i in data_to_sort_by:
            res.append(i[robot_id_index])

    @staticmethod
    def __sort_code_to_value(self, code: str, robot: Robot):
        X = 0
        Y = 1

        # Sort by robots
        if code == "R_x":
            return robot.pos[X]
        if code == "R_y":
            return robot.pos[Y]

        if code == "R_x_y":
            return robot.pos
        if code == "R_y_x":
            return (robot.pos[Y], robot.pos[X])

        if code == "R_x_Y":
            return (robot.pos[X], (-1)*robot.pos[Y])
        if code == "R_y_X":
            return (robot.pos[Y], (-1)*robot.pos[X])


        # Sort by Targets
        if code == "T_x":
            return robot.target_pos[0]
        if code == "T_y":
            return robot.target_pos[1]

        return robot.extra_data
