from infrastructure.robot import Robot


class Preprocess:
    def __init__(self, data):
        self.data = data
        self.sorted_robots_by_x = []
        self.sorted_robots_by_y = []
        self.sorted_robots_by_x_than_by_y = []
        self.sorted_robots_by_y_than_by_x = []
        self.sorted_robots_by_target_x = []

    def sort_robots_by_x(self, robots: list) -> list:
        if len(self.sorted_robots_by_x) > 0:
            return self.sorted_robots_by_x

        x_vals = []
        for r in robots:
            x_vals.append((r.pos[0], r.robot_id))

        x_vals.sort(key=lambda x: x[0])
        for i in x_vals:
            self.sorted_robots_by_x.append(i[1])
        return self.sorted_robots_by_x

    def sort_robots_by_y(self, robots: list) -> list:
        if len(self.sorted_robots_by_y) > 0:
            return self.sorted_robots_by_y

        y_vals = []
        for r in robots:
            y_vals.append((r.pos[1], r.robot_id))

        y_vals.sort(key=lambda y: y[0])
        for i in y_vals:
            self.sorted_robots_by_y.append(i[1])
        return self.sorted_robots_by_y

    def sort_robots_by_x_than_by_y(self, robots: list) -> list:
        if len(self.sorted_robots_by_x_than_by_y) > 0:
            return self.sorted_robots_by_x_than_by_y

        pos_vals = []
        for r in robots:
            pos_vals.append((r.pos, r.robot_id))

        pos_vals.sort(key=lambda xy: xy[0])
        for i in pos_vals:
            self.sorted_robots_by_x_than_by_y.append(i[1])
        return self.sorted_robots_by_x_than_by_y

    def sort_robots_by_y_than_by_x(self, robots: list) -> list:
        if len(self.sorted_robots_by_y_than_by_x) > 0:
            return self.sorted_robots_by_y_than_by_x

        pos_vals = []
        for r in robots:
            pos_vals.append(((r.pos[1], r.pos[0]), r.robot_id))

        pos_vals.sort(key=lambda yx: yx[0])
        for i in pos_vals:
            self.sorted_robots_by_y_than_by_x.append(i[1])
        return self.sorted_robots_by_y_than_by_x

    def sort_robots_by_target_x(self, robots: list) -> list:
        if len(self.sorted_robots_by_target_x) > 0:
            return self.sorted_robots_by_target_x

        x_vals = []
        for r in robots:
            x_vals.append((r.target_pos[0], r.robot_id))

        x_vals.sort(key=lambda x: x[0])
        for i in x_vals:
            self.sorted_robots_by_target_x.append(i[1])
        return self.sorted_robots_by_target_x