import time


def sum_tuples(first_tuple: tuple, second_tuple: tuple):
    return (first_tuple[0] + second_tuple[0], first_tuple[1] + second_tuple[1])

def sum_tuples_with_time(first_tuple: tuple, second_tuple: tuple):
    return (first_tuple[0] + second_tuple[0], first_tuple[1] + second_tuple[1], first_tuple[2] + second_tuple[2])

def sub_tuples(first_tuple: tuple, second_tuple: tuple):
    return (first_tuple[0] - second_tuple[0], first_tuple[1] - second_tuple[1])

def sub_tuples_with_time(first_tuple: tuple, second_tuple: tuple):
    return (first_tuple[0] - second_tuple[0], first_tuple[1] - second_tuple[1], first_tuple[2] - second_tuple[2])

class Timer:
    def __init__(self, label="Time elapsed:"):
        self.label = label
        self.__start_time = None
        self.__end_time = None

    def __str__(self):
        return self.label + ": " + str(self.get_timed()) + " seconds"

    def __repr__(self):
        return str(self)

    def start(self):
        """
        keep only the first "start"
        """
        if self.__start_time is None:
            self.__start_time = time.time()

    def end(self, to_print=False):
        """
        keep the last call for end
        :param to_print:
        """
        if self.__start_time is None:
            print("Error using timer - get_timed")
            return
        self.__end_time = time.time()
        if to_print:
            print(self)

    def get_timed(self):
        if self.__end_time is None:
            print("Error using timer - get_timed")
            return
        return (int(100 * (self.__end_time-self.__start_time))) / 100
