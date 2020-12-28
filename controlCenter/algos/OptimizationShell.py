
class OptimizationShell:
    def __init__(self, algo: classmethod, name="_", print_info=True, data_bundle=None ):
        self.algo_class = algo
        self.algo_name = name
        self.print_info = print_info
        self.data_bundle = data_bundle
