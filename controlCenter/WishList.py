import functools
print = functools.partial(print, flush=True)

from enum import Enum
from typing import List
from algos.InitShell import *
from algos.init_algos.OutAndInByPercentage import *
from algos.optimization_algos.BFS_in_time import *
from algos.OptimizationShell import *
from cgshop2021_pyutils import Instance
from controlCenter import ControlCenter
from Utils.loadInstances import *

class WishListPackagesTypes(Enum):
    TINY = 0
    SMALL = 1
    MEDIUM = 2
    MEDIUM_LARGE = 3
    LARGE = 4
    HUGE = 5


class PackagesFunctionsByType:
    @staticmethod
    def get_function(package_type: WishListPackagesTypes):
        functions = {
            WishListPackagesTypes.TINY.name: PackagesFunctionsByType.run_tiny,
            WishListPackagesTypes.SMALL.name: PackagesFunctionsByType.run_small
        }
        return functions[package_type.name]

    @staticmethod
    def init_control_center(instance: Instance) -> ControlCenter:
        print("====================")
        print("Start instance: ", instance.name, "(number:" + str(id) + ")")
        out_path = "../solutions/" + instance.name + "/"
        num_of_robots = instance.number_of_robots
        max_makespan = -1
        max_sum = -1
        return ControlCenter(instance, out_path, max_makespan, max_sum, print_init_sol=True)

    @staticmethod
    def run_tiny(instance: Instance, initShells: List[InitShell]=None, optShells: List[InitShell]=None) -> (int, int):
        control_center = PackagesFunctionsByType.init_control_center(instance)

        if initShells:
            for i in initShells:
                control_center.add_init_algo(i)
        else:
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": ""})
            for i in range(0, 5):
                control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                             data_bundle={"sync_insertion": False, "secondary_order": "rand"})

        if optShells:
            for i in optShells:
                control_center.add_opt_algo(i)
        else:
            control_center.add_opt_algo(BFS_in_time, data_bundle={})

        control_center.run_all(print_only_success=True, stop_on_success=False, validate=False)
        return (control_center.min_makespan, control_center.min_sum)

    @staticmethod
    def run_small(instance: Instance, initShells: List[InitShell]=None, optShells: List[InitShell]=None):
        control_center = PackagesFunctionsByType.init_control_center(instance)

        if initShells:
            for i in initShells:
                control_center.add_init_algo(i)
        else:
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_grid",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_from_target",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS"})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": "dist_BFS",
                                                      "descending_order": True})
            control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                         data_bundle={"sync_insertion": False, "secondary_order": ""})
            for i in range(0, 25):
                control_center.add_init_algo(OutAndInByPercentage, print_info=False,
                                             data_bundle={"sync_insertion": False, "secondary_order": "rand"})

        if optShells:
            for i in optShells:
                control_center.add_opt_algo(i)
        else:
            control_center.add_opt_algo(BFS_in_time, data_bundle={})

        control_center.run_all(print_only_success=True, stop_on_success=False, validate=False)
        return (control_center.min_makespan, control_center.min_sum)


class InstancesPackage:
    def __init__(self, package_type: WishListPackagesTypes, package_id, num_of_servers: int = 16, instances_packages=None, hard_coded_instances = None):
        self.instances_package = instances_packages
        if self.instances_package is None:
            self.instances_package = InstancesPackage.get_instances_packages()

        self.package_type = package_type
        self.results = []
        self.instances_id = hard_coded_instances

        if self.instances_id is None:
            chunk_size = len(self.instances_package[self.package_type.name])/num_of_servers
            self.instances_id = self.instances_package[self.package_type.name][round(chunk_size * package_id):round(chunk_size * (package_id + 1))]
            # TODO check if any instance is in exactly one chunk

        all_instances = load_all_instances()
        self.instances = []
        for id in self.instances_id:
            self.instances.append(all_instances[id])

    def run(self, initShells: List[InitShell]=None, optShells: List[InitShell]=None):
        for instance in self.instances:
            self.results.append(PackagesFunctionsByType.get_function(self.package_type)(instance, initShells, optShells))

    def print_results(self):
        for i in range(len(self.results)):
            id = self.instances_id[i]
            instance = self.instances[i]
            print("instance: ", instance.name, " (" + str(id) + ") ended with a minimal makespan of :", self.results[i][0], " and a minimal sum of :", self.results[i][1])


    @staticmethod
    def get_instances_packages():

        instances_packages = {}

        for i in WishListPackagesTypes:
            instances_packages[i.name] = []

        specials = {
            30: WishListPackagesTypes.SMALL.name,
            181: WishListPackagesTypes.SMALL.name,
            192: WishListPackagesTypes.SMALL.name
        }

        instances_table = open("TableOfContents.txt", "r")
        next_instance = instances_table.readline()

        while len(next_instance) > 0:
            next_instance_index = int(next_instance.split(' ')[0])
            next_instance_name = next_instance.split(' ')[2]
            if next_instance_index in specials:
                instances_packages[specials[next_instance_index]].append(next_instance_index)
                next_instance = instances_table.readline()
                continue

            # Find size - currently not in use
            first_size_index = next_instance_name.rfind('x') + 1
            last_size_index = first_size_index + (next_instance_name[first_size_index:]).find('_')
            size = int(next_instance_name[first_size_index:last_size_index])

            # Find num of robots
            robots = int(next_instance_name.split('_')[-1])


            if robots <= 100:
                instances_packages[WishListPackagesTypes.TINY.name].append(next_instance_index)
            elif robots <= 250:
                instances_packages[WishListPackagesTypes.SMALL.name].append(next_instance_index)
            elif robots <= 1000:
                instances_packages[WishListPackagesTypes.MEDIUM.name].append(next_instance_index)
            elif robots <= 2000:
                instances_packages[WishListPackagesTypes.MEDIUM_LARGE.name].append(next_instance_index)
            elif robots <= 5000:
                instances_packages[WishListPackagesTypes.LARGE.name].append(next_instance_index)
            else:
                instances_packages[WishListPackagesTypes.HUGE.name].append(next_instance_index)

            next_instance = instances_table.readline()

        instances_table.close()
        return instances_packages


class WishList:
    def __init__(self):
        self.initShells = None
        self.optShells = None

    def optimize_solutions(self, solutionsToOptimizePath: str, number_of_iterations: int = 1):
        pass

    def all_to_all(self, number_of_iterations: int = 1):
        """
        run all init algorithms
        then run number_of_iterations of optimizations
        :return:
        """
        pass

    @staticmethod
    def farm_instances(package_type: WishListPackagesTypes, package_id, num_of_servers: int = 16):
        package = InstancesPackage(package_type, package_id, num_of_servers)
        print("Start farming", str(len(package.instances_id)), package_type.name, "instances.")
        print(package.instances_id)

        package.run()
        package.print_results()

    def zip_and_verify(self):
        pass

