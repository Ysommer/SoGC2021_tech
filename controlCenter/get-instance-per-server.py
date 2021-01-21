import sys

sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")
from WishList import *

out = {}
packages = InstancesPackage.get_instances_packages()
num_of_servers = 11
# package_id = int(sys.argv[1])

for package_id in range(num_of_servers):
    out[package_id] = {}
    for p in packages:
        chunk_size = len(packages[p]) / num_of_servers
        out[package_id][p] = packages[p][round(chunk_size * package_id):round(chunk_size * (package_id + 1))]

for x in out:
    print(x)
    for y in out[x]:
        print('\t', y, ':', out[x][y])
