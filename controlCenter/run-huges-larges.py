import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")
import os
import subprocess
from WishList import WishList, WishListPackagesTypes

servers_ips = {
    "132.69.8.13": 0,
    "132.69.8.8": 1,
    "132.69.8.10": 2,
    "132.69.8.11": 3,
    "10.10.43.32": 4,
    "10.10.43.26": 5,
    "10.10.43.27": 6,
    "10.10.43.29": 7,
    "10.10.43.14": 8,
    "10.10.43.30": 9,
    "10.10.43.31": 10
}

large_lists = [
    [66, 58, 39],
    [78, 26, 27],
    [120, 137, 57],
    [100, 190, 118],
    [200, 28, 19],
    [119, 56, 199],
    [77, 128, 6],
    [201, 64, 8, 140],
    [55, 139, 189, 49],
    [127, 48, 65, 38],
    [188, 47, 7, 138]
]

huge_lists = [
    [80, 130],
    [68, 79],
    [67, 129],
    [59, 191],
    [69, 202],
    [9],
    [29],
    [40],
    [50],
    [60],
    [70]
]

try:
    cmd = "ip route get 1 | awk '{print $(NF-1);exit}'"
    result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

    ip = result.stdout.readlines()[0].decode('UTF-8')[:-1]
    server_id = servers_ips[ip]
except:
    cmd = "ip route get 1 | awk '{print $(NF-2);exit}'"
    result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

    ip = result.stdout.readlines()[0].decode('UTF-8')[:-1]
    server_id = servers_ips[ip]


print("server_id", server_id)
large_list = large_lists[server_id]
huge_list = huge_lists[server_id]
print("large list:", large_list)
print("huge list:", huge_list)


try:
    os.mkdir("../out_files/")
except:
    pass

for instance_id in large_list:
    cmd = "python3.7 farm-instance.py " + \
          str(instance_id) + " 1 6 > ../out_files/Large" + str(instance_id)+".txt 2> ../out_files/Large"+ str(instance_id) + "_err.txt &"
    print("cmd", cmd)
    os.system(cmd)

for instance_id in huge_list:
    in_parallel = 6 // len(huge_list)
    in_queue = len(huge_list)
    cmd = "python3.7 farm-instance.py " + \
          str(instance_id) + " " + str(in_parallel) + " " + str(in_queue) + " > ../out_files/Huge"+ str(instance_id)+".txt 2> ../out_files/Huge"+ str(instance_id) + "_err.txt &"

    print("instance_id", instance_id)
    print("in_parallel", in_parallel)
    print("in_queue", in_queue)
    print("cmd", cmd)

    os.system(cmd)

cmd = "disown -a"
print("cmd", cmd)
os.system(cmd)
