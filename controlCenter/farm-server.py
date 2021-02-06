import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")
import os
import subprocess
from subprocess import Popen
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
    "10.10.43.31": 10,
    "10.10.43.38": 11,
    "10.10.43.41": 12,
    "10.10.43.42": 13,
    "10.10.43.43": 14,
    "10.10.43.44": 15,
    "10.10.43.45": 16
}

server_id = -1
if len(sys.argv) > 1:
    server_id = sys.argv[1]

if server_id == -1:
    cmd = "ip route get 1 | awk '{print $(NF-2);exit}'"
    result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

    ip = result.stdout.readlines()[0].decode('UTF-8')[:-1]
    print("ip", ip)
    server_id = servers_ips[ip]

types_to_farm = [WishListPackagesTypes.TINY, WishListPackagesTypes.SMALL, WishListPackagesTypes.MEDIUM, WishListPackagesTypes.MEDIUM_LARGE]

try:
    os.mkdir("../out_files/")
except:
    pass

for packageType in types_to_farm:
    cmd = "python3.7 run-server.py " + str(
        packageType.value) + " " + str(server_id) + " > ../out_files/" + packageType.name + "_mixed.txt 2> ../out_files/" + packageType.name + "_mixed_err.txt &"
    print("cmd", cmd)
    os.system(cmd)

os.system("disown -a")
