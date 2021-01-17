import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")

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
    "10.10.43.31": 10
}

cmd = "ip route get 1 | awk '{print $(NF-2);exit}'"
result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

ip = result.stdout.readlines()[0].decode('UTF-8')[:-1]
print("ip", ip)
server_id = servers_ips[ip]

for packageType in WishListPackagesTypes:
    cmd = "python3.7 run-server.py " + str(
        packageType.value) + " > ../out_files/" + packageType.name + ".txt 2> ../out_files/" + packageType.name + "_err.txt &"
    print("cmd", cmd)
    process = subprocess.Popen(cmd)

cmd = "disown -a"
print("cmd", cmd)
process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
