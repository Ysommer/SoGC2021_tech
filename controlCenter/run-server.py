import subprocess
import sys
sys.path.append("/home/gilbe/workspace/SoGC2021_tech/Utils")
sys.path.append("/home/gilbe/workspace/SoGC2021_tech")
from subprocess import Popen


from WishList import WishList, WishListPackagesTypes
from Utils.compress_solutions_and_validate import compress_best_and_send


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

packageType = None
if len(sys.argv) > 1:
    packageType = WishListPackagesTypes(int(sys.argv[1]))

cmd = "ip route get 1 | awk '{print $(NF-2);exit}'"
result = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

ip = result.stdout.readlines()[0].decode('UTF-8')[:-1]
print("ip", ip)
server_id = servers_ips[ip]

if packageType:
    WishList.farm_instances(packageType, server_id, 11)
    compress_best_and_send(packageType.name)
else:
    for packageType in WishListPackagesTypes:
        cmd = "python3.7 run-server.py "+str(int(packageType))+" > ../out_files/"+packageType+".txt 2> ../out_files/"+packageType+"_err.txt &"
        print("cmd", cmd)
        process = Popen([cmd])

    cmd = "disown -a"
    print("cmd", cmd)
    process = Popen([cmd])

