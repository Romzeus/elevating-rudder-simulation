from simulation import run_simulation
import socket
from time import sleep
import threading

hostIp = "localhost"
hostPort = 9999
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind((hostIp, hostPort))
loaderIp = "192.168.4.1"
loaderPort = 8888

def receive(params):
    global udp
    while True:
        try:
            message, addr = udp.recvfrom(8)
            params["angle"] = float(message.decode())
        except:
            pass

def transmit(params):
    global udp, loaderIp, loaderPort
    while True:
        udp.sendto(bytes(params["f"], "utf-8"), (loaderIp, loaderPort))
        sleep(1./240.)

params = {"f":0,"angle":0}
sim = threading.Thread(target=run_simulation,args=(params,))
rec = threading.Thread(target=receive,args=(params,), daemon=True)
tran = threading.Thread(target=transmit,args=(params,), daemon=True)
sim.start()
rec.start()
tran.start()
