from simulation import run_simulation
import socket
from time import sleep
import threading

hostIp = "localhost"
hostPort = 9999
receiveSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiveSock.bind((hostIp, hostPort))
sendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpIp = "127.0.0.1"
udpPort = 5005

def receive(params):
    global receiveSock
    while True:
        try:
            message, addr = receiveSock.recvfrom(1024)
            params["angle"] = float(message.decode())
        except:
            pass

def transmit(params):
    global sendSock, udpIp, udpPort
    while True:
        sendSock.sendto(bytes(params["f"], "utf-8"), (udpIp, udpPort))
        sleep(1./240.)

params = {"f":0,"angle":0,"t":0}
sim = threading.Thread(target=run_simulation,args=(params,))
rec = threading.Thread(target=receive,args=(params,), daemon=True)
tran = threading.Thread(target=transmit,args=(params,), daemon=True)
sim.start()
rec.start()
tran.start()
