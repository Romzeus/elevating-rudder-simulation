from simulation import run_simulation
import numpy as np
from time import sleep
import threading

def angle_fun(params):
    while params["t"] < 100:
        params["angle"] = 0.3*np.sin(0.5*np.pi*params["t"])
        params["t"] += 1./240.
        print(params["f"])
        sleep(1./240.)

params = {"f":0,"angle":0,"t":0}
sim = threading.Thread(target=run_simulation,args=(params,))
ang = threading.Thread(target=angle_fun,args=(params,), daemon=True)
ang.start()
sim.start()