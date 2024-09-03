from backup3 import run_main_gui
import gc
from time import sleep
run_gui=True
import os
sleep(10)
while run_gui:

    try:
        run_gui = run_main_gui(run_gui)
        print(run_gui)
    except:
        sleep(10)
        gc.collect()
        os.system('sudo reboot')

    if run_gui:
        gc.collect()
        
        print("cleared memory")
        os.system('sudo reboot')
        run_gui=True