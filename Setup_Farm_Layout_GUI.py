import PySimpleGUI as sg
import csv
from os.path import exists
import numpy as np
w,h = sg.Window.get_screen_size() #w/2 h/2 full screen size

#Try to read settings
def new_user_setup_farm():
    NAME_SIZE = 23

    def name(name):
        dots = NAME_SIZE-len(name)-2
        return sg.Text(name + ' ' + ' '*dots, size=(NAME_SIZE,1), justification='r',pad=(0,0), font='Courier 10')


    
    setting_layout_r = [
        [name('# lines of inspection'), sg.Slider((1,8), orientation='h')],
        [name('Sows/rails'), sg.Slider((5,50), orientation='h')],
        [name(" ")],
        [sg.Button("Exit"), sg.Button("Next")]
        ]
    window = sg.Window(title="Setting", layout=setting_layout_r, auto_size_text = True, element_justification='c')
    return window
    
def farm_layout_window(c,r):
    NAME_SIZE=23
    w,h = sg.Window.get_screen_size() #w/2 h/2 full screen size

    cc= ['Line 1', 'Line 2', 'Line 3', 'Line 4', 'Line 5', 'Line 6', 'Line 7', 'Line 8'] 
    element = [str( "sow 1 ~ "+ str(int(r)))]* int(c)
    #print(element)
    farm_layout = [
        [sg.Table( [element], cc[0:int(c)], num_rows=1)],
        [sg.Input(str(r),visible = False)],
        [* [ sg.Slider((1,2), orientation='h', disable_number_display=False,size = (8,15)) for i in range(c)]],
        [sg.Text("Inspect 1 or 2 sow per stop")],
        [sg.Button("Exit"), sg.Button("Done")]
        
        ]
    window = sg.Window(title="Layout", layout=farm_layout, auto_size_text = True, element_justification='c')
    return window


def setup_farm_layout_gui():
    
    window = new_user_setup_farm()
    setting1= True
    setting2=False
    while setting1:
        event, values = window.read()
        if event == sg.WIN_CLOSED or event == "Exit":
            #print("Break")
            window.close()
            break
        if event == "Next":
            #print(values[0])
            l_window = farm_layout_window(int(values[0]), int(values[1]))
            setting1 = False
            setting2 = True
            window.close()
    while setting2:
        event, values = l_window.read()
        if event == sg.WIN_CLOSED or event == "Exit":
            #print("Break")
            l_window.close()
            break
        if event == "Done":
            setup = []
            for v in range(1, len(values)):
                print(values[v])
                setup.append(int(values[v]))
            
            np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/farm_setting.txt", setup)
            r=setup[0]
            del setup[0]
            print(setup)

            total_line = sum(setup)

            r_list = list(range(1,int(values[1])+1))
            prepend_str=['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H','I','J','K','L']
            l_r = ['1_','2_']
            d_frame = r_list
            line=0
            #for i in range(0,total_line):
            for i in range(0,len(setup)):
                print("c", i)
                for k in range(0, setup[i]):
                    print(k)
                    p_str = prepend_str[i]+l_r[k]
                    #print(p_str)
                    p_str += '{0}'
                    rr_list = [p_str.format(j) for j in r_list]
                    d_frame = np.vstack((d_frame, rr_list))
            
            d_frame = np.delete(d_frame,0,0)
            #print(d_frame)
            np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/layout_id.csv", d_frame, fmt ="%s", delimiter = ',')
            
            a = np.zeros((d_frame.shape),int)
            a = a.astype(str)
            print(a)
            np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/sow_stall_id.csv", a, fmt ="%s", delimiter = ',')

            np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/monitor_status.csv", np.zeros((d_frame.shape)), delimiter = ',')
            setting2 = False
            l_window.close()

 



