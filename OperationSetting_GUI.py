import PySimpleGUI as sg
#import csv
from os.path import exists
import numpy as np
from Setup_Farm_Layout_GUI import setup_farm_layout_gui
from re import search
import json
from datetime import date
#import re
#import itertools
import ast
import os
w,h = sg.Window.get_screen_size() #w/2 h/2 full screen size
def check_duplicate(stall_sow_id,main_window,monitoring):
    for i in range(stall_sow_id.shape[0]):
        for j in range(stall_sow_id.shape[1]):
            main_window["stall_sow_text"+str(i)+str(j)].update(background_color=["lightgray", "green"][int(monitoring[i,j])])
    mask = stall_sow_id.astype(int)>0
    #print(mask)
    dup, unq_cnt = (np.unique(stall_sow_id[mask], return_counts=True))
    dup = dup[unq_cnt>1]
    for d in dup:
        [i,j]= np.where(stall_sow_id == d)
        for ii in range(len(i)):
            main_window["stall_sow_text"+str(i[ii])+str(j[ii])].update(background_color="red")

        #print(i,j)
def move_sow_records(old_path, new_path, main_window,sow_lists, farrow_lists):
    if not exists(old_path):
        sg.popup_ok("Record not exist, try create record instead")
    else:      
        sow_info = open(old_path)
        sow_info = json.load(sow_info)
        sow_info = json.dumps(sow_info)
        #l = len([f.split('.')[0] for f in os.listdir('Removed_Sow_ID')])
        if not exists(new_path): 
            with open(new_path, 'w') as outfile:
                outfile.write(sow_info)
        #Update sow lists
        sow_lists =[f.split('.')[0] for f in os.listdir('Sow_ID')]

        if len(sow_lists)==0:
            sow_lists = ['No ID records']
        main_window["sow_lists"].update(values = sow_lists )
        #Update Farrowing List
        farrow_lists =[f.split('.')[0] for f in os.listdir('Farrow_Sow_ID')]
        if len(farrow_lists)==0:
            farrow_lists = ['No ID records']
        main_window["farrow_lists"].update(values = farrow_lists )
        os.remove(old_path)
    return sow_lists, farrow_lists        
def update_sow_stall_records(sow_id, sow_stall, ids, main_window,stall_sow_id, monitoring):
    #print(sow_id, sow_stall)
    #print(stall_sow_id)
    [i,j] = np.where(ids == sow_stall)
    
    stall_sow_id[i,j] = sow_id
    #print("stall_sow_text"+str(i)+str(j))
    main_window["stall_sow_text"+str(i[0])+str(j[0])].update(value=sow_stall+ ": " +sow_id )
    stall_sow_id[i,j] = sow_id
    stall_sow_id1 = stall_sow_id.astype(str)
    stall_sow_id1 = (np.char.zfill(stall_sow_id1, 5))
    np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/sow_stall_id.csv", stall_sow_id1, fmt ="%s", delimiter = ',')
    check_duplicate(stall_sow_id,main_window,monitoring)
    return stall_sow_id


    
def update_monitoring(ids, selected,main_window):
    #select = np.asanyarray(selected)
    selected = np.concatenate(selected).ravel().tolist()
    #selected.flatten()
    monitoring=np.zeros(ids.shape)
    
    #loc = (ids[:,:,None] == selected).argmax(axis=1)
    for s in selected:
        
        monitoring[np.where(ids == s)]=1
    
    np.savetxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/monitor_status.csv", monitoring, delimiter = ',')
    for i in range(ids.shape[0]):
        for j in range(ids.shape[1]):
            main_window["stall_sow_text"+str(i)+str(j)].update(background_color=["lightgray", "green"][int(monitoring[i,j])])
    return monitoring
#def read_text_input():
    
def create_sow_record(values):
    sow_info = {
        "ID":values["-Sow-ID-"],
        "breed" : values["-Sow-Breed-"],
        "removal_date": values["-Remove-date-"],
        "removal_reason": values["-Remove-Reason-"],
        "parity" : [{
            #int(values["-Sow-Parity-"]) : {
            "P": int(values["-Sow-Parity-"]),
            "weaning_date": values["-Wean-date-"],
            "AI_date": [values["-AI-date-"]],
            "total_born": values["-Tot-Born-"],
            "tot_born_alive": values["-Born-Alive-"],
            "tot_wean": values["-Tot-Weaned-"],
            "Other_notes": [values["-Other-Notes-"]],
            "time_stamp": [str(date.today())]
            #}
            }]
        }
    #print(json.dumps(sow_info, indent=8, sort_keys=True))

    sow_info = json.dumps(sow_info)
    file_path = "Sow_ID/" + str(values["-Sow-ID-"]) + ".json"
    if not exists(file_path): 
        with open(file_path, 'w') as outfile:
            outfile.write(sow_info)
            #main_window['listbox_'+str(selection)].update(set_to_index = list(range(0,ids.shape[1])))

    else:
        sg.popup_ok("Record already exist, try update record instead")
def search_sow(main_window, values):
    file_path = "Sow_ID/" + str(values["-Sow-ID-"]) + ".json"
    if not exists(file_path):
        sg.popup_ok("Record not exist, try create record instead")
    else:      
        sow_info = open(file_path)
        sow_info = json.load(sow_info)
        #print(json.dumps(sow_info, indent=8, sort_keys=True))

        main_window["-Sow-Breed-"].update(sow_info['breed'])
        main_window["-Remove-date-"].update(sow_info['removal_date'])
        main_window["-Remove-Reason-"].update(sow_info['removal_reason'])
        sow_info = max(sow_info['parity'], key=lambda x: x['P'])
        main_window["-Sow-Parity-"].update(sow_info['P'])
        main_window["-Wean-date-"].update(sow_info['weaning_date'])
        main_window["-AI-date-"].update(sow_info['AI_date'].pop())
        main_window["-Tot-Born-"].update(sow_info['total_born'])
        main_window["-Born-Alive-"].update(sow_info['tot_born_alive'])

        main_window["-Tot-Weaned-"].update(sow_info['tot_wean'])
        main_window["-Other-Notes-"].update(str(sow_info['Other_notes'].pop()))

def update_sow_records(values):
    file_path = "Sow_ID/" + str(values["-Sow-ID-"]) + ".json"
    if not exists(file_path):
        sg.popup_ok("Record not exist, try create record instead")
    else:
        sow_info = open(file_path)
        sow_info = json.load(sow_info)
        current_parity =int(values["-Sow-Parity-"])
        #sow_info = max(sow_info['parity'], key=lambda x: x['P'])

        if current_parity> max(sow_info['parity'], key=lambda x: x['P'])['P']:
            #print("add new child")
            sow_info['parity'].append({
                "P": int(values["-Sow-Parity-"]),
                "weaning_date": values["-Wean-date-"],
                "AI_date": [values["-AI-date-"]],
                "total_born": values["-Tot-Born-"],
                "tot_born_alive": values["-Born-Alive-"],
                "tot_wean": values["-Tot-Weaned-"],
                "Other_notes": [values["-Other-Notes-"]],
                "time_stamp": [str(date.today())]
                })
        else:
            #print("fill existing")
            #print(sow_info['parity'][-1])
            sow_info['parity'][-1]['weaning_date'] = values["-Wean-date-"]
            if values["-AI-date-"] not in sow_info['parity'][-1]['AI_date'] and values["-AI-date-"] != "" :
                sow_info['parity'][-1]['AI_date'].append(values["-AI-date-"])
            sow_info['parity'][-1]['total_born']= (values["-Tot-Born-"])
            sow_info['parity'][-1]['tot_born_alive']= (values["-Born-Alive-"])
            sow_info['parity'][-1]['tot_wean']= (values["-Tot-Weaned-"])
            if values["-Other-Notes-"] not in sow_info['parity'][-1]['Other_notes'] and values["-Other-Notes-"] != "" :
                sow_info['parity'][-1]['Other_notes'].append(values["-Other-Notes-"])
            d = str(date.today())
            if d not in sow_info['parity'][-1]['time_stamp']:
                sow_info['parity'][-1]['time_stamp'].append(d)

            #print(sow_info['parity'][-1])
        #print(json.dumps(sow_info, indent=8, sort_keys=True))

        sow_info = json.dumps(sow_info)
        with open(file_path, 'w') as outfile:
            outfile.write(sow_info)

        
def main_gui_window(ids, monitoring, stall_sow_id,sow_lists,farrow_lists):
    print(w)
    print(w/8)
    #print(ids.shape)
   
    c,r = ids.shape
    #c=1
    #r=5
    #print(ids[1])
    
    stall_layout = [[sg.Frame("Monitoring Status",
        [
        #[* [ sg.Text(i) for i in range(c)]],
        [* [ sg.Button("  Select All Line "+ str(i)+"  ", size = (15,1)) for i in range(c)]],
        [* [ sg.Button("Remove Selection Line "+ str(i), size = (15,1)) for i in range(c)]],
        [* [ sg.Listbox(ids[i],default_values = ids[np.where(monitoring==1)], size = (15,21), highlight_background_color ='green', expand_x = True, expand_y=True, no_scrollbar=False, select_mode="multiple",enable_events=True, key = 'listbox_'+str(i))  for i in range(c)]],
        [sg.Button("Save Selection",size = (15,1))]
        ])
        ]]

    arrange_stall = [[sg.Frame("Monitoring Summary", 
        [
            #[sg.OptionMenu(values = sow_lists, default_value = ids_s[i][j],size = (int(w/(10*c*4)),3)) for i in range(c)] for j in range(r)
            [sg.Text(ids[i][j] +": "+ str(stall_sow_id[i][j]), size = (12,1), key="stall_sow_text"+str(i)+str(j), background_color = ["lightgray", "green"][monitoring[i,j]]) for i in range(c)] for j in range(r)
        ])
        ], [sg.Button("Check Duplicate",size = (12,1))]
                     ]
    info_layout = [[sg.Frame("Enter Sow Information",
        [
            
            [sg.Text("Sow ID (5-digits): ",size = (30,1)), sg.Input(key="-Sow-ID-", do_not_clear=True, size = (20,1))],
            [sg.Button("Search Sow")],
            [sg.Text("Sow Breeds: ",size = (30,1)), sg.Input(key="-Sow-Breed-", do_not_clear=True, size = (20,1))],
            [sg.Text("Sow Parity: ",size = (30,1)), sg.Input(key="-Sow-Parity-",do_not_clear=True, size = (20,1))],
            [sg.Text("Weaning Date (YYYY-MM-DD): ",size = (30,1)), sg.Input(key="-Wean-date-",do_not_clear=True, size = (20,1))],
            [sg.Text("Sow AI Time (YYYY-MM-DD-HH) ",size = (30,1)), sg.Input(key="-AI-date-",do_not_clear=True, size = (20,1))],
            [sg.Text("Total Born ",size = (30,1)), sg.Input(key="-Tot-Born-",do_not_clear=True, size = (20,1))],
            [sg.Text("Total Born Alive ", size = (30,1)), sg.Input(key="-Born-Alive-",do_not_clear=True, size = (20,1))],
            [sg.Text("Total Weaned ", size = (30,1)), sg.Input(key="-Tot-Weaned-",do_not_clear=True, size = (20,1))],
            [sg.Text("Removal Date (YYYY-MM-DD): ",size = (30,1)), sg.Input(key="-Remove-date-",do_not_clear=True, size = (20,1))],
            [sg.Text("Removal Reason (sick/lame/etc)", size = (30,1)), sg.Input(key="-Remove-Reason-",do_not_clear=True, size = (20,1))],
            [sg.Text("Other Notes", size = (30,1)), sg.Input(key="-Other-Notes-",do_not_clear=True, size = (20,1))],
            [sg.HSeparator(pad =(5,5))],
            [sg.Button("Create Sow Records",size = (23,1)),
            sg.Button("Update Sow Records", size = (23,1))],
            [sg.Button("Move Sow to Farrow Crate", size = (23,1)),
            sg.Button("Move Sow from Farrow Crate", size = (23,1))],
            [sg.Button("Remove Sow Records", size = (23,1)),
            sg.Button("Clear Entry", size=(23,1))],
            [sg.OptionMenu(default_value = "See Gestation Sow IDs",size=(23,1), values = np.sort(sow_lists), key="sow_lists"),  sg.Button("ViewRecordG"),sg.Text("Gestation")],
            [sg.OptionMenu(default_value = "See Farrowing Sow IDs",size=(23,1), values = np.sort(farrow_lists), key="farrow_lists"),sg.Button("ViewRecordF"),sg.Text("Farrow")],

            [sg.HSeparator()],
            [sg.Text("Assign stall for sow", size=(20,1)), sg.OptionMenu(values = ids.flatten(), key="Sow_to_Stall", size=(10,1)), sg.Button("Confirm Stall")]
        ])
        ]]
    window_layout = [
        [
            [sg.Column(info_layout, expand_y = True,size=(400,80)),
            sg.VSeparator(),
            sg.Column(stall_layout, expand_y = True, size=(310,80)),
            sg.VSeparator(),
            sg.Column(arrange_stall,expand_y = True, size=(250,80))],
            [sg.HSeparator(pad =(5,5))],
            [sg.Button("HELP"), sg.Button("Continue", size = (100,1)), sg.Button("Exit")]
         ]]
    window = sg.Window(title="Edit Sow Info", layout=window_layout, resizable=True, no_titlebar=False, size=(1000,650), auto_size_text = True, element_justification='c',finalize=True)
    #window.Maximize()
    
    return window


def operation_setting_gui(ids, monitoring, stall_sow_id):

    sow_lists =[f.split('.')[0] for f in os.listdir('Sow_ID')]
    if len(sow_lists)==0:
        sow_lists = ['No ID records']
    #print(sow_lists)  
    farrow_lists =[f.split('.')[0] for f in os.listdir('Farrow_Sow_ID')]
    if len(farrow_lists)==0:
        farrow_lists = ['No ID records']
    #print(farrow_lists)    
    previous_setting = np.loadtxt("/home/pi/Desktop/ROBOTSOFTWARE/Config/farm_setting.txt")
    #ids = np.genfromtxt('Config/layout_id.csv', delimiter = ',', dtype = None, encoding=None)
    
    #monitoring = np.genfromtxt('Config/monitor_status.csv', dtype = int, delimiter = ',')
    #stall_sow_id = np.genfromtxt('Config/sow_stall_id.csv', dtype = int, delimiter = ',')
    #if len(ids.shape)==1:
     #   ids = np.transpose(ids.reshape(-1,1))
      #  monitoring = np.transpose(monitoring.reshape(-1,1))
       # stall_sow_id = np.transpose(stall_sow_id.reshape(-1,1))

    #mask = stall_sow_id>0
    #stall_sow_id = stall_sow_id.astype(str)
    #stall_sow_id[mask] = (np.char.zfill(stall_sow_id[mask], 5))
    #print(stall_sow_id)

    r=previous_setting[0]
    c=0
    for i in range(1,len(previous_setting)):
        c = c+int(previous_setting[i])
    main_window = main_gui_window(ids, monitoring,stall_sow_id,sow_lists,farrow_lists)
    while True:
        event, values = main_window.read()
        #print(event)
        if event == sg.WIN_CLOSED or event == "Exit":
                #print("Break")
                main_window.close()
                break
        if event == "Continue":
            #main_window.close()
            #print(monitoring)
            #print(stall_sow_id)
            main_window.close()
            return monitoring, stall_sow_id
        if event == "Save Selection":
            selected = []
            i=0
            
            for i in range(c):
                
                selected.append(values['listbox_'+str(i)])
            #print(selected)    
            update = sg.Popup("Confirm?", button_type=4)
            if update =="OK":
                #print("OK")
                monitoring = update_monitoring(ids,selected, main_window)
            if update == "Cancel":
                print("Cancel")
        if search("Remove Selection", event):
            #print(event.split()[-1])
            selection = event.split()[-1]
            #print(values)
            #monitoring=np.zeros((ids.shape[1]))
            main_window['listbox_'+str(selection)].update(set_to_index = -1)
        if search("Select All", event):
            #print(event.split()[-1])
            selection = event.split()[-1]
            #print(range(ids.shape[1]))
            main_window['listbox_'+str(selection)].update(set_to_index = list(range(0,ids.shape[1])))
        if event == "Create Sow Records":
            update = sg.Popup("Confirm add sow to the records?", button_type=4)
            if update =="OK":
                #print("OK")
                if values["-Sow-Parity-"] != "" and values["-Sow-ID-"]!= "" :
                    if values["-Sow-ID-"] not in farrow_lists and values["-Sow-ID-"] not in sow_lists:
                        create_sow_record(values)
                        sow_lists =[f.split('.')[0] for f in os.listdir('Sow_ID')]
                        #print("sow lists")
                        #print(len(sow_lists))
                        if len(sow_lists)==0:
                            sow_lists = ['No ID records']
                        main_window["sow_lists"].update(values = sow_lists )
                    else:
                        sg.popup_ok("Record exists already")
                else:
                    sg.popup_ok("Enter ID and parity numebr, try again")

            if update == "Cancel":
                print("Cancel")
        if event == "Search Sow":
            search_sow(main_window, values)
        if event == "Update Sow Records":
            update = sg.Popup("Confirm update sow records?", button_type=4)
            if update =="OK":
                #print("OK")
                update_sow_records(values)
            if update == "Cancel":
                print("Cancel")
        if event == "Confirm Stall":
            if values["-Sow-ID-"] not in sow_lists :
                sg.popup_ok("Did not find existing record for the sow, create record first")
            elif values["Sow_to_Stall"] not in ids:
                sg.popup_ok("Select Stall")

            else:
                #print(values["Sow_to_Stall"])
                update = sg.Popup("Put Sow" +values["-Sow-ID-"] + " in " + values["Sow_to_Stall"]+ " stall?", button_type=4)
                if update =="OK":
                    #print("OK")
                    stall_sow_id = update_sow_stall_records(values["-Sow-ID-"], values["Sow_to_Stall"],ids, main_window,stall_sow_id,monitoring)
                if update == "Cancel":
                    print("Cancel")
        if event =="Clear Entry":
            for key in ["-Sow-ID-","-Sow-Breed-","-Sow-Parity-","-Other-Notes-","-Wean-date-","-AI-date-", "-Tot-Born-", "-Born-Alive-","-Tot-Weaned-","-Remove-date-","-Remove-Reason-"]:
                main_window[key]('')
        if event =="Remove Sow Records":
            try:
                if values["-Sow-ID-"] in sow_lists :
                    update = sg.Popup("Delet Sow " +values["-Sow-ID-"]+ " ?", button_type=4)
                    if update == "OK":
                        #print("A")
                        old_path = "/home/pi/Desktop/ROBOTSOFTWARE/Sow_ID/" + str(values["-Sow-ID-"]) + ".json"                
                        l = len([f.split('.')[0] for f in os.listdir('Removed_Sow_ID')])
                        new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Removed_Sow_ID/"+str(l)+".json"
                        sow_lists, farrow_lists = move_sow_records(old_path, new_path, main_window,sow_lists, farrow_lists)
                elif values["-Sow-ID-"] in farrow_lists :
                    update = sg.Popup("Delet Sow " +values["-Sow-ID-"]+ " ?", button_type=4)
                    if update == "OK":
                        #print("A")
                        old_path = "/home/pi/Desktop/ROBOTSOFTWARE/Farrow_Sow_ID/" + str(values["-Sow-ID-"]) + ".json"                
                        l = len([f.split('.')[0] for f in os.listdir('Removed_Sow_ID')])
                        new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Removed_Sow_ID/"+str(l)+".json"
                        sow_lists, farrow_lists = move_sow_records(old_path, new_path,main_window,sow_lists, farrow_lists)
                else:
                    sg.popup_ok("Sow record not found in folder")
            except:
                print("")

        if event == "Move Sow to Farrow Crate":
            if values["-Sow-ID-"] in sow_lists :
                update = sg.Popup("Move Sow " +values["-Sow-ID-"]+ "to farrowing crate?", button_type=4)
                if update == "OK":
                    #print("A")
                    old_path = "/home/pi/Desktop/ROBOTSOFTWARE/Sow_ID/" + str(values["-Sow-ID-"]) + ".json"                
                    new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Farrow_Sow_ID/"+str(values["-Sow-ID-"])+".json"
                    sow_lists, farrow_lists = move_sow_records(old_path, new_path,main_window,sow_lists, farrow_lists)
                    #print(farrow_lists)
                    #print(sow_lists)
            else:
                sg.popup_ok("Sow record not found in folder") 
                

        if event == "Move Sow from Farrow Crate":
            if values["-Sow-ID-"] in farrow_lists :
                update = sg.Popup("Move Sow " +values["-Sow-ID-"]+ "to farrowing crate?", button_type=4)
                if update == "OK":
                    #print("A")
                    old_path = "/home/pi/Desktop/ROBOTSOFTWARE/Farrow_Sow_ID/" + str(values["-Sow-ID-"]) + ".json"                
                    new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Sow_ID/"+str(values["-Sow-ID-"])+".json"
                    sow_lists, farrow_lists = move_sow_records(old_path, new_path,main_window,sow_lists, farrow_lists)
                    #print(farrow_lists)
                    #print(sow_lists)
            else:
                #print("NO records")
                sg.popup_ok("Sow record not in farrowing folder")
                
        if event == "Check Duplicate":
            check_duplicate(stall_sow_id,main_window,monitoring)
            #stall_sow_id = stall_sow_id.astype(str)
            #stall_sow_id[mask] = (np.char.zfill(stall_sow_id[mask], 5))
        if event == "ViewRecordG":
            if values["sow_lists"] not in sow_lists :
                sg.popup_ok("no record found in Gestation folder")
            else:
                new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Sow_ID/"+str(values["sow_lists"])+".json"
                sow_info = open(new_path)
                sow_info = json.load(sow_info)
                sg.popup_ok(json.dumps(sow_info, indent=8, sort_keys=True))
        if event == "ViewRecordF":
            if values["farrow_lists"] not in sow_lists :
                sg.popup_ok("no record found in Gestation folder")
            else:
                new_path = "/home/pi/Desktop/ROBOTSOFTWARE/Sow_ID/"+str(values["farrow_lists"])+".json"
                sow_info = open(new_path)
                sow_info = json.load(sow_info)
                sg.popup_ok(json.dumps(sow_info, indent=8, sort_keys=True))
        if event == "HELP":
            sg.popup_ok("Explain butons")
                
#operation_setting_gui()
