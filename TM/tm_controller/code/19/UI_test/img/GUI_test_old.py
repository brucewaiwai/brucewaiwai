from tkinter import *
from tkinter import messagebox 
from tkinter import ttk
from PIL import Image
from PIL import ImageTk

import os
import serial
import datetime
import threading
import time

import pickle
import subprocess
from functools import partial
import tkinter.font as font

HEADER = 0xff
uart = [0] * 4

IM_DIRECTORY = "Image"
setting_changed = threading.Event()
WATER_IM = "/Water-180/1.png"

################################FUNCTIONS
#water_sensor
def water_sensor():
	global dist
	global time

	global height
	global volume
	global percent

	print("Water level sensor setup [success]")
	max_height = 35
	#detect water sensor reading
	port = serial.Serial(port="/dev/ttyS0", baudrate=9600, timeout=3.0)
	while True:
		#byte = port.read(4)
		#height = (byte[2])/10 #in mm
		if (port.read(1)[0] == HEADER):
			uart[0] = HEADER
			for i in range(1,4):
				uart[i] = port.read(1)[0]
			check = (uart[0] + uart[1] + uart[2])& 0xff
			if (uart[3] == (check & 0xff)):
				height = (uart[1]*256 + uart[2])/10
				volume = (0.340*0.180*(height/100))*1000
				percent = (height/max_height)*100
				h.set(f'{height}cm')
				v.set(f'{"{:.2f}".format(volume)}L')
				p.set(f'{"{:.2f}".format(percent)}%')
			#print("["+timestamp+"]"+" Height: "+str(height)+"; Volume:"+str(volume)+"; Percent:"+str(percent))
			
			###########Change image on level
			if volume < 2:
				WATER_IM = "/Water-180/low.png"
				level.set('LOW\nlevel')
				warning.set("Please\nreplace\ndisinfectant\nnow")

			elif volume > 2 and volume < 5:
				WATER_IM = "/Water-180/medium.png"
				level.set("MEDIUM\nlevel")
				warning.set("Sufficient\namount of\ndisinfectant")

			else:
				WATER_IM = "/Water-180/high.png"
				level.set("HIGH\nlevel")
				warning.set("Full tank\nof disinfectant")

			img2 = PhotoImage(file = IM_DIRECTORY+WATER_IM)
			level_label.configure(image=img2)
			level_label.image = img2

			if setting_changed.isSet():
				d.set(dist+' meters')
				t.set(time+' seconds')
			elif setting_changed.isSet() == False:
				d.set('0.1 meters')
				t.set('7 seconds')

############Keyboard
num_run1 = 0
btn_funcid1 = 0
num_run2 = 0
btn_funcid2 = 0

def click1(btn):
	global num_run1
	text = "%s" % btn
	if not text == "Del" and not text == "Close":
		new_d.insert(END, text)
	if text == 'Del':
		new_d.delete(0, END)
	if text == 'Close':
		boot1.destroy()
		num_run1 = 0
		root.unbind('<Button-1>', btn_funcid1)

def click2(btn):
	global num_run2
	text = "%s" % btn
	if not text == "Del" and not text == "Close":
		new_t.insert(END, text)
	if text == 'Del':
		new_t.delete(0, END)
	if text == 'Close':
		boot2.destroy()
		num_run2 = 0
		root.unbind('<Button-1>', btn_funcid2)

def close1(event):
	global num_run1, btn_funcid1
	boot1.destroy()
	num_run1 = 0
	root.unbind('<Button-1>', btn_funcid1)

def close2(event):
	global num_run1, num_run2, btn_funcid1, btn_funcid2
	boot2.destroy()
	num_run2 = 0
	root.unbind('<Button-1>', btn_funcid2)

def run1(event):
	global num_run1, btn_funcid1
	global boot1
	if num_run1 == 0:
		num_run1 = 1
	#print('first entry')
	boot1 = Tk()
	boot1.title("NumPad")
	boot1['bg'] = '#5b97ca'
	lf = LabelFrame(boot1, text=" keypad ", bd=3)
	lf.pack(padx=15, pady=10)
	#numpad()
	btn_list = [
		'7',  '8',  '9',
		'4',  '5',  '6',
		'1',  '2',  '3',
		'0',  '.',  'Del',
		'',  'Close',  '']		
	r = 1
	c = 0
	n = 0
	btn = list(range(len(btn_list)))

	for label in btn_list:
		cmd = partial(click1, label)
		btn[n] = Button(lf, text=label, width=10, height=3, command=cmd)
		btn[n].grid(row=r, column=c)
		n += 1
		c += 1
		if c == 3:
			c = 0
			r += 1
	###############end of numpad
	btn_funcid1 = root.bind('<Button-1>', close1)
	

def run2(event):
	global num_run2, btn_funcid2
	global boot2
	if num_run2 == 0:
		num_run2 = 1
	#print('second entry')
	boot2 = Tk()
	boot2.title("NumPad")
	boot2['bg'] = '#5b97ca'
	lf = LabelFrame(boot2, text=" keypad ", bd=3)
	lf.pack(padx=15, pady=10)
	#numpad()
	btn_list = [
		'7',  '8',  '9',
		'4',  '5',  '6',
		'1',  '2',  '3',
		'0',  '.',  'Del',
		'',  'Close',  '']		
	r = 1
	c = 0
	n = 0
	btn = list(range(len(btn_list)))

	for label in btn_list:
		cmd = partial(click2, label)
		btn[n] = Button(lf, text=label, width=10, height=3, command=cmd)
		btn[n].grid(row=r, column=c)
		n += 1
		c += 1
		if c == 3:
			c = 0
			r += 1
	###############end of numpad
	btn_funcid2 = root.bind('<Button-1>', close2)

#get user input for new settings
def change_settings():
	global dist
	global time

	dist = new_d.get()
	time = new_t.get()
	setting_changed.set()
	data = [dist, time]
	with open('newsettings', 'wb') as f:
		#file = open('newsettings', 'wb')
		pickle.dump(data, f)
		f.close()
	print("Changing disinfection parameters. D: "+dist+" t: "+time)
	new_d.delete(0, END)
	new_t.delete(0, END)

def reset_settings():
	with open('newsettings', 'wb') as f:
		pickle.dump([0.1,7], f)
		f.close()

	with open('modesettings', 'wb') as f:
		pickle.dump(False, f)
		f.close()
	
	setting_changed.clear()
	print("Resetting to original parameters")

def always_on(arg):
	if arg == 1:
		ALWAYS_ON = True
		print("Disinfection mode is ALWAYS ON")
	elif arg == 2:
		ALWAYS_ON = False
		print("Disinfection mode is SENSOR ON")

	with open('modesettings', 'wb') as f:
		pickle.dump(ALWAYS_ON, f)
		f.close()
		print(ALWAYS_ON)

try:
	global height
	global volume
	global percent

	#########################MAIN
	root = Tk()
	root.title("Coronaspray GUI")
	root.configure(background='white')
	#root.geometry("800x480")
	root.attributes('-fullscreen', True)

	#Initialise variables for thread
	h = StringVar() 
	v = StringVar() 
	p = StringVar() 

	t = StringVar()
	d = StringVar()

	warning = StringVar()
	level = StringVar()

	task = threading.Thread(target=water_sensor, daemon = True)

	#######Tabs
	style = ttk.Style(root)
	style.configure('lefttab.TNotebook', background='#5b97ca', tabposition='w')

	icon1 = Image.open(IM_DIRECTORY+"/Tab-80/1.png")
	icon2 = Image.open(IM_DIRECTORY+"/Tab-80/2.png")
	icon3 = Image.open(IM_DIRECTORY+"/Tab-80/3.png")

	im_tab1 = ImageTk.PhotoImage(icon1)
	im_tab2 = ImageTk.PhotoImage(icon2)
	im_tab3 = ImageTk.PhotoImage(icon3)

	tabControl = ttk.Notebook(root, style='lefttab.TNotebook')
	tab1 = ttk.Frame(tabControl)
	tab2 = ttk.Frame(tabControl)
	tab3 = ttk.Frame(tabControl)

	tabControl.add(tab1, image=im_tab1, compound=CENTER)
	tabControl.add(tab2, image=im_tab2, compound=CENTER)
	tabControl.add(tab3, image=im_tab3, compound=CENTER)

	tabControl.pack(expand=1, fill=BOTH)

	####################################Tab1: coronaspray current settings
	#configure row&column symmetry
	tab1.columnconfigure(0, weight=1, uniform='a')
	tab1.columnconfigure(1, weight=1, uniform='a')
	tab1.columnconfigure(2, weight=1, uniform='a')
	tab1.rowconfigure(0, weight=1, uniform='a')
	tab1.rowconfigure(1, weight=1, uniform='a')
	tab1.rowconfigure(2, weight=1, uniform='a')
	tab1.rowconfigure(3, weight=1, uniform='a')
	tab1.rowconfigure(4, weight=1, uniform='a')

	#Choose disinfection mode
	ttk.Label(tab1, text="Please choose\ndisinfection mode", font=('Consolas', 12, 'bold')).grid(row = 0, column = 0)
	ttk.Button(tab1, text="Always ON", command=lambda: always_on(1)).grid(row=0, column=1)
	ttk.Button(tab1, text="Sensor mode", command=lambda: always_on(2)).grid(row=0, column=2)

	#Header
	ttk.Label(tab1, text="Parameters", font=('Consolas', 12, 'bold')).grid(row = 1, column = 0)
	ttk.Label(tab1, text="Current settings", font=('Consolas', 12, 'bold')).grid(row = 1, column = 1)
	ttk.Label(tab1, text="New settings", font=('Consolas', 12, 'bold')).grid(row = 1, column = 2)
	#labels
	ttk.Label(tab1, text="Trigger distance: ", font=('Consolas', 12, 'bold')).grid(row = 2, column = 0)
	ttk.Label(tab1, text="Spray time: ", font=('Consolas', 12, 'bold')).grid(row = 3, column = 0)

	ttk.Label(tab1, textvariable=d, font=('Consolas', 12, 'bold')).grid(row = 2, column = 1) 
	ttk.Label(tab1, textvariable=t, font=('Consolas', 12, 'bold')).grid(row = 3, column = 1)
	#new settings
	new_d = ttk.Entry(tab1, font=('Consolas', 12, 'bold'))
	new_d.grid(row = 2, column = 2)
	new_d.bind('<Button-1>', run1)
	#new_d.extra = 'entry1'
	new_t = ttk.Entry(tab1, font=('Consolas', 12, 'bold'))
	new_t.grid(row = 3, column = 2)
	new_t.bind('<Button-1>', run2)
	#new_t.extra = 'entry2'
	#save buttons
	ttk.Button(tab1, text="Reset original\nsettings", command=reset_settings).grid(row=4, column=1)
	ttk.Button(tab1, text="Modify\nsettings", command=change_settings).grid(row=4, column=2)

	####################################Tab2: Water
	tab2.columnconfigure(0, weight=1, uniform='a')
	tab2.columnconfigure(1, weight=1, uniform='a')
	tab2.columnconfigure(2, weight=1, uniform='a')
	tab2.columnconfigure(3, weight=1, uniform='a')
	tab2.rowconfigure(0, weight=2, uniform='a')
	tab2.rowconfigure(1, weight=0, uniform='a')
	tab2.rowconfigure(2, weight=2, uniform='a')
	###water background image
	#im1 = PhotoImage(file = IM_DIRECTORY+"/Water-180/1.png")
	im2 = PhotoImage(file = IM_DIRECTORY+"/Water-180/2.png")
	im3 = PhotoImage(file = IM_DIRECTORY+"/Water-180/3.png")
	im4 = PhotoImage(file = IM_DIRECTORY+"/Water-180/4.png")

	ttk.Label(tab2, image=im2).grid(row = 0, column = 1)
	ttk.Label(tab2, image=im3).grid(row = 0, column = 2)
	ttk.Label(tab2, image=im4).grid(row = 0, column = 3)
	
	###labels		
	im1 = PhotoImage(file = IM_DIRECTORY+WATER_IM)
	level_label = ttk.Label(tab2, image=im1)
	level_label.grid(row = 0, column = 0)

	ttk.Label(tab2, textvariable=warning, font=('Consolas', 15, 'bold')).grid(row = 1, column = 0)
	ttk.Label(tab2, text="Liquid\nheight", font=('Consolas', 15, 'bold')).grid(row = 1, column = 1)
	ttk.Label(tab2, text="Remaining\nvolume", font=('Consolas', 15, 'bold')).grid(row = 1, column = 2)
	ttk.Label(tab2, text="Remaining\npercentage", font=('Consolas', 15, 'bold')).grid(row = 1, column = 3)
	###measurent
	ttk.Label(tab2, textvariable=level, font=('Consolas', 28, 'bold')).grid(row = 2, column = 0)
	ttk.Label(tab2, textvariable=h, font=('Consolas', 28, 'bold')).grid(row = 2, column = 1)
	ttk.Label(tab2, textvariable=v, font=('Consolas', 28, 'bold')).grid(row = 2, column = 2)
	ttk.Label(tab2, textvariable=p, font=('Consolas', 28, 'bold')).grid(row = 2, column = 3)

	###log disinfectant data into file
	def log_data():
		timestamp = str(datetime.datetime.now())
		f_path = "LOGDATA"
		if os.path.exists(f_path) == False:
			os.makedirs(f_path, mode = 0o777)
			print(timestamp+": LOGDATA directory was not found. Creating new directory.")
		fo = open('LOGDATA/datalog.txt', "a+")
		#log = 'Disinfectant status '+timestamp+'\nHeight: '+f'{"{:.2f}".format(height)}cm'+'\nVolume: '+f'{"{:.2f}".format(volume)}L'+'\nPercent '+f'{"{:.2f}".format(percent)}%'+'\n-----------------\n'
		log = 'Disinfectant status '+timestamp+'\nHeight: '+f'{(height)}cm'+'\nVolume: '+f'{(volume)}L'+'\nPercent '+f'{(percent)}%'+'\n-----------------\n'
		fo.write(log)
		fo.close()
		print(timestamp+": Data logged Successfully!")
	log_tab2 = ttk.Button(tab2, text="LOG DATA", command=log_data).grid(row=3, column=1)

	##################################Tab3: Infographics
	tab3.columnconfigure(0, weight=0, uniform='a')
	tab3.rowconfigure(0, weight=0, uniform='a')
	im_info = PhotoImage(file = IM_DIRECTORY+"/info.png")
	ttk.Label(tab3, image=im_info).grid(row = 0, column = 0)

	########################################END#############	
	#exit function
	def exit_f(): 
		root.destroy()
		print("Shutdown successfully")
		sys.exit(0)

	#exit button
	exit_tab1 = ttk.Button(tab1, text="EXIT", command=exit_f).grid(row=4, column=0)
	exit_tab2 = ttk.Button(tab2, text="EXIT", command=exit_f).grid(row=3, column=0)
	exit_tab3 = ttk.Button(tab3, text="EXIT", command=exit_f).grid(row=1, column=0) #make bg white

	task.start()
	root.mainloop()

except KeyboardInterrupt as e:
	print(e)
	print("Shutdown successfully")
	sys.exit(0)
