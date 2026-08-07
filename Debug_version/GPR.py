#GPR.py
#<c> Mirel Paun 2026

from __future__ import print_function
import serial
import time
import os
import sys
import signal
import numpy
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib.ticker import FuncFormatter


def create_new_file():
    # Generates a unique filename based on the current timestamp
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"output_{timestamp}.txt"
    print(f"New file created: {os.path.join(folder_script, filename)}")          #print the name of the new file
    # Open and return the new file
    return open(os.path.join(folder_script, filename), 'w')

def signal_handler(signal, frame):
    print("\nStopped")
    ser.close()                                     #close port
    if save_data=='y':
    # Save data in txt file for RGPR
        csv_file = create_new_file()                #open a new file for the next data
        try:
            for r in range(len(data_for_save)):
                for c in range(len(data_for_save[r])):
                    if(r != 0 or c != len(data_for_save[r])-1):
                        csv_file.write(str(f"{(data_for_save[r][c]):.5f}"))
                    if(r == 0):
                        l=2
                    else:
                        l=1
                    if(c < (len(data_for_save[r])-l)):
                        csv_file.write(' ')
                csv_file.write('\n')            
        finally:
            csv_file.close()
    sys.exit(0)

def format_x_axis(val, pos):
    return f"{val * step - max_length:.1f}"

def format_y_axis(val, pos):
    return f"{(val - offset_cells) * resolution:.1f}"    
    
    
    

signal.signal(signal.SIGINT, signal_handler)
folder_script = os.path.dirname(os.path.abspath(__file__))

prt='COM14'                                         #serial virtual port created  by Arduino
baud='115200'                                       #serial port baud rate

debugging_mode = 0                                  #debugging 1/0

save_data = input("Save data on HDD? y/n: ")
if debugging_mode==0:
    velocity_index = int(input("Insert prop. media index 1-Air, 2-Ice, 3-Dry sand, 4-Dry soil (Granite), 5-Limestone, 6-Asphalt, 7-Concrete, 8-Moist soil,  9-Wet soil (Silt), 10-Saturated sand (Clay), 11-Sweet water, 12-Salt water: "))
else:
    velocity_index = 1
velocities = [300000000, 160000000, 150000000, 130000000, 120000000, 110000000, 100000000, 90000000, 70000000, 60000000, 30000000, 10000000]

max_value = 120                                     #maximum image intensity, used for normalizing the image, controls the brightness
max_length = 6                                      #maximum ploted horizontal length (window) in meters
max_depth = 7                                       #maximum ploted depth in meters
offset = 1.3                                        #cables + antenna_to_soil offset in meters
step = 0.12                                         #horizontal step in meters
cable_velocity = 198000000                          #electromagnetic wave velocity in cable and antennas
velocity = velocities[velocity_index-1]             #electromagnetic wave velocity in meters/second
interp_order_depth = 10                             #interpolation order on depth axis

no_of_samples = 256                                 #number of time domain samples from GPR
bandwidth = 539000000.0                             #GPR bandwidth in Hz; 355.7 MHz - 894.7 MHz
resolution = velocity / (2.0 * bandwidth) / interp_order_depth  #GPR resolution
offset_cells = int(offset / (cable_velocity / (2.0 * bandwidth)) * interp_order_depth)
image_rows = int(float(max_depth)/resolution + offset_cells)    #no of rows of the image
if image_rows > int(no_of_samples/2) * interp_order_depth + 1:
    image_rows = int(no_of_samples/2) * interp_order_depth + 1
image_columns = int(float(max_length)/step)         #no of columns of the image
image = numpy.zeros((image_rows, image_columns),dtype=numpy.uint8) #create blank image matrix
iteration = 1
if save_data=='y':
    data_for_save = numpy.zeros((int(no_of_samples/2) * interp_order_depth + 1, 1)) #create blank data matrix for saving
    for i in range(int(no_of_samples/2) * interp_order_depth):    
        data_for_save[i+1][0] = 1e9/interp_order_depth/bandwidth * i
if debugging_mode==1:
    plt.ion()       
    fig = plt.figure()
    ax = fig.add_subplot(111)
    graph, = ax.plot(numpy.zeros(int(no_of_samples)))
else:    
    fig, ax = plt.subplots()
    im = ax.imshow(image, cmap='gray', vmin=0, vmax=255, aspect='auto')
    ax.xaxis.set_major_formatter(FuncFormatter(format_x_axis))
    ax.yaxis.set_major_formatter(FuncFormatter(format_y_axis))
    ax.set_title("GPR Image")
    ax.set_xlabel("Position [m]")
    ax.set_ylabel("Depth [m]")
    plt.ion()
    plt.show(block=False)
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

ser = serial.Serial(port=prt, baudrate=baud)        #open serial virtual port created by Arduino
time.sleep(3) 
ser.write(b'S')                                     #write start command
time.sleep(1)

while True:
    if ser.in_waiting > 0:    
        s = ser.read(int(no_of_samples*2))          #read 512 bytes (256 time domain samples on 2 bytes each)
        dt = numpy.dtype('uint16')
        dt = dt.newbyteorder('>')                   #highbyte first
        s_array = numpy.frombuffer(s, dtype=dt)     #convert to array
        #print(s_array)                             #show the numbers
        
        if debugging_mode==1:
            graph.set_ydata(s_array)    
            plt.title("Time domain samples")
            plt.ylim(0, 1023)
            plt.xlabel("Sample index")
            fig.canvas.draw()
            fig.canvas.flush_events()
            s_array = s_array - numpy.mean(s_array)                                                         #remove d.c. component
            column = numpy.absolute(numpy.fft.rfft(s_array, int(no_of_samples) * interp_order_depth))       #compute fft magnitude
            if save_data=='y':
                column_conc=numpy.concatenate((numpy.array([iteration*step]), column), axis=0)
                data_for_save=numpy.hstack((data_for_save,numpy.transpose(numpy.array([column_conc[0:(int(no_of_samples/2) * interp_order_depth + 1)]]))))  #add as new last column            
        else:  
            s_array = s_array - numpy.mean(s_array)                                                         #remove d.c. component
            column = numpy.absolute(numpy.fft.rfft(s_array, int(no_of_samples) * interp_order_depth))       #compute fft magnitude
            if save_data=='y':
                column_conc=numpy.concatenate((numpy.array([iteration*step]), column), axis=0)
                data_for_save=numpy.hstack((data_for_save,numpy.transpose(numpy.array([column_conc[0:(int(no_of_samples/2) * interp_order_depth + 1)]]))))  #add as new last column
            for i in range(int(no_of_samples/2) * interp_order_depth + 1):
                if i < 60 * interp_order_depth:
                    column[i] = column[i] * numpy.exp(i * 0.025 / interp_order_depth)                       #depth correction
                else:
                    column[i] = column[i] * 4.482
            image=numpy.delete(image,0,1)           #remove first column
            #print(column[0:image_rows])            #show the numbers
            normalized_column = column / max_value  #normalize column
            image = numpy.hstack((image,numpy.transpose(numpy.array([normalized_column[0:image_rows]]))))   #add as new last column
            fig.canvas.restore_region(background)
            im.set_array(image)
            ax.draw_artist(im)
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()
            print(f"\rCurrent position: {iteration * step:.2f} m ", end="")
        iteration = iteration + 1
