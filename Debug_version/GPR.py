#GPR.py

from __future__ import print_function
import serial
import time
import sys
import signal
import numpy
import matplotlib.pyplot as plt

def create_new_file():
    # Generates a unique filename based on the current timestamp
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"output_{timestamp}.txt"
    print(f"New file created: {filename}")  #print the name of the new file
    # Open and return the new file
    return open(filename, 'w')

def signal_handler(signal, frame):
    print("\nStopped")
    ser.close()                                   #close port
    if save_data=='y':
    # Save data in txt file for RGPR
        csv_file = create_new_file()              #open a new file for the next data
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

signal.signal(signal.SIGINT, signal_handler)

plt.ion()

prt='COM14'                                       #serial virtual port created  by Arduino

save_data = input("Save data on HDD? y/n: ")
velocity_index = int(input("Insert prop. media index 1-Air, 2-Ice, 3-Dry sand, 4-Dry soil (Granite), 5-Limestone, 6-Asphalt, 7-Concrete, 8-Moist soil,  9-Wet soil (Silt), 10-Saturated sand (Clay), 11-Sweet water, 12-Salt water: "))
velocities = [300000000, 160000000, 150000000, 130000000, 120000000, 110000000, 100000000, 90000000, 70000000, 60000000, 30000000, 10000000]


debugging_mode = 0                                #debugging 1/0

max_length = 6                                    #maximum ploted horizontal length (window) in meters
max_depth = 5                                     #maximum ploted depth in meters
offset = 1.3                                      #cables + antenna_to_soil offset in meters
step = 0.12                                       #horizontal step in meters
cable_velocity = 198000000                        #electromagnetic wave velocity in cable and antennas
velocity = velocities[velocity_index-1]           #electromagnetic wave velocity in meters/second

bandwidth = 539000000.0                                 #GPR bandwidth in Hz; 355.7 MHz - 894.7 MHz
resolution = velocity / (2.0 * bandwidth)               #GPR resolution
offset_cells = int(offset / (cable_velocity / (2.0 * bandwidth)))
image_rows = int(float(max_depth)/resolution + offset_cells) #no of rows of the image
if image_rows > 129:
    image_rows = 129
image_columns = int(float(max_length)/step)             #no of columns of the image
image = numpy.zeros((image_rows, image_columns))        #create blank image matrix
iteration = 1
if save_data=='y':
    data_for_save = numpy.zeros((129, 1))               #create blank data matrix for saving
    for i in range(128):    
        data_for_save[i+1][0] = 1e9/bandwidth * i
fig = plt.figure()
ax = fig.add_subplot(111)
if debugging_mode==1:
    graph, = ax.plot(numpy.zeros(256))


ser = serial.Serial(port=prt, baudrate=115200)    #open serial virtual port created by Arduino
time.sleep(3) 
ser.write(b'S')                                   #write start command
time.sleep(1)

while True:
    if ser.in_waiting > 0:    
        s = ser.read(512)                                       #read 512 bytes (256 time domain samples on 2 bytes each)
        dt = numpy.dtype('uint16')
        dt = dt.newbyteorder('>')                               #highbyte first
        s_array = numpy.frombuffer(s, dtype=dt)                 #convert to array
        #print(s_array)                                          #show the numbers
        
        if debugging_mode==1:
            graph.set_ydata(s_array)    
            plt.title("Time domain samples")
            plt.ylim(0, 1023)
            plt.xlabel("Sample index")
            fig.canvas.draw()
            fig.canvas.flush_events()
            
        else:    
            column = numpy.absolute(numpy.fft.rfft(s_array))        #compute fft magnitude
            column[0] = 0.0                                         #remove d.c. component
            if save_data=='y':
                column_conc=numpy.concatenate((numpy.array([iteration*step]), column), axis=0)
                data_for_save=numpy.hstack((data_for_save,numpy.transpose(numpy.array([column_conc[0:129]]))))  #add as new last column
            for i in range(129):
                if i < 60:
                    column[i] = column[i] * numpy.exp(i * 0.025)    #depth correction
                else:
                    column[i] = column[i] * 4.482
            image=numpy.delete(image,0,1)                           #remove first column
            #print(column[0:image_rows])                             #show the numbers
            image = numpy.hstack((image,numpy.transpose(numpy.array([column[0:image_rows]]))))  #add as new last column
            ax.matshow(image, extent=[(iteration-image_columns)*step,iteration*step,(image_rows-offset_cells)*resolution,-offset_cells*resolution], origin='upper', aspect='auto', cmap='gray') #plot image
            plt.title("GPR Image")                                                                                   #plot image
            plt.xlabel("Position [m]")                                                                               #plot image
            plt.ylabel("Depth [m]")                                                                                  #plot image
            fig.canvas.draw()                                                                                        #plot image
            fig.canvas.flush_events()                                                                                #plot image   
        iteration = iteration + 1