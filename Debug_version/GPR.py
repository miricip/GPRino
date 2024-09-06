#GPR.py

from __future__ import print_function
import serial
import time
import sys
import signal
import numpy
import matplotlib.pyplot as plt


def signal_handler(signal, frame):
    print("\nStopped")
    ser.close()                                   #close port
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

plt.ion()

debugging_mode = 0                                #debugging 1/0

max_length = 6                                    #maximum ploted horizontal length in meters
max_depth = 5                                     #maximum ploted depth in meters
offset = 1.2                                      #cables + antenna_to_soil offset in meters
step = 0.12                                       #horizontal step in meters
velocity = 150000000.0                            #electromagnetic wave velocity in asphalt in meters/second

bandwidth = 587000000.0                                 #GPR bandwidth in Hz; 910 MHz - 323 MHz
resolution = velocity / (2.0 * bandwidth)               #GPR resolution
image_rows = int(float(max_depth+offset)/resolution)    #no of rows of the image
image_columns = int(float(max_length)/step)             #no of columns of the image
image = numpy.zeros((image_rows, image_columns))        #create blank image matrix
fig = plt.figure()
ax = fig.add_subplot(111)
if debugging_mode==1:
    graph, = ax.plot(numpy.zeros(256))


ser = serial.Serial(port='COM3', baudrate=115200) #open serial virtual port created by Arduino
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
            for i in range(0, len(column)):
                column[i] = column[i] * numpy.exp(i * 0.025)        #depth correction
            image=numpy.delete(image,0,1)                           #remove first column
            #print(column[0:image_rows])                             #show the numbers
            image = numpy.hstack((image,numpy.transpose(numpy.array([column[0:image_rows]]))))  #add as new last column
            ax.matshow(image, extent=[-image_columns*step,0,image_rows*resolution-offset,-offset], origin='upper', aspect='auto', cmap='gray') #plot image
            plt.title("GPR Image")                                                                                   #plot image
            plt.xlabel("Position [m]")                                                                               #plot image
            plt.ylabel("Depth [m]")                                                                                  #plot image
            fig.canvas.draw()                                                                                        #plot image
            fig.canvas.flush_events()                                                                                #plot image    
