from tkinter import *
import serial
import os
import pty

#Esto es una prueba 4
master, slave= pty.openpty()
puertoe = os.ttyname(slave)
Sere = serial.Serial('/dev/ttyUSB0',baudrate=9600,timeout=3)
print(Sere.name)
print(Sere.read(10))




