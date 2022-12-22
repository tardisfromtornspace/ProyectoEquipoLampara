from tkinter import *
import serial
import os
import pty

#Esto es una prueba 4
master, slave= pty.openpty()
puertoe = os.ttyname(slave)
Sere = serial.Serial(puertoe,timeout=3,baudrate=9600)
print(Sere.name)
Sere.flush()
Sere.write(b'hola maestro')
print(os.read(master,100))
Sere.write(b'hola siuuu')
print(os.read(master,100))



