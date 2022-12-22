from tkinter import *
import serial
import os
import pty
import threading
from threading import Semaphore
import time

#Esto es una prueba 4
master, slave= pty.openpty()
puertoe = os.ttyname(slave)
puerto=serial.Serial(puertoe,baudrate=9600,timeout=3)
print('puerto serial listo para la comunicacion')

root = Tk()
entrada_led = Entry(root)
entrada_led.grid(row=0, column=0)

entrada_ventilador = Entry(root)
entrada_ventilador.grid(row=2, column=0)

entrada_ventilador = Entry(root)
entrada_ventilador.grid(row=5, column=1)
#estado=StringVar() 
#l=StringVar()
v=StringVar()
l=IntVar()
r=IntVar()
g=IntVar()
b=IntVar()
x=1
estado=IntVar()
entrada=Entry(root)
semaphore = Semaphore(2)

#declaracion de hilos

def f():
	with semaphore:
		while 1:
			puerto.write(b'c')
			time.sleep(3)

def a():
	while 1:
		print (os.read(master,100))

#hilos
t = threading.Thread(target=f)
v = threading.Timer(4,a)
t.start(), v.start()

#funciones para botones

def encender_lampara():
	with semaphore:
		puerto.write("a".encode())

def apagar_lampara():
	with semaphore:
		puerto.write("b".encode())

def intensidad_luz():
	with semaphore:
		l = entrada_led.get()#Enviamos intensidad de luz
		puerto.write(l.encode())

def intensidad_r():
	with semaphore:
		r = entrada_led.get()#Enviamos intensidad de r
		puerto.write(r.encode())

def intensidad_g():
	with semaphore:
		g = entrada_led.get()#Enviamos intensidad de g
		puerto.write(g.encode())

def intensidad_b():
	with semaphore:
		b = entrada_led.get()#Enviamos intensidad de b
		puerto.write(b.encode())

def estado_chip():
		with semaphore:
			print (os.read(master,100))

def encender_ventilador():
	with semaphore:
		puerto.write("c".encode())

def apagar_ventilador():
	with semaphore:
		puerto.write("d".encode())

def intensidad_ventilador():
	with semaphore:
		v = entrada_ventilador.get()
		puerto.write(v.encode())
def off_system():
	with semaphore:
		puerto.close
		
		
#Botones para de LED
def click_boton():
	texto = Label(root,
				 ).grid(row=2,column=0)#f'Se almacenó "{entrada.get()}" correctamente').grid(row=2, column=0)
	encender_lampara()
#Enviar valores al chip 
def click_boton_2():
	texto = Label(root,
				 ).grid(row=2,column=0)
	apagar_lampara()

#Botones para el ventilador                 
def click_boton_3():
	texto = Label(root).grid(row=2,column=0)#f'Se almacenó "{entrada.get()}" correctamente').grid(row=2, column=0)
	encender_ventilador()

def click_boton_4():
	texto = Label(root,
				 ).grid(row=2,column=0)
	apagar_ventilador()



boton1 = Button(root,
				text="Encender",
				#bg="red",
				padx=50,
				pady=10,
				command=click_boton).grid(row=1, column=1)

boton2 = Button(root,
				text="Apagar",
				#bg="red",
				padx=50,
				pady=10,
				command=click_boton_2).grid(row=1, column=2)

boton3 = Button(root,
				text="Enviar intensidad",
				#bg="red",
				padx=50,
				pady=10,
				command=intensidad_luz).grid(row=1, column=0)
boton4 = Button(root,
				text="Encender_V",
				#bg="red",
				padx=50,
				pady=10,
				command=click_boton_3).grid(row=3, column=1)

boton5 = Button(root,
				text="Apagar_V",
				#bg="red",
				padx=50,
				pady=10,
				command=click_boton_4).grid(row=3, column=2)

boton6 = Button(root,
				text="Enviar intensidad V",
				#bg="red",
				padx=50,
				pady=10,
				command=intensidad_ventilador).grid(row=3, column=0)
boton7 = Button(root,
				text="Estado",
				#bg="red",
				padx=50,
				pady=10,
				command=estado_chip).grid(row=4, column=1)

boton8 = Button(root,
				text="Valor R",
				#bg="red",
				padx=50,
				pady=10,
				command=intensidad_r).grid(row=6, column=0)

boton9 = Button(root,
				text="Valor G",
				#bg="red",
				padx=50,
				pady=10,
				command=intensidad_g).grid(row=6, column=1)

boton10 = Button(root,
				text="Valor B",
				#bg="red",
				padx=50,
				pady=10,
				command=intensidad_b).grid(row=6, column=2)

boton11 = Button(root,
				text="Apagar Sistema",
				#bg="red",
				padx=50,
				pady=10,
				command=off_system).grid(row=7, column=1)
root.mainloop()
