import Tkinter
import tkMessageBox

global contador
contador = 1
while(1):
	top = Tkinter.Tk()

	def SaveFrame():
	   #tkMessageBox.showinfo( "Frame Salvo","Frame %d Salvo")
	   global contador
	   contador=contador+1;
	   print contador	
	   return contador
	B = Tkinter.Button(top, text ="Save", command = SaveFrame())

	B.pack()
	top.mainloop()
