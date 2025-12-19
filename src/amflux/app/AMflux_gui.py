import tkinter as tk
from tkinter import ttk
#import ttkbootstrap as ttk
#from main import OperationModes


#HELPERS

class ControlWindow:
    Home                     = 6
    ProfilePosition             = 0
    Homing                      = 1
    ProfileVelocity             = 2
    CyclicSynchronousPosition   = 3
    CyclicSynchronousVelocity   = 4
    CyclicSynchronousTorque     = 5


operation_modes = [
    "ProfilePosition", 
    "Homing", 
    "ProfileVelocity", 
    "CyclicSynchronousPosition", 
    "CyclicSynchronousVelocity", 
    "CyclicSynchronousTorque"
]

def button_func(window: ControlWindow):
    opt.get()
    if window
    landing_text_var.set("")



#WINDOW

window = tk.Tk()
window.title("EPOS4 Control Window")
window.geometry("800x500")
window.configure(bg="#363131") 

#WIDGETS

#label
landing_text_var = tk.StringVar()
landing_text_var.set("This is the EPOS4 Control Window\n Please Select an Operation Mode")

label = tk.Label(
    window, 
    textvariable=landing_text_var, 
    anchor=tk.CENTER,       
    bg="lightblue",      
    height=3,              
    width=30,              
    bd=3,                  
    font=("Helvetica", 16, "bold"),   
    fg="orange",             
    padx=15,               
    pady=15,                
    justify=tk.CENTER,    
    relief=tk.RAISED,           
    wraplength=250         
)
label.pack(pady=20)


#dropdown
opt = tk.StringVar()
opt.set("Operation Mode")
drop = ttk.OptionMenu(window, opt, *operation_modes)
drop.pack(side="top")


#button

button_text_var = tk.StringVar()
button_text_var.set("Start")

button = ttk.Button(
    window, 
    textvariable=button_text_var,
    command=button_func
)
button.pack(side="top")

window.mainloop()