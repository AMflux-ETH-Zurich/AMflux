import tkinter as tk
import ttkbootstrap as ttk


window = tk.Tk()
window.title("EPOS4 Control Window")
window.geometry("800x500")

label = ttk.Label(master = window, text = "This is the EPOS4 Control Window", font=('Helvetica', 12))
label.pack()


window.mainloop()