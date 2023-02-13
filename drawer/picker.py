import tkinter as tk
import tkinter.ttk as ttk

class Picker(tk.Frame):

    def __init__(self, parent, *args, **kwargs):
        super().__init__(master=parent, *args, **kwargs)

        self.pack_box(self)

    def pack_box(self, parent):
        pass
