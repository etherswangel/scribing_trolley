import tkinter as tk
import tkinter.ttk as ttk

from .canvas import Canvas
from .picker import Picker

class Drawer(tk.Frame):

    def __init__(self, root, *args, **kwargs):
        super().__init__(master=root, *args, **kwargs)

        self.root = root

    def run(self, path):
        c = Canvas(self)
        c.pack(
            side='right',
            expand=True, fill='both',
        )

        Picker(self, c, path).pack(
            side='left',
            expand=False, fill='both',
        )

        self.pack(
            expand=True, fill='both',
        )
        self.root.mainloop()
