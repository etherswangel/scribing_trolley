import tkinter as tk
import tkinter.ttk as ttk

from .canvas import Canvas

class Drawer(tk.Frame):

    def __init__(self, root, *args, **kwargs):
        super().__init__(master=root, *args, **kwargs)

        self.root = root

    def run(self):
        Canvas(self).pack(
            expand=True, fill='both',
        )

        self.pack(
            expand=True, fill='both',
        )
        self.root.mainloop()
