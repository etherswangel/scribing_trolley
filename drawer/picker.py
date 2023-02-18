import os
from functools import partial
import tkinter as tk
import tkinter.ttk as ttk

class Picker(tk.Frame):

    def __init__(self, parent, canvas, path, *args, **kwargs):
        super().__init__(master=parent, *args, **kwargs)

        self.canvas = canvas
        self.path = path
        self.names = []

        self.read_files()
        self.pack_picker(self)


    def read_files(self):
        files = [f for f in os.listdir(self.path) if os.path.isfile(os.path.join(self.path, f))]
        for file in files:
            with open(os.path.join(self.path, file), 'r') as f:
                name = f.readline().strip()

            self.names.append(name)


    def show_trajectory(self, name):
        with open(os.path.join(self.path, name+'.txt'), 'r') as f:
            f.readline()
            coor = f.readlines()
        trajectory = [[float(n) for n in c.strip().split()] for c in coor]

        print('read')
        self.canvas.mode_show(trajectory)


    def pack_picker(self, parent):
        frm_picker = tk.Frame(master=parent,
            width=150,
        )
        frm_picker.pack_propagate(False)
        frm_picker.pack(
            side='left',
            expand=True, fill='y',
        )

        self.pack_box(frm_picker, name='-- 绘制 --')
        for i in range(len(self.names)):
            self.pack_box(frm_picker, idx=i)


    def pack_box(self, parent, idx=None, name=''):
        if not idx == None:
            name = self.names[idx]
            func = partial(self.show_trajectory, name)
        else:
            func = self.canvas.mode_edit

        frm = tk.Frame(master=parent,
        )
        frm.pack(
            side='top',
            expand=False, fill='x',
            padx=10, pady=10,
        )
        ttk.Button(master=frm,
            text=name,
            command=func
        ).pack(
            expand=True, fill='both',
        )

