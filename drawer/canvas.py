import tkinter as tk
import tkinter.ttk as ttk

class Canvas(tk.Frame):

    def __init__(self, parent, *args, **kwargs):
        super().__init__(master=parent, *args, **kwargs)

        self.origin = None
        self.last_coor = None
        self.trajactory = []
        self.drawable = True
        self.scale = 75

        self.pack_canvas(self)
        self.pack_buttons(self)


    def confirm(self):
        if not self.drawable:
            return

        ox, oy = self.origin if self.origin else (None, None)
        lx, ly = self.last_coor if self.last_coor else (None, None)

        if ox and oy and lx and ly and ox!=lx and oy!=ly:
            self.canvas.create_line(lx, ly, ox, oy, width=3, smooth=True)
            self.trajactory.append([ox, oy])
            self.drawable = False
            print(self.trajactory)


    def clear(self):
        self.canvas.delete('all')
        self.origin = None
        self.last_coor = None
        self.trajactory = []
        self.drawable = True


    def pack_canvas(self, parent):

        def paint(e):
            if not self.drawable:
                return

            x, y = e.x, e.y
            lx, ly = self.last_coor if self.last_coor else (None, None)

            if not lx or not ly:
                canvas.create_oval(x-4, y-4, x+4, y+4, outline='red', fill='red')
                self.origin = x, y
            elif abs(x-lx) > 2 or abs(y-ly) > 2:
                canvas.create_line(lx, ly, x, y, width=3, smooth=True)
            else:
                return

            self.last_coor = x, y
            self.trajactory.append([x, y])

        def grid():
            pass

        canvas = tk.Canvas(master=parent,
            width=500, height=400,
            bg='white',
        )
        canvas.bind('<Button-1>', paint)
        canvas.bind('<B1-Motion>', paint)
        canvas.pack(
            side='top',
            expand=True, fill='both',
        )

        self.canvas = canvas


    def pack_buttons(self, parent):

        def add_button(name, func):
            frm = tk.Frame(master=frm_buttons,
                width=75,
            )
            frm.pack_propagate(False)
            frm.pack(
                side='right',
                expand=False, fill='y',
                padx=9, pady=7,
            )

            tk.Button(master=frm,
                text=name,
                command=func
            ).pack(
                expand=True, fill='both',
            )

        frm_buttons = tk.Frame(master=parent,
            height=50,
        )
        frm_buttons.pack_propagate(False)
        frm_buttons.pack(
            side='bottom',
            expand=False, fill='x',
            padx=7,
        )

        add_button('清空', self.clear)
        add_button('确定', self.confirm)

