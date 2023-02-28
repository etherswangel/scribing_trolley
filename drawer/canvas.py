import os
import tkinter as tk

from .control import Control

class Canvas(tk.Frame):

    def __init__(self, root, parent, *args, **kwargs):
        super().__init__(master=parent, *args, **kwargs)

        self.root = root
        self.scale = 20
        self.drawable = True
        self.trajactory = [[0, 0]]


    def set_picker(self, picker):
        self.picker = picker

        self.pack_canvas(self)
        self.pack_buttons(self)

        self.mode_edit()


    def init_canvas(self):
        w = int(self.canvas['width'])
        h = int(self.canvas['height'])
        ox = w * 0.5
        oy = h * 0.75

        self.origin = ox, oy
        self.last_coor = self.origin

        self.canvas.delete('all')
        self.canvas.create_oval(ox-4, oy-4, ox+4, oy+4, outline='red', fill='red')

        y, n = oy, 0
        while y < h:
            y += self.scale
            n += 1
        while y > 0:
            if n%5 == 0:
                self.canvas.create_line(0, y, w, y, width=2, fill='grey')
            else:
                self.canvas.create_line(0, y, w, y, fill='grey')
            y -= self.scale
            n -= 1

        x, n = ox, 0
        while x < w:
            x += self.scale
            n += 1
        while x > 0:
            if n%5 == 0:
                self.canvas.create_line(x, 0, x, h, width=2, fill='grey')
            else:
                self.canvas.create_line(x, 0, x, h, fill='grey')
            x -= self.scale
            n -= 1


    def confirm(self):
        ox, oy = self.origin
        lx, ly = self.last_coor

        if len(self.trajactory) == 1:
            print('empty trajectory')
            return

        if self.drawable and ox!=lx and oy!=ly:
            self.canvas.create_line(lx, ly, ox, oy, width=3)
            self.trajactory.append([0, 0])
            self.drawable = False

        self.mode_show(self.trajactory)


    def clear(self):
        self.drawable = True
        self.trajactory = [[0, 0]]

        self.init_canvas()


    def publish(self):
        path = Control(self.trajactory).run()
        self.draw(path, width=2, smooth=True, fill='orange')


    def save_file(self, path=os.path.join(os.getcwd(), 'paths')):
        file_name = '新建路径'
        def popup(text):
            def get_text():
                nonlocal top, text, txt
                text = txt.get()
                if text != '':
                    top.destroy()

            top = tk.Toplevel(master=self,
                width=200, height=150,
            )

            tk.Label(master=top,
                text='路径名: '
            ).pack(
                side='left',
            )

            txt = tk.Entry(master=top,
                width=20,
            )
            txt.insert(0, text)
            txt.pack(
                side='left',
            )

            tk.Button(master=top,
                text='确定',
                command=get_text
            ).pack(
                side='left',
            )

            self.wait_window(top)
            return text

        file_name = popup(file_name)

        file = os.path.join(path, file_name + '.txt')
        with open(file, 'w') as f:
            f.write(file_name + '\n')
            for t in self.trajactory:
                f.write(str(t[0]) + ' ' + str(t[1]) + '\n')

        self.picker.refresh()


    def mode_show(self, trajectory):
        self.drawable = False
        self.trajactory = trajectory

        self.init_canvas()
        self.draw(trajectory)

        self.btn2['text'] = '保存'
        self.btn2['command'] = self.save_file
        self.btn1['text'] = '确定'
        self.btn1['command'] = self.publish


    def mode_edit(self):
        self.clear()

        self.btn2['text'] = '确定'
        self.btn2['command'] = self.confirm
        self.btn1['text'] = '清空'
        self.btn1['command'] = self.clear


    def draw(self, trajectory, width=3, smooth=False, fill='black'):
        if not trajectory:
            return

        path = []
        for t in trajectory:
            path.append(self.pose_to_pix(t))

        self.canvas.create_line(path, width=width, smooth=smooth, fill=fill)


    def zoom(self, dir):
        if dir == 'in':
            self.scale += 5
        else:
            self.scale -= 5
        if self.drawable:
            self.mode_edit()
        else:
            self.mode_show(self.trajactory)


    def pack_canvas(self, parent):

        def paint(e):
            if not self.drawable:
                return

            x, y = e.x, e.y
            lx, ly = self.last_coor

            if not (abs(x-lx) > 2 or abs(y-ly) > 2):
                return

            self.canvas.create_line(lx, ly, x, y, width=3)

            self.last_coor = x, y
            self.trajactory.append(self.pix_to_pose([x, y]))

        self.canvas = tk.Canvas(master=parent,
            width=500, height=400,
            bg='white',
        )
        self.canvas.bind('<Button-1>', paint)
        self.canvas.bind('<B1-Motion>', paint)
        self.canvas.pack(
            side='top',
            expand=True, fill='both',
        )


    def pack_buttons(self, parent):

        def add_button():
            frm = tk.Frame(master=frm_buttons,
                width=75,
            )
            frm.pack_propagate(False)
            frm.pack(
                side='right',
                expand=False, fill='y',
                padx=9, pady=7,
            )

            btn = tk.Button(master=frm,
            )
            btn.pack(
                expand=True, fill='both',
            )

            return btn

        frm_buttons = tk.Frame(master=parent,
            height=50,
        )
        frm_buttons.pack_propagate(False)
        frm_buttons.pack(
            side='bottom',
            expand=False, fill='x',
            padx=7,
        )

        self.btn1 = add_button()
        self.btn2 = add_button()

        btn_zoom_in = add_button()
        btn_zoom_in['text'] = '放大'
        btn_zoom_in['command'] = lambda: self.zoom('in')
        btn_zoom_out = add_button()
        btn_zoom_out['text'] = '缩小'
        btn_zoom_out['command'] = lambda: self.zoom('out')


    def pose_to_pix(self, pose):
        ox, oy = self.origin
        x = -pose[1]*self.scale + ox
        y = -pose[0]*self.scale + oy

        return [x, y]


    def pix_to_pose(self, pix):
        ox, oy = self.origin
        x = -(pix[1]-oy) / self.scale
        y = -(pix[0]-ox) / self.scale

        return [x, y]


