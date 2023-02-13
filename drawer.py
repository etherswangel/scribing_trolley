import tkinter as tk

from drawer import Drawer

if __name__ == '__main__':
    root = tk.Tk()
    root.bind('<Escape>',
        lambda _: root.destroy()
    )

    Drawer(root).run()

