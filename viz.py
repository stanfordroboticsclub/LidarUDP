import msgpack
import UDPComms
import tkinter as tk
import math


sub = UDPComms.Subscriber("data",  "4096s", 8110)

r = tk.Tk()
canvas = tk.Canvas(r,width=500,height=500)
canvas.pack()




def create_point(x,y):
    canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

def update():
    unpacker = msgpack.Unpacker()
    unpacker.feed(sub.get().data)
    data = unpacker.unpack()

    canvas.delete('all')
    canvas.create_oval(250, 250, 250, 250, width = 10, fill = '#FF0000')
    for _, angle, dist in data:
        print(angle,dist)
        a = math.radians(angle)
        create_point(math.sin(a) * dist/10 +250, math.cos(a) * dist/10 +250)

    print()
    r.after(100,update)


r.after(100,update)
r.mainloop()


