import msgpack
import UDPComms
import tkinter as tk
import math
import time


SIDE = 1000

sub = UDPComms.Subscriber("data",  "4096s", 8110, 1)
odom = UDPComms.Subscriber("l r",  "ff", 8820, timeout = 2)
print (odom.timeout)

r = tk.Tk()
canvas = tk.Canvas(r,width=SIDE,height=SIDE)
canvas.pack()

time.sleep(2)

print("getting")
print( odom.get() )

class Robot:

    def __init__(self, l, r):

        # self.sub = subscriber

        self.x = 0
        self.y = 0
        self.th = 0

        self.WHEEL_RAD = 6.5/2 # inch
        self.WHEEL_BASE = 18 # inch
        self.wheel_l, self.wheel_r = l, r

    def update(self, l ,r):
        # import pdb; pdb.set_trace()
        # l, r = self.sub.get()

        # http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html

        dist_l = (l - self.wheel_l) * 2 * math.pi * self.WHEEL_RAD
        dist_r = (r - self.wheel_r) * 2 * math.pi * self.WHEEL_RAD

        self.wheel_l = l
        self.wheel_r = r

        vx =   (dist_l + dist_r) / 2
        vth =  (dist_r - dist_l) / self.WHEEL_BASE

        vy = 0
        delta_x = (vx * math.cos(self.th + vth/2) ) # - vy * math.sin(th))
        delta_y = (vx * math.sin(self.th + vth/2) ) # + vy * math.cos(th))

        self.x += delta_x;
        self.y += delta_y;
        self.th += vth;

        print( self.x, self.y, self.th)

rob = Robot( *odom.get() )

def create_point(x,y):
    canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')


arrow_length = 10

arrow = canvas.create_line(0, 0, SIDE, SIDE, arrow=tk.LAST)


def convert_lidar_to_map(rob_x, rob_y, rob_th, angle, dist):
    return( rob_x + dist * math.cos( angle + rob_th) + 18 * 2.54, rob_y + dist * math.sin( angle + rob_th) )


def convert_map_to_cavnas(in_x, in_y):
    y= SIDE/2 - in_x / SCALE
    x= SIDE/2 + in_y / SCALE
    return (x,y)


SCALE = 4

INCH_to_mm =2.54
INCH_to_mm =2.5
INCH_to_mm =3.5
INCH_to_mm =5.08

def update():
    global arrow
    try:


        rob.update(  *odom.get())

        unpacker = msgpack.Unpacker()
        unpacker.feed(sub.get().data)
        data = unpacker.unpack()
        

        rob_x, rob_y = convert_map_to_cavnas(rob.x * INCH_to_mm, rob.y * INCH_to_mm)

        canvas.delete(arrow)
        arrow = canvas.create_line(rob_x, rob_y, rob_x + arrow_length*math.sin(rob.th),
                                         rob_y - arrow_length*math.cos(rob.th), arrow=tk.LAST)
        
        # canvas.delete('all')
        # canvas.create_oval(255, 255, 245, 245, fill = '#FF0000')


        print("Robot loc: ", rob_x, rob_y)

        for _, angle, dist in data:
            a = math.radians(angle)


            point_x, point_y = convert_lidar_to_map( rob.x * INCH_to_mm , rob.y * INCH_to_mm, rob.th, a, dist/ SCALE )
            # print(angle,dist, point_x, point_y)
            create_point( *convert_map_to_cavnas(point_x, point_y ) )
        print()


    finally:
        r.after(100,update)





r.after(100,update)
r.mainloop()


