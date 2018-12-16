import msgpack
import UDPComms
import tkinter as tk
import math
import time


WINDOW_SIDE = 1000
MM_PER_PIX = 4

MM_PER_INCH =2.54


class Map:

    def __init__(self,map_topic, world_frame):
        self.x_size_m = 40
        self.y_size_m = 40
        self.resolution = 0.05

        self.x_size_g = int(self.x_size_m/self.resolution)
        self.y_size_g = int(self.y_size_m/self.resolution)

        self.map = [ float("nan") ] * self.y_size_g*self.x_size_g

        # self.map_pub = UDPComms.Publisher(8888)

    def __getitem__(self, key):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        return self.map[ y * self.x_size_g + x ]

    def __setitem__(self, key, value):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        self.map[ y * self.x_size_g + x ] = value

    def get_raw(self, x , y):
        return self.map[ y * self.x_size_g + x ]

    def set_raw(self, x , y, value):
        self.map[ y * self.x_size_g + x ] = value

    def publish_map(self):
        pass
        

class Robot:
    def __init__(self, l, r):

        # self.sub = subscriber

        # self.th measured CCW from X axis
        self.x = 0
        self.y = 0
        self.th = math.pi/2

        self.WHEEL_RAD = MM_PER_INCH * 6.5/2 #mm
        self.WHEEL_BASE = MM_PER_INCH * 18  #mm
        self.wheel_l, self.wheel_r = l, r #rotations

        self.lidar_offset_forward = MM_PER_INCH * 18

    def get_pose(self):
        return (self.x, self.y, self.th)

    def convert_lidar_to_map(self, angle, dist):
        return (self.x + dist * math.cos(angle + self.th) + self.lidar_offset_forward * math.cos(self.th), 
                self.y + dist * math.sin(angle + self.th) + self.lidar_offset_forward * math.sin(self.th))

    def update(self, l ,r):
        # import pdb; pdb.set_trace()
        # l, r = self.sub.get()

        # http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html

        dist_l = (l - self.wheel_l) * 2 * math.pi * self.WHEEL_RAD
        dist_r = (r - self.wheel_r) * 2 * math.pi * self.WHEEL_RAD

        self.wheel_l = l
        self.wheel_r = r

        v_right   = 0
        v_forward = (dist_l + dist_r) / 2
        v_th      = (dist_r - dist_l) / self.WHEEL_BASE

        delta_x = (v_forward * math.cos(self.th + v_th/2) ) # - v_right * math.sin(th)) # needed fro omni robots
        delta_y = (v_forward * math.sin(self.th + v_th/2) ) # + v_right * math.cos(th))

        self.x += delta_x;
        self.y += delta_y;
        self.th += v_th;

        print( self.x, self.y, self.th)



sub = UDPComms.Subscriber("data",  "4096s", 8110, 1)
odom = UDPComms.Subscriber("l r",  "ff", 8820, timeout = 2)
rob = Robot( *odom.get() )


class LidarWindow:
    def __init__(self):
        print (odom.timeout)

        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root,width=WINDOW_SIDE,height=WINDOW_SIDE)
        self.canvas.pack()


        arrow_length = 10
        arrow = canvas.create_line(0, 0, SIDE, SIDE, arrow=tk.LAST)

        time.sleep(2)

        self.root.after(100,update)
        self.root.mainloop()


    def create_point(self,x,y):
        self.canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

    def convert_map_to_cavnas(self,in_x, in_y):
        y= SIDE/2 - in_x / WINDOW_SIDE
        x= SIDE/2 + in_y / WINDOW_SIDE
        return (x,y)

def update():
    global arrow
    try:


        rob.update(  *odom.get())

        unpacker = msgpack.Unpacker()
        unpacker.feed(sub.get().data)
        data = unpacker.unpack()
        

        rob_x, rob_y = convert_map_to_cavnas(rob.x * MM_PER_INCH, rob.y * MM_PER_INCH)

        canvas.delete(arrow)
        arrow = canvas.create_line(rob_x, rob_y, rob_x + arrow_length*math.sin(rob.th),
                                         rob_y - arrow_length*math.cos(rob.th), arrow=tk.LAST)
        
        # canvas.delete('all')
        # canvas.create_oval(255, 255, 245, 245, fill = '#FF0000')


        print("Robot loc: ", rob_x, rob_y)

        for _, angle, dist in data:
            a = math.radians(angle)


            point_x, point_y = convert_lidar_to_map( rob.x * MM_PER_INCH , rob.y * MM_PER_INCH, rob.th, a, dist/ SCALE )
            # print(angle,dist, point_x, point_y)
            create_point( *convert_map_to_cavnas(point_x, point_y ) )
        print()


    finally:
        r.after(100,update)





