import msgpack
import UDPComms
import tkinter as tk
import math
import time


RATE = 100
WINDOW_SIDE = 1000
MM_PER_PIX = 4

MM_PER_INCH =2.54


class Map:

    def __init__(self,map_topic, world_frame):
        self.x_size_m = 40 # chagne to mm?
        self.y_size_m = 40
        self.resolution = 0.05

        self.x_size_g = int(self.x_size_m/self.resolution)
        self.y_size_g = int(self.y_size_m/self.resolution)

        self.map = [ [float("nan")]*self.y_size_g for _ in range(self.x_size_g)] 

        # self.map_pub = UDPComms.Publisher(8888)

    def up(self,key):
        pass

    def down(self, key):
        x = int((key[0] + self.x_size_m/2)/self.resolution)

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

    def interpolate(self):
        pass

    def interpolate_derivative(self):
        pass

    def scan_match(self):
        pass
        

class Robot:
    def __init__(self, l, r):

        self.odom = UDPComms.Subscriber("l r",  "ff", 8820, timeout = 2)

        # self.th measured CCW from X axis
        self.x = 0
        self.y = 0
        self.th = math.pi/2

        self.WHEEL_RAD = MM_PER_INCH * 6.5/2 #mm
        self.WHEEL_BASE = MM_PER_INCH * 18  #mm
        self.wheel_l, self.wheel_r = self.odom.recv() #rotations

        self.lidar_offset_forward = MM_PER_INCH * 18

    def get_pose(self):
        return (self.x, self.y, self.th)

    def set_pose(self, x, y, th):
        self.x  = x
        self.y  = y
        self.th = th

    def lidar_to_map(self, angle, dist):
        return (self.x + dist * math.cos(angle + self.th) + self.lidar_offset_forward * math.cos(self.th), 
                self.y + dist * math.sin(angle + self.th) + self.lidar_offset_forward * math.sin(self.th))

    def update_odom(self):
        # import pdb; pdb.set_trace()
        l, r = self.odom.get()

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

        print(self.x, self.y, self.th)



sub = UDPComms.Subscriber("data",  "4096s", 8110, 1)

rob = Robot( *odom.get() )

class LidarWindow:
    def __init__(self):
        print (odom.timeout)

        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root,width=WINDOW_SIDE,height=WINDOW_SIDE)
        self.canvas.pack()

        self.arrow = canvas.create_line(0, 0, SIDE, SIDE, arrow=tk.LAST)

        self.root.after(RATE,update)
        self.root.mainloop()

    def create_point(self,x,y):
        return self.canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

    def create_pose(self,x,y, th):
        return canvas.create_line(x, y, x + 10*math.sin(th),
                                        y - 10*math.cos(th), arrow=tk.LAST)

    def to_canvas(self,x, y):
        y= WINDOW_SIDE/2 - x / MM_PER_PIX
        x= WINDOW_SIDE/2 + y / MM_PER_PIX
        return (x,y)

    def update(self):
        try:

            rob.update_odom()
            pose = sdf.scan_match()
            rob.set_pose(*pose)

            unpacker = msgpack.Unpacker()
            unpacker.feed(sub.get().data)
            data = unpacker.unpack()

            rob_x, rob_y = self.to_canvas(rob.x, rob.y)

            canvas.delete(self.arrow)
            self.arrow = canvas.pose(*self.to_canvas(rob.x, rob.y), rob.th )
            
            # canvas.delete('all')

            for _, angle, dist in data:
                a = math.radians(angle)

                point_x, point_y = rob.lidar_to_map(a, dist)
                create_point( *self.to_canvas(point_x, point_y ) )
            print()


        finally:
            r.after(RATE,update)





