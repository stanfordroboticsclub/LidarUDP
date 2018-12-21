#!/usr/bin/env python3

import math
import time
import tkinter as tk

import numpy as np

import msgpack
import UDPComms


MM_PER_INCH =2.54

WINDOW_SIDE = 1000
MM_PER_PIX = 4

RESOLUTION_MM = 500
MAP_SIZE_M = 4
RATE = 100


class Line:
    def __init__(self, a, b, c):
        # ax + by + c = 0
        self.a = a
        self.b = b
        self.c = c

    @classmethod
    def from_mc(cls, m, c):
        y = mx + c
        return cls(m, -1, c)

    def get_distance(self,point):
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        x, y = point
        return math.fabs(self.a * x + self.b * y + self.c) \
               /math.sqrt(self.a**2 + self.b**2)

    @classmethod
    def from_fit(cls, points):
        A = np.array(points)
        y = A[:,1].copy()
        A[:,1] = 1

        # mx + c = y
        m,c = np.linalg.pinv(A) @ y
        return cls.from_mc(m,c)

    @classmethod
    def from_points(cls, p1, p2):
        x1, y1 = p1
        x2, y2 = p2

        a = y2 - y1
        b = x1 - x2

        c = x2 * y1 - y2 * x1
        return cls(a,b,c)



class SDFMap:

    SW = [ self.dw, self.dw ]
    NW = [ self.dw, self.up ]
    SE = [ self.up, self.dw ]
    NE = [ self.up, self.up ]

    def __init__(self):
        self.size_mm = MAP_SIZE_M * 1000
        self.resolution = RESOLUTION_MM

        self.size_g = int(self.size_mm/self.resolution)

        # TODO: figure out what defaut works
        self.map = [ [0]*self.size_g for _ in range(self.size_g)] 

        # self.map_pub = UDPComms.Publisher(8888)

    def up(self, idx):
        "ceiling of index"
        return int((idx + self.size_mm/2)/self.resolution) + 1

    def dw(self, idx):
        "floor of index"
        return int((idx + self.size_mm/2)/self.resolution)

    def fc(self, idx):
        "fractional part of index"
        return (idx + self.size_mm/2) % self.resolution

    def __getitem__(self, key):
        x = int((key[0] + self.size_mm/2)/self.resolution)
        y = int((key[1] + self.size_mm/2)/self.resolution)
        return self.map[ y * self.size_g + x ]

    def __setitem__(self, key, value):
        x = int((key[0] + self.size_mm/2)/self.resolution)
        y = int((key[1] + self.size_mm/2)/self.resolution)
        self.map[ y * self.size_g + x ] = value


    def publish_map(self):
        pass


    def interpolate(self, x, y):
        bl = self.map[self.dw(x)][self.dw(y)]
        br = self.map[self.up(x)][self.dw(y)]

        tl = self.map[self.dw(x)][self.up(y)]
        tr = self.map[self.up(x)][self.up(y)]

        M =      self.fc(y)  * (self.fc(x)* tr + (1 - self.fc(x))* tl ) +  \
            (1 - self.fc(y)) * (self.fc(x)* br + (1 - self.fc(x))* bl )
        
        return M

    def interpolate_derivative(self, x, y):
        bl = self.map[self.dw(x)][self.dw(y)]
        br = self.map[self.up(x)][self.dw(y)]

        tl = self.map[self.dw(x)][self.up(y)]
        tr = self.map[self.up(x)][self.up(y)]

        # if np.sign(bl) == np.sign(br) == np.sign(tl) == np.sign(tr):
        dx = self.fc(y)* (br - bl) + (1 - self.fc(y))* (tr - tl)  
        dy = self.fc(x)* (tl - bl) + (1 - self.fc(x))* (tr - br)  
        return (dx,dy)
        

class Robot:
    def __init__(self):

        self.odom = UDPComms.Subscriber(8820, timeout = 2)

        # self.th measured CCW from X axis
        self.x = 0
        self.y = 0
        self.th = math.pi/2

        self.WHEEL_RAD = MM_PER_INCH * 6.5/2 #mm
        self.WHEEL_BASE = MM_PER_INCH * 18  #mm
        # self.wheel_l, self.wheel_r = self.odom.recv() #rotations

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
        return
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

class SLAM:
    def __init__(self):
        self.lidar = UDPComms.Subscriber(8110, 1)
        self.robot = Robot()
        self.sdf = SDFMap()

    def update(self):
        scan = self.lidar.get()

        # self.robot.update_odom()

        # for _ in range(5):
        #     pose = self.sdf.scan_match(scan)
        #     self.robot.set_pose(*pose)
        self.update_sdf(scan)

    def get_map(self):
        return self.sdf.map

    def get_pose(self):
        return self.robot.get_pose()

    def get_lidar(self):
        # TODO: change to stored scan?
        for _, angle, dist in self.lidar.get():
            a = math.radians(angle)
            yield self.robot.lidar_to_map(a, dist)


    def update_sdf(self, scan):
        """ first janky update rule """

        points = {}
        for x,y in self.get_lidar():
            ind_x = self.sdf.dw(x)
            ind_y = self.sdf.dw(y)
            points[ (ind_x,ind_y) ] = points.get( (ind_x,ind_y) , []) + [(x,y)]


        print(points)
        for key, values in points.items():
            if len(values) < 2:
                continue

            line = Line.from_fit(values)

            self.sdf.map[ self.sdf.dw(key[0]) ][ self.sdf.dw(key[1]) ] = \
                    line.get_distance( ( self.sdf.dw(key[0]) , self.sdf.dw(key[1]) )
            


    def scan_match(self):
        Map_derivate = np.zeros((3))
        current_M = 0
        for angle,dist in scan:
            a = math.rad(angle)
            th = a + self.robot.th

            # d (x,y)/ d(rob_x, rob_y, rob_th)
            dPointdPose = np.array( [[1, 0, math.cos(th)], [0, 1,  math.sin(th)]] )

            # (x, y)
            point = robot.lidar_to_map(a,dist)

            # current_M
            M = sdf.interpolate(point)
            current_M += M**2

            # dM/ d(x,y)
            dMdPoint = 2 * M * np.array(sfd.interpolate_derivative(point))

            # not handling sqare!!! ?
            # (x,y,th) = [2, _] @ [2 ,3]
            Map_derivate += dMdPoint @ dPointdPose

        # aprox = current_M + Map_derivate * dPose = 0 
        #  dPose = Map_derivate ^-1 @  (-current_M)
        dPose = np.linalg.pinv( dMdPose[np.newaxis, :] ) * (-current_M)
        return tuple(np.array(self.robot.get_pose())+ dPose[:,0])



class LidarWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root,width=WINDOW_SIDE,height=WINDOW_SIDE)
        self.canvas.pack()

        self.slam = SLAM()

        self.arrow = self.canvas.create_line(0, 0, 1, 1, arrow=tk.LAST)

        self.root.after(RATE,self.update)
        self.root.mainloop()

    def create_point(self,x,y):
        return self.canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

    def create_pose(self,x,y, th):
        return self.canvas.create_line(x, y, x + 10*math.sin(th),
                                        y - 10*math.cos(th), arrow=tk.LAST)

    def create_map(self, sdf):
        pass

    def to_canvas(self,x, y):
        y_new= WINDOW_SIDE/2 - x / MM_PER_PIX
        x_new= WINDOW_SIDE/2 + y / MM_PER_PIX
        return (x_new,y_new)

    def update(self):
        try:
            self.slam.update()

            for row in self.slam.get_map():
                print(row)
            print()

            self.canvas.delete(self.arrow)
            x, y, th = self.slam.get_pose()
            print(self.slam.get_pose())
            self.arrow = self.create_pose(*self.to_canvas(x, y), th )
            
            # self.canvas.delete('all')
            for x,y in self.slam.get_lidar():
                self.create_point( *self.to_canvas(x, y ) )
            print()

        finally:
            self.root.after(RATE,self.update)

if __name__ == "__main__":
    window = LidarWindow()



