import msgpack
import UDPComms
import tkinter as tk
import math
import time

import numpy as np

MM_PER_INCH =2.54

WINDOW_SIDE = 1000
MM_PER_PIX = 4

RESOLUTION = 5
MAP_SIZE_M = 30
RATE = 100


class SDFMap:
    def __init__(self):
        self.size_mm = MAP_SIZE_M * 1000
        self.resolution = RESOLUTION

        self.size_g = int(self.size_mm/self.resolution)

        # TODO: figure out what defaut works
        self.map = [ [0]*self.y_size_g for _ in range(self.size_g)] 

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

        if np.sign(bl) == np.sign(br) == np.sign(tl) == np.sign(tr):
            dx = self.fc(y)* (br - bl) + (1 - self.fc(y))* (tr - tl)  
            dy = self.fc(x)* (tl - bl) + (1 - self.fc(x))* (tr - br)  
            return (dx,dy)

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


class SLAM:
    def __init__(self):
        self.lidar = UDPComms.Subscriber("data",  "4096s", 8110, 1)
        self.robot = Robot()
        self.sdf = SDFMap()

    def update(self):
        scan = self.lidar.get()

        self.robot.update_odom()

        for _ in range(5):
            pose = self.sdf.scan_match(scan)
            self.robot.set_pose(*pose)
        self.update_sdf(scan)

    def get_map(self):
        pass

    def get_pose(self):
        return self.robot.get_pose()

    def get_lidar(self):
        for _, angle, dist in data:
            a = math.radians(angle)
            yield self.robot.lidar_to_map(a, dist)


    def update_sdf(self, scan):
        """ first janky update rule """

        for angle,dist in scan:
            a = math.rad(angle)

        pass

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

        self.arrow = canvas.create_line(0, 0, 1, 1, arrow=tk.LAST)

        self.root.after(RATE,update)
        self.root.mainloop()

    def create_point(self,x,y):
        return self.canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

    def create_pose(self,x,y, th):
        return canvas.create_line(x, y, x + 10*math.sin(th),
                                        y - 10*math.cos(th), arrow=tk.LAST)

    def create_map(self, sdf):
        pass

    def to_canvas(self,x, y):
        y= WINDOW_SIDE/2 - x / MM_PER_PIX
        x= WINDOW_SIDE/2 + y / MM_PER_PIX
        return (x,y)

    def update(self):
        try:

            unpacker = msgpack.Unpacker()
            unpacker.feed(sub.get().data)
            data = unpacker.unpack()

            canvas.delete(self.arrow)

            x, y, th = self.slam.get_pose()
            self.arrow = canvas.pose(*self.to_canvas(x, y), th )
            
            # canvas.delete('all')
            for x,y in self.slam.get_lidar()
                create_point( *self.to_canvas(x, y ) )
            print()

        finally:
            r.after(RATE,update)

if __name__ == "__main__":
    window = LidarWindow()



