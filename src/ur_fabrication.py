from utils import *
from ur_comm import *
from ur_script import *
from math import radians
import Rhino.Geometry as rg

ROBOT_IP = '192.168.10.20'

class UR(object):

    def __init__(self, way_planes, toggles, wait_times, ip = ROBOT_IP):
        """
        Class for communicating with robot
        Args:
            way_planes  : Flattened list of planes in rhino space
            toggles     : Flattended list of toggles for extruder
            wait_times  : Flattened list of wait times for each point
        """
        self.way_planes = way_planes
        self.toggles = toggles
        self.wait_times = wait_times
        self.robot_base = None
        self.robot_frames = []
        self.script = ""

    def tcp(self,x,y,z,rx,ry,rz):
        """
        This function set the TCP for robot
        x,y,z       : TCP dimension, float
        rx,ry,rz    : TCP rotation, float
        """
        self.script += set_tcp_by_angles(float(x),float(y),float(z),radians(rx),radians(ry),radians(rz))

    def set_base_plane(self, origin, x_axis, y_axis):
        """
        This function creates a base plane from measured points for workobject
        origin  : point on base plane
        x_axis  : vector for x axis
        y_axis  : vector for y axis
        """
        self.robot_base = rg.Plane(origin, x_axis - origin, y_axis - origin)
    
    def rhino_planes_to_robot_planes(self):
        rhino_planes = self.way_planes
        for pl in rhino_planes:
            pl = rg.Plane(rg.Point3d(pl.Origin[0],pl.Origin[1],pl.Origin[2]),rg.Vector3d(1,0,0),rg.Vector3d(0,1,0))
            robot_plane = rhino_to_robotbase(pl,self.robot_base)
            self.robot_frames.append(robot_plane)

    def prepare_script(self):
        self.rhino_planes_to_robot_planes()
        for i,(pl, toggle, wait) in enumerate(zip(self.robot_frames, self.toggles, self.wait_times)):
            self.script += move_l(pl,MAX_ACCEL, MAX_VELOCITY)
            self.script += sleep(0.5)
            if toggle:
                self.script += set_digital_out(0,True)
                self.script += sleep(wait)
                self.script += set_digital_out(0,False)
            if i%5 == 0:
                self.script += textmsg('Pt : %d' %i)
    
    
    def trigger(self):
        self.script = concatenate_script(self.script)
        send_script(self.script, ROBOT_IP)

        


            
