import Rhino.Geometry as rg
from compas import json_dumps

ABSOLUTE_Z_LIMIT = 2000

class ConstructionSequencer(object):
    def __init__(self, design, toolpath_volume = 20):
        self.design = design
        self.toolpaths = []
        self.targets_flat = []
        self.toolpath_volume = toolpath_volume

    def generate_toolpaths_continuous(self):
        self.create_targets_from_design()
        
        #create chunks from flat list of targets
        nested_chunks = list(split_list(self.targets_flat, self.toolpath_volume))

        for index, chunk in enumerate(nested_chunks):
            self.toolpaths.append(Toolpath(targets = chunk, index = index))

    def create_targets_from_design(self):
        for layer in self.design.layers:
            for path in layer.paths:
                for part in path.parts:
                    self.targets_flat.append(Target.target_from_part(part))

    def regenerate_toolpaths(self, uid):
        #TO DO: implement method for taking UID and generate next path

        layer_index, path_index, part_index = uid.split("-")

        self.create_targets_from_design() #regenrate targets based on layer and path index
        
        pass



class Toolpath(object):
    def __init__(self, targets, index):
        self.targets = targets
        self.index = index

    def add_safety_points(self, type = None, setup = "Gantry"):
        #Add safety height based on setup
        if setup != "Gantry":
            ABSOLUTE_Z_LIMIT = 250
        else:
            ABSOLUTE_Z_LIMIT = 2100
        if self.targets == None:
            raise Exception("No valid toolpath, please add points before executing this function")
        start_safety_point = Target(rg.Point3d(self.targets[0].position.X, self.targets[0].position.Y, ABSOLUTE_Z_LIMIT), shoot = False, uid = self.targets[0].uid)
        end_safety_point = Target(rg.Point3d(self.targets[-1].position.X, self.targets[-1].position.Y, ABSOLUTE_Z_LIMIT), shoot = False, uid = self.targets[-1].uid)

        if type == 0:
            self.targets.insert(0, start_safety_point)
        elif type == 1:
            self.targets.append(end_safety_point)
        else:
            self.targets.insert(0, start_safety_point)
            self.targets.append(end_safety_point)
        
        print("Safety point added")

    def create_feeding_trajectory(self, feeding_point, start_uid = None):
        pre_feeding_pt = feeding_point.Clone()
        pre_feeding_pt.Z = ABSOLUTE_Z_LIMIT

        last_uid = self.targets[-1].uid 
        self.targets.append(Target(pre_feeding_pt, uid = last_uid))
        self.targets.append(Target(feeding_point, uid = last_uid))

        if self.index > 0:
            next_uid = self.targets[0].uid
            self.targets.insert(0,Target(pre_feeding_pt, next_uid))

    
    
    def export_toolpath_to_ROS(self, frame = "map_o3d", prototype_name = "Default"):
        num_of_points = len(self.targets)

        msg_dictionary = {}
        msg_dictionary['frame'] = frame
        msg_dictionary['prototypeName'] = prototype_name
        #msg_dictionary['header'] = {'seq': 0, 'frame_id': 'map', 'stamp': {'nsecs': 0, 'secs': 0}}
        msg_dictionary['waypoints']=  []

        for index,target in enumerate(self.targets):
            #new_point = {'position.z': z_coordinates_desired[i], 'position.y': y_coordinates_desired[i], 'positionx': x_coordinates_desired[i]}
            new_pose =  format_target_for_ROS(target, index)
            msg_dictionary['waypoints'].append(new_pose)

        jsonMsg = json_dumps(msg_dictionary)
        return msg_dictionary, jsonMsg
    
    def export_toolpath_to_UR(self, wait_time = 1.0):
        data = {}

        data['planes'] = []
        data['toggles'] = []
        data['wait times'] = []

        for target in self.targets:
            data['planes'].append(rg.Plane(target.position, rg.Vector3d.XAxis, rg.Vector3d.YAxis))
            data['toggles'].append(target.shoot)
            data['wait times'].append(wait_time)
    
        return data
    
    def visualize_pts(self):
        points = [target.position for target in self.targets]
        polyline = rg.Polyline(points)
        return points, polyline.ToPolylineCurve()
    



class Target(object):
    def __init__(self, position, shoot = False,  extruder_speed = 25, belt_speed = 27, uid = None):
        self.shoot = shoot
        self.part_size = 0.0
        self.time = 0.0
        self.extruder_speed = extruder_speed
        self.belt_speed = belt_speed
        self.position = position
        self.uid = uid
        self.layer_index = 0
        self.plane = rg.Plane(self.position, rg.Vector3d.XAxis, rg.Vector3d.YAxis)


    @classmethod
    def target_from_part(cls, part):
        target = cls(position = part.position, shoot = part.shoot, uid = part.uid)
        target.layer_index = part.layer_index
        return target



def get_quaternion_rhino(RhinoPlane, base_plane):
    quaternion = rg.Quaternion.Rotation(base_plane, RhinoPlane)
    xq = quaternion.A
    yq = quaternion.B
    zq = quaternion.C
    wq = quaternion.D
    return quaternion

def format_target_for_ROS(target, index):
    """This method formats a plane using Rhino Common"""

    target_dict = {}
    target_dict['layer'] = target.layer_index # index int
    target_dict['toolpathIndex'] = index # index int
    target_dict['partIndex'] = 0 # index int???
    target_dict['partIdentifier'] = target.uid  #  uniqueidentifier string ... this is becuase when we reoutput toolpaths parts will shift
    target_dict['shoot'] = target.shoot
    target_dict['position'] = {'x': target.plane.Origin.X, 'y' : target.plane.Origin.Y, 'z': target.plane.Origin.Z}

    #Qwhat is base plane?
    quaternion = get_quaternion_rhino(target.plane, rg.Plane.WorldXY)
    target_dict['orientation'] = {'x': quaternion.A,   'y': quaternion.B,  'z': quaternion.C, 'w': quaternion.D}

    #target_dict['orientation'] : {'x': 0.0,   'y': 0.0,  'z': 0, 'w': 0}
    target_dict['time'] = target.time
    target_dict['particleSize'] =  target.part_size
    target_dict['extruderSpeed'] = target.extruder_speed
    target_dict['beltSpeed']=  target.belt_speed

    return target_dict

def split_list(in_list, chunk_size):
  for i in range(0, len(in_list), chunk_size):
    yield in_list[i:i + chunk_size]
