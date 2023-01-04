import Rhino.Geometry as rg  #to install in the compas conda environment use pip install Rhino-stubs
import Grasshopper as gh # https://pypi.org/project/Grasshopper-stubs/

from compas.geometry import Curve, Point


class GlobalDesign(object):
    """A global design class

    Attributes:
        name (str): Name of the design.
        layers (:obj:`list` of :obj:`Layer`): list of Layer objects.

    """

    def __init__(self, layers, name):

        self.name = name
        if not layers == None:
            self.layers = layers


    def generate_uid_for_all_parts(self):
        for layer in self.layers:
            for path in layer.paths:
                for part in path.parts:
                    part.generate_uid(layer.index, path.index)

    @classmethod
    def design_from_curve(cls, curve, h_spacing, height, default_layer_height = 40, name = "defaultName"):
        ts = curve.DivideByCount(int(round(curve.GetLength()/h_spacing*2)), True)

        flip_alternating = True if not curve.IsClosed else False

        layers = []
        for index in range(int(height/default_layer_height)):
            #since only one path on each layer path index : 0
            path = [Part(rg.Point3d(curve.PointAt(t).X,  curve.PointAt(t).Y,  index*default_layer_height), i) for i,t in enumerate(ts[index%2::2])]

            if flip_alternating and index % 2 == 0:
                path.reverse()

            paths = [Path(path,0)]
            layers.append(Layer(paths, index))

        design = cls(layers, name)

        return design

    def design_from_curve_legacy(self, curve, h_spacing, height, default_layer_height = 40):
        """single layer curve extrusion structure"""

        self.curve = curve
        self.curve_domain = self.curve.Domain
        self.nurbs_curve = self.curve.ToNurbsCurve(self.curve_domain)
        self.layer_height = default_layer_height
        self.no_layers = no_layers

        self.curves = curve_series_extrusion(self.curve, self.layer_height, self.no_layers)

    def create_layers_simple_extrusion(self):

        self.layers = []

        index = 0
        for curve in self.curves:

            new_layer = Layer(curve, index, self.layer_height)
            self.layers.append(new_layer)
            index = index + 1

    def create_targets_from_design(self, dist):
        self.dist = dist
        self.targets_as_points = []

        for layer in self.layers:
            layer.make_parts()
            self.targets_as_points.extend(layer.points)

    def visualize(self):
        design_pts = []
        for layer in self.layers:
            layer_pts = []
            for path in layer.paths:
                path_pts = []
                for part in path.parts:
                    path_pts.append(part.position)
                layer_pts.append(path_pts)
            design_pts.append(layer_pts)
        return design_pts


class Layer(object):
    """A class for each layer, which could consist of multiple paths

    Attributes:
        index (:obj:`int`): Layer index in a design.
        paths (:obj:`list` of :obj:`Path`): List of paths on one layer.
        height (:obj:`float`): Height in `mm` of the layer.

    """
    def __init__(self, paths, index,  height = 40):
        self.paths = paths
        self.index = index
        self.height = height



class Path(object):
    """A class for each path within a layer

    Attributes:
        index (:obj:`int`): Index of path within a layer.
        parts (:obj:`list` of :obj:`Part`): List of parts on the path.
        curve (:obj:`compas.geometry.Curve`): Curve for the path

    """
    def __init__(self, parts, index = 0, curve = None):
        self.parts = parts
        self.index = index
        self.curve = curve


class Part(object):
    """A class for each part

    Attributes:
        position (:obj:`compas.geometry.Point`): Description of `attr3`.
        index (:obj:`int`): Index of path within a layer.
        shoot (:obj:`bool`): Shooting on/off information.


    """
    def __init__(self, position, index, shoot = True):
        self.position = position
        self.index = index
        self.shoot = shoot
        self.layer_index = None
        self.path_index = None

    def generate_uid(self, layer_index, path_index):
        self.uid = "0%i-0%i-0%i"%(layer_index, path_index, self.index)
        self.layer_index = layer_index
        self.path_index = path_index

def get_points_from_control_points(control_points):
    points = []

    for control_point in control_points:
        point = control_point.Location
        points.append (point)
    return points


def curve_series_extrusion(design_curve, default_layer_height = 40, no_layers= 10):
    domain = design_curve.Domain
    nurbsCurve = design_curve.ToNurbsCurve(domain )

    nurbsPoints = nurbsCurve.Points
    points = get_points_from_control_points(nurbsPoints)


    curves = []

    offset_vector = rg.Vector3d.Multiply(rg.Vector3d(0,0,1), default_layer_height)
    new_curves = []

    for i in range(no_layers):
        new_points = []
        for point in points:
            new_point =point.Add(point, offset_vector)
            new_points.append(new_point)

        #handle case of line line curve
        if len(new_points) <3:
            new_curve = rg.Line(new_points[0], new_points[1])
            new_curve  = new_curve.ToNurbsCurve()
        else:
            new_curve = rg.NurbsCurve.Create(False, 3, new_points)

        points = new_points

        new_curves.append(new_curve)
    return new_curves


def make_points_from_curve(crv, dist):

    domain = crv.Domain
    nurbsCurve = crv.ToNurbsCurve(domain )

    crv = nurbsCurve

    length = crv.GetLength()
    div  = int(length / dist)
    updated_dist= length/div

    params = crv.DivideByCount(div, True)
    params2 = crv.DivideByLength(updated_dist, True)
    points = [crv.PointAt(p) for p in params2]
    return points

def make_alternate_points_from_curve(crv, dist):

    length = crv.GetLength()
    div  = int(length / dist)

    updated_dist = length/div # new distance between parts
    pointsHalf = rg.Curve.DivideByLength(crv, updated_dist/2, True)

    domain = crv.Domain
    newInterval =rg.Interval(pointsHalf[1], pointsHalf[-2])
    newcrv = crv.Trim(newInterval)

    params =newcrv.DivideByCount(div-1, True)
    points = [newcrv.PointAt(p) for p in params]

    return newcrv, points


def make_all_points(crvs, dist):
    """ Make all points on a curve series through divisions """
    allPoints = []

    i = 0
    for i in range(len(crvs)):
        if i%2 == 0:
            points = make_points_from_curve(crvs[i], dist)
            allPoints.extend(points)
        else:
            points = make_alternate_points_from_curve(crvs[i], dist) [1]
            allPoints.extend(points)
    return allPoints
