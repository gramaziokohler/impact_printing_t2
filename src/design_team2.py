import Rhino.Geometry as rg  #to install in the compas conda environment use pip install Rhino-stubs
import Grasshopper as gh # https://pypi.org/project/Grasshopper-stubs/

from compas.geometry import Curve, Point
from ghpythonlib.treehelpers import list_to_tree as _tree
from ghpythonlib.treehelpers import tree_to_list as _tl


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
    def design_from_curve_with_Openings(cls,wall_type, points,openings, layers_num,v_spacing, b_list, heights, name = "defaultName", alternating=True, Safety=False, Support=False, below=True, Footer=False ):
        layers = []
        cnt_footer=0
        points_in_open=0
        for index in range(layers_num):
            parts=[]
            path_index=0
            if wall_type== "Single_layer":
                for i,p in enumerate(points[index%2::2]):
                    h=index*v_spacing
                    if h<= heights[i]:
                        part_position = rg.Point3d(p.X,  p.Y, h)
                    else:
                        continue

                    #check if it is part of an opening ////////////////////////////////////
                    part_inhole = PointInsideOpening(part_position, openings)
                    if part_inhole==False: 
                        #check if it is part of the frame's support////////////////////////
                        part_support, f = IsPointSupport(part_position,openings)
                        parts.append(Part(part_position,i, path_index, part_support))
                        points_in_open=0
                    else:
                        points_in_open+=1
                        if points_in_open==1:
                            path_index+=1
            elif wall_type=="Bifurcated":
                for i,p in enumerate(points):
                    h=index*v_spacing
                    if h<= heights[i]:
                        part_position = rg.Point3d(p.X,  p.Y, h)
                    else:
                        continue
                    vec=rg.Vector3d.XAxis
                    if (i+index)%2==0:
                        T= rg.Transform.Translation(vec*b_list[i])
                        part_position.Transform(T)
                    else:
                        T=rg.Transform.Translation(vec*b_list[i]*-1)
                        part_position.Transform(T)

                    #check if it is part of an opening ////////////////////////////////////
                    part_inhole = PointInsideOpening(part_position, openings)
                    if part_inhole==False: 
                        #check if it is part of the frame's support////////////////////////
                        part_support, f = IsPointSupport(part_position,openings)
                        parts.append(Part(part_position,i, path_index, part_support))
                        points_in_open=0
                    else:
                        points_in_open+=1
                        if points_in_open==1:
                            path_index+=1

            paths=[Path(parts)]

            UpdateOpeningsStatus(index*v_spacing, openings)
            openings=UpdateOpenings(openings)
            highest=HighestFrame(openings)

            if Support:
                    parts = AddSupport(curve, index, i, points, part_position,f,d, parts,  part_index)
                    part_index+=2      
            # if index%2==1 and alternating:          #zig zag sequence
            #     if len(paths)!=1:
            #         paths.reverse()
            #         if below and path.FrameToAvoid_Start==None:
            #             for p in paths:
            #                 p.parts.reverse()
            #     else:
            #         paths[0].parts.reverse()
            
            new_layer= Layer(paths,index, highest)
            
            #check if we are in the level of the footer ///////////
            # if Footer and IsFooterTime(index*default_layer_height, openings):
            #     if cnt_footer<foo:
            #         new_layer.paths = [new_layer.LayerwithFooter()]
            #         cnt_footer+=1
            
            layers.append(new_layer)
            
        #for layer in layers:
            #for path in layer.paths:
                #print (layer.index,"/", path.index, "//",path.FrameToAvoid_Start, "//", path.FrameToAvoid_End)
        design = cls(layers, name)

        return design


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


class Opening(object):
    """A class for each opening"""
    
    def __init__(self,crv,wall_type, origin,h_spacing,v_spacing,lock_spacing,bifurcation_thickness, scale_factor, points, index=0):
        self.crv=crv
        self.initialorigin=origin
        self.h_sp=h_spacing
        self.v_sp= v_spacing
        self.factor = scale_factor
        self.origin, self.b_index = self.AlignOrigin(points) # b_index is used only for the case of bifurcated
        self.control_points = self.ControlPoints(sc_fac=self.factor+1)
        self.outline=rg.Polyline(self.control_points)
        self.visline= rg.Polyline(self.ControlPoints())
        self.brep = self.OpeningBrep( wall_type, lock_spacing, bifurcation_thickness)
        self.index = index
        self.avoid_height=self.AvoidFrameHeight()
        
        self.status=0 #[0: Before Placing a Frame // 1: With Frame but not completly Covered // 2: Fully Covered]
        
        self.column_index=((self.origin.Y- crv.PointAt(0).Y)/(self.h_sp/2))
        self.support=self.Support()

    def AlignOrigin(self,points):
        pt= self.initialorigin.Clone()
        pt.Z=0
        closest_pts, closest_index= two_closest_points(pt, points)
        if closest_index[0]%2==0:
            even=closest_pts[0]
            even_index=closest_index[0]
            odd=closest_pts[1]
            odd_index=closest_index[1]
        else:
            even=closest_pts[1]
            even_index=closest_index[1]
            odd=closest_pts[0]
            odd_index=closest_index[0]
        newZ= round(self.initialorigin.Z/self.v_sp)* self.v_sp
        pt.Z= newZ
        layer= (newZ/(self.v_sp)%2) # even (0) or odd (1) layer
        or_position= self.factor%2 # if origin point should be between two points (1) or not (0)
        if (layer+or_position)%2==0:
            pt.X=even.X
            pt.Y=even.Y
            cl_index=even_index
        else:
            pt.X=odd.X
            pt.Y=odd.Y
            cl_index=odd_index
        return pt, cl_index
        
    def ControlPoints(self,sc_fac=0):
        #to do: tangent
        if sc_fac==0:
            sc_fac=self.factor
        pt0= rg.Point3d(self.origin.X,self.origin.Y -(self.h_sp*sc_fac)/2, self.origin.Z)
        pt1= rg.Point3d(self.origin.X,self.origin.Y, self.origin.Z+(self.v_sp*sc_fac))
        pt2= rg.Point3d(self.origin.X,self.origin.Y+(self.h_sp*sc_fac)/2, self.origin.Z)
        pt3= rg.Point3d(self.origin.X,self.origin.Y, self.origin.Z-(self.v_sp*sc_fac))
        
        return [pt0, pt1, pt2, pt3, pt0]
        
    def OpeningBrep(self, wall_type, lock_spacing, bifurcation_thickness):
        dist=100
        if wall_type=="Bifurcated":
            dist += bifurcation_thickness+lock_spacing
        T= rg.Transform.Translation(rg.Vector3d.XAxis*dist)
        temp1= self.outline.Duplicate().ToNurbsCurve()
        temp1.Transform(T)
        temp2= self.outline.Duplicate().ToNurbsCurve()
        T1= rg.Transform.Translation(rg.Vector3d.XAxis*-dist)
        temp2.Transform(T1)
        
        cap1= rg.Brep.CreatePlanarBreps(temp1)[0]
        cap2= rg.Brep.CreatePlanarBreps(temp2)[0]
        loft= rg.Brep.CreateFromLoft([temp1,temp2], rg.Point3d.Unset, rg.Point3d.Unset,0, False)[0]
        cap_br= rg.Brep.CreateSolid([cap1,loft,cap2],0.001)[0]
        
        return cap_br
        
    def Support(self):
        tol=2
        interval=[]
        sc=(self.factor/2)
        y_int1=[round(self.origin.Y- sc*self.h_sp-tol),self.origin.Y-tol ]
        y_int2=[self.origin.Y+ 0.5*self.h_sp+tol,round(self.origin.Y+ sc*self.h_sp+tol) ]
        interval.append(y_int1)
        interval.append(y_int2)
        interval.append(self.origin.Z)
        
        return interval
        
    def Tip(self):
        higher_point =self.control_points[0]
        for point in self.control_points:
            if point.Z > higher_point.Z:
                higher_point=point
        return higher_point
    
    def Bottom_Tip(self):
        lowest_point =self.control_points[0]
        for point in self.control_points:
            if point.Z < lowest_point.Z:
                lowest_point=point
        return lowest_point
    
    def AvoidFrameHeight(self):
        safety_distance =50
        tip= self.Tip()
        return tip.Z+ safety_distance
    
    def FrameSize(self):
        fr_size= self.control_points[0].DistanceTo(self.control_points[1])
        return round(fr_size)

class Layer(object):
    """A class for each layer, which could consist of multiple paths

    Attributes:
        index (:obj:`int`): Layer index in a design.
        paths (:obj:`list` of :obj:`Path`): List of paths on one layer.
        height (:obj:`float`): Height in `mm` of the layer.

    """
    def __init__(self, paths, index,FrameToAvoid=None, Footer=False, height = 40):
        self.paths = paths
        self.index = index
        self.height = height
        self.insert_frame= False
        self.HasFooter=Footer
        self.FrameToAvoid=FrameToAvoid
        
    def LayerwithFooter(self):
        self.paths[0].parts.reverse()
        new_path = Path(twolists(self.paths[0].parts, self.paths[1].parts),0)
        return new_path

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
        
        self.FrameToAvoid_Start=None
        self.FrameToAvoid_End= None
        
        self.HasFooter= False
        self.safety_distance=50
        
    def AddSafetyPoints(self):
        if self.FrameToAvoid_Start==None:
            safety_start= self.parts[0].position.Z + self.safety_distance 
        else:
            safety_start= self.FrameToAvoid_Start.avoid_height
        
        if self.FrameToAvoid_End==None :
            safety_end= self.parts[-1].position.Z + self.safety_distance
        else:
            safety_end= self.FrameToAvoid_End.avoid_height 
            
        p_start= Part(rg.Point3d(self.parts[0].position.X, self.parts[0].position.Y, safety_start), index=-1, shoot = False)
        self.parts.insert(0, p_start)
        
        p_end= Part(rg.Point3d(self.parts[-1].position.X, self.parts[-1].position.Y, safety_end), index=-1, shoot = False)
        self.parts.append(p_end)
        return
        
class Part(object):
    """A class for each part
    Attributes:
        position (:obj:`compas.geometry.Point`): Description of `attr3`.
        index (:obj:`int`): Index of path within a layer.
        shoot (:obj:`bool`): Shooting on/off information.
    """
    def __init__(self, position, index=0, path_index=None, layer_index=None,part_support=False, shoot = True):
        self.position = position
        self.index = index
        self.shoot = shoot
        self.layer_index = layer_index
        self.path_index = path_index
        self.IsSupport= part_support

    def generate_uid(self, layer_index, path_index):#####?????????#########
        self.uid = "0%i-0%i-0%i"%(layer_index, path_index, self.index)
        self.layer_index = layer_index
        self.path_index = path_index

def divide_crv(crv,h_spacing,minor,major,tol):
    if crv.IsLinear():
        ts = crv.DivideByCount(int(round(crv.GetLength()/h_spacing*2)), True)
        points =[rg.Point3d(crv.PointAt(t).X,  crv.PointAt(t).Y,  0) for t in ts]
    else:
        ts, points =ArrayEllipsesonCrv(crv,minor,major,tol)
    return points

def ArrayEllipsesonCrv(crv, minor, major,tol): 
    p=0
    param=[]
    centers=[]
    el= rg.Ellipse(rg.Plane(rg.Point3d(0,0,0), rg.Vector3d.ZAxis), major, minor)
    
    for i in range(500):
        param.append(p)
        c= crv.PointAt(p)
        centers.append(c)
        el.Center=c
        inter= rg.Intersect.Intersection.CurveCurve(crv,el.ToNurbsCurve(),0.001, 0.001)
        n= len(inter)-1
        
        if n<0:
            print ("Curve is too short or part size too big")
            break
            
        inter_point= inter[n].PointA
        inter_par = inter[n].ParameterA
        
        rad= c.DistanceTo(inter_point)
        cir= rg.Circle(inter_point,rad+tol).ToNurbsCurve()
        
        c_inter= rg.Intersect.Intersection.CurveCurve(crv,cir,0.001, 0.001)
        c_n=len(inter)-1
        
        if c_n<=0 and not p==0:
            print ("End of curve")
            break
        
        p= inter[n].ParameterA
    return param,centers

def fix_index(list_objects):
    for i, obj in enumerate(list_objects):
        if  not obj.index == i: 
            obj.index= i

def twolists(list1, list2):
    newlist = []
    a1 = len(list1)
    a2 = len(list2)

    for i in range(max(a1, a2)):
        if i < a1:
            newlist.append(list1[i])
        if i < a2:
            newlist.append(list2[i])

    return newlist

def match_list_lengths(list1,list2):
    "This function matches the length of the list2 with the length of the list1"
    difference= len(list1)-len(list2)
    if difference>0:
        last= list2[-1]
        list2.extend([last]*difference)
    elif difference<0:
        list2= list2[:len(list1)]
    return list1,list2

def two_closest_points(pt, list_pts):
    "This function takes as inputs  a point and a list of points and returns thw two closest points of this lists to the points and their indexes "
    min_dist1=10**10
    min_index1=-1
    min1=None
    min_dist2=10**10
    min_index2=-1
    min2=None

    for i, p in enumerate(list_pts):
        dist= pt.DistanceTo(p)
        if dist< min_dist1:
            min_dist1=dist
            min1=p
            min_index1=i
        elif dist< min_dist2:
            min_dist2=dist
            min2=p
            min_index2=i
    
    if min1==None or min2==None:
        print ("Point too far away")
    
    return [min1,min2],[min_index1,min_index2]

def Update_Number_Paths(Avoid,layers,end_part,paths, path):
    """Splits a Path into two Sub-Paths and Reverses the direction of the first path and adds safety points in the beginning and the start"""
    sub_paths=[]
    
    if len(paths)==0 or (len(path.parts)<3): # first path of each layer or a path with 2 or less parts
        sub_path= path
        if Avoid: sub_path.AddSafetyPoints()
        sub_paths.append(sub_path)
        
    elif end_part and len(paths)!=0:     # last path of a layer with 2 or more paths 
        sub_path =path
        if path.FrameToAvoid_Start!=None: sub_path.parts.reverse()
        fix_index(sub_path.parts)
        if Avoid: sub_path.AddSafetyPoints()
        sub_paths.append(sub_path)
        
    elif len(path.parts)>=3 :              # rest of the cases
        middle= int(round(len(path.parts)/2))
        
        first_half=path.parts[:middle]
        first_half.reverse()
        fix_index(first_half)
        first_path = Path(first_half,path.index)
        first_path.FrameToAvoid_End=None
        if Safety: first_path.AddSafetyPoints()
        sub_paths.append(first_path)
        
        second_half= path.parts[middle:]
        fix_index(second_half)
        second_path = Path(second_half,path.index+1)
        second_path.FrameToAvoid_Start=None
        if Safety:second_path.AddSafetyPoints()
        sub_paths.append(second_path)
    return sub_paths

def create_Openings(crv,wall_type, origin,h_spacing,v_spacing,lock_spacing,bifurcation_thickness, scale_factor,points):
    openings=[]
    for i, orig in enumerate (origin):
        op= Opening(crv,wall_type, orig,h_spacing,v_spacing,lock_spacing,bifurcation_thickness, scale_factor[i],points,i)
        openings.append(op)
    return openings

def PointInsideOpening(point, openings):
    """checks if a point is inside one of the Openings
    if true and its frame has been placed (status==1) then returns also the opening"""
    if len(openings)==0 : return False
    pointInhole= False
    for opening in openings:
        if opening.brep.IsPointInside(point, 0.00, True):
            pointInhole= True
            if opening.status==1:
                break
            
    return pointInhole

def IsPointSupport(point,openings):
    if len(openings)==0 : return False, None
    pointIsSupport=False
    for op in openings:
        if point.Z< op.support[-1]:
            if (point.Y>op.support[0][0] and point.Y<=op.support[0][-1]):
                return True,1
            elif (point.Y>op.support[1][0] and point.Y<=op.support[1][-1]):
                return True, -1
    return pointIsSupport, None

def AddSupport(curve, index, i, ts, part_position,f, d, parts, part_index):
    if f>0:
        if index%2==0:
            extra= i*2+1
        else:
            extra= i*2+2
    else:
        if index%2==0:
            extra=i*2-1
        else:
            extra= i*2
    extra_fr= rg.Point3d(curve.PointAt(ts[extra]).X+d,  curve.PointAt(ts[extra]).Y,  part_position.Z)
    extra_bc= rg.Point3d(curve.PointAt(ts[extra]).X-d,  curve.PointAt(ts[extra]).Y,  part_position.Z)
    if f>0:
        parts.append(Part(extra_fr,part_index))
        parts.append(Part(extra_bc,part_index+1))
    else:
        parts.insert(-1,Part(extra_fr,part_index))
        parts.insert(-1,Part(extra_bc,part_index+1))
    return parts

def AvoidFrames(avoid_opening,layers, paths, path):
    if len(paths)==0:
        if len(layers)!=0:
            frame=layers[-1].paths[-1].FrameToAvoid_End
            if  frame!= None:
                path.FrameToAvoid_Start=frame
    else:
        fr= paths[-1].FrameToAvoid_End
        if fr!=None:
            path.FrameToAvoid_Start = fr
    if avoid_opening!=None:
        path.FrameToAvoid_End = avoid_opening

def UpdateOpeningsStatus(built_height, openings):
    if len(openings)==0 : return
    for opening in openings:
        if opening.status==2:
            pass
        elif opening.status==1 and built_height> opening.Tip().Z:
            opening.status=2
        elif opening.status==0 and built_height> opening.origin.Z:
            opening.status=1
            #layer.insert_frame= True
            print ("Insert the frame for the opening no." + str(opening.index))

def UpdateOpenings(openings):
    if len(openings)==0 : return []
    unfinished=[]
    for opening in openings:
        if opening.status!=2:
            unfinished.append(opening)
    return unfinished

def HighestFrame(openings):
    if len(openings)==0 : return None
    highest=None
    highest_tip =0
    for opening in openings:
        if opening.status==1:
            tip =opening.Tip().Z
            if tip> highest_tip:
                highest=opening
                highest_tip= tip
    return highest

def IsFooterTime(built_height, openings):
    if len(openings)==0 : return False
    for opening in openings:
        if opening.status==0:
            lowest =opening.Bottom_Tip().Z
            if lowest< built_height-60:
                return True

