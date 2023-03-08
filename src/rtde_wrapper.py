from rtde_control import RTDEControlInterface as RTDEControl
from rtde_control import Path, PathEntry
from rtde_io import RTDEIOInterface
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import threading
#from compas.geometry import Frame, Transformation, Translation, Vector, Point
#from compas.robots import Configuration
#import compas
import psutil, sys, os


def get_config(ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    robot_joints = ur_r.getActualQ()
    config = Configuration.from_revolute_values(robot_joints)
    return config

def get_tcp_offset(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    tcp = ur_c.getTCPOffset()
    return tcp

def set_tcp_offset(pose, ip = "127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.setTcp(pose)

def get_tcp_force(ur_r):
    #ur_r = RTDEReceive(hostname = ip, frequency = -0.05)
    forces = ur_r.getActualTCPForce()
    ts = ur_r.getTimestamp()
    return forces,ts


def move_to_joints(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c = RTDEControl(ip)
    ur_c.moveJ(config.joint_values, speed, accel, nowait)

def movel_to_joints(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c = RTDEControl(ip)
    ur_c.moveL_FK(config.joint_values, speed, accel, nowait)

def move_to_target(frame, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    pose = frame.point.x/1000, frame.point.y/1000, frame.point.z/1000, *frame.axis_angle_vector
    ur_c = RTDEControl(ip)
    ur_c.moveL(pose ,speed, accel, nowait)
    return pose

def pick_and_place_async(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist = 100):
    thread = threading.Thread(target=pick_and_place, args=(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist))
    thread.start()

def pick_and_place(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist = 100):
#move to pick safety plane
    if isinstance(pick_frames,Frame):
        pick_frames = [pick_frames]*len(place_frames)

    for pick, place in zip(pick_frames, place_frames):
        move_to_target(pick.transformed(Translation.from_vector(Vector(0,0,safe_dist))), speed, accel, False, ip = ip)
        #move to pick plane
        move_to_target(pick, speed, accel, False, ip = ip)
        #turn IO on
        set_digital_io(vaccum_io,True,ip=ip)
        #sleep on position to give some time to pick up
        time.sleep(0.5)
        #move to pick safety plane
        move_to_target(pick.transformed(Translation.from_vector(Vector(0,0,safe_dist))), speed, accel, False, ip = ip)
        #move to pre placement frame
        pre_place_frame = place.transformed(Translation.from_vector(Vector(0,0,safe_dist)))
        move_to_target(pre_place_frame, speed, accel, False, ip = ip)
        #move to placement frame
        move_to_target(place, speed, accel, False, ip = ip)
        #turn vaccuum off to place brick
        set_digital_io(vaccum_io,False,ip=ip)
        #sleep robot to make sure it is placed
        time.sleep(0.5)
        #move to post placement frame
        post_place_frame = place.transformed(Translation.from_vector(Vector(0,0,safe_dist)))
        move_to_target(post_place_frame, speed, accel, False, ip = ip)

def create_path(frames, velocities, accelerations, radii):
    # speed rad/s, accel rad/s^2, nowait bool
    path = Path()
    for i,(f,v,a,r) in enumerate(zip(frames,velocities,accelerations, radii)):
        pose = f.point.x/1000, f.point.y/1000, f.point.z/1000, *f.axis_angle_vector
        if i == 0 or i >= len(frames)-2:
            r = 0.0
        target = [*pose,v/1000,a/1000, r/1000]
        path.addEntry(PathEntry(PathEntry.MoveL, PathEntry.PositionTcpPose,target))
    return path

def move_to_path(frames, velocities, accelerations, radii, ip = "127.0.0.1", ur_c = None):
    # speed, accel, nowait bool
    path = create_path(frames, velocities, accelerations, radii)
    ur_c.movePath(path, True)
    return path

def turn_extrusion_on(speed, ip):
    set_digital_io(5,False,ip)
    if speed == 0:
        set_digital_io(6,False, ip)
        set_digital_io(7,True, ip)
    elif speed == 1:
        set_digital_io(6,True, ip)
        set_digital_io(7,False, ip)

def send_printpath(frames, velocities, accelerations, radii, toggles, ip = '127.0.0.1'):
    #if no list for accel provided, create a list for it with values repeated
    if isinstance(accelerations,float):
        accelerations = [accelerations]*len(frames)
    if isinstance(velocities,float):
        velocities = [velocities]*len(frames)
    ur_c = RTDEControl(ip)
    exec = move_to_path(frames, velocities, accelerations, radii, ur_c= ur_c)
    waypoint = -1 #initial waypoint
    cur_time = time.time()
    change_toggle = False
    try:
        while ur_c.getAsyncOperationProgress() >= 0:
            new_waypoint = ur_c.getAsyncOperationProgress() #counter index
            if new_waypoint != waypoint:
                counter = new_waypoint-1
                if toggles[counter] != toggles[counter-1] or counter == 0:
                    change_toggle = True
                waypoint = new_waypoint
                print(str(counter/len(frames)) + "%")
                if  change_toggle:
                    if toggles[counter]:
                        turn_extrusion_on(speed = 0,ip= ip)
                    else:
                        set_digital_io(5, True, ip)
                    cur_time = time.time()
        print('done')
        set_digital_io(5, True, ip)
    except KeyboardInterrupt:
        safe_acc = 0.1
        ur_c.stopL(safe_acc)
        set_digital_io(5, True, ip)
        ur_c.stopScript()
        exit()


def stopL(accel, ip = "127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.stopL(accel)

def get_digital_io(signal, ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    return ur_r.getDigitalOutState(signal)

def set_digital_io(signal, value, ip="127.0.0.1"):
    io = RTDEIOInterface(ip)
    io.setStandardDigitalOut(signal, value)

def set_tool_digital_io(signal, value, ip="127.0.0.1"):
    io = RTDEIOInterface(ip)
    io.setToolDigitalOut(signal, value)

def get_tcp_frame(ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    tcp = ur_r.getActualTCPPose()
    frame = Frame.from_axis_angle_vector(tcp[3:], point=tcp[0:3])
    return frame

def move_trajectory(configurations, speed, accel, blend, ur_c):
    path = []
    for config in configurations:
        path.append(config.joint_values + [speed, accel, blend])
    if len(path):
        ur_c.moveJ(path)

def start_teach_mode(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.teachMode()

def stop_teach_mode(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.endTeachMode()



if __name__ == "__main__":
    rtde_r = RTDEReceive("192.168.0.10", 500)
    while(True):
        print(get_tcp_force(rtde_r))

