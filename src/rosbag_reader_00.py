from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.typesys import get_types_from_msg, register_types
import os
import compas
import sys
import pandas as pd
import numpy as np


## reading recorded rosbag
def read_rosbag():
    with Reader(file_path) as reader :
        connections = [x for x in reader.connections if x.topic == '/waypoint_feedback'] ## define topic to read
        ## create dictionary
        typs = {}
        for name, topic in reader.topics.items():
            typs.update(get_types_from_msg(topic.msgdef, topic.msgtype))
        register_types(typs)

        waypoints = []
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            cdrdata = ros1_to_cdr(rawdata, topic.msgtype)
            msg = deserialize_cdr(cdrdata, topic.msgtype)
            
            waypoints.append(msg)

    return waypoints

def parse_waypoint_data(waypoints):
    i = 0
    id = 0
    shoot = 0
    time = 0.0
    follow = True
    pts = {}
    waypoint_data = []
    while(follow):
        if(i>(len(waypoints)-2)):
            follow = False
            break
        pt = {}
        current_pt = waypoints[i]
        next_pt = waypoints[i+1]
        
        if (current_pt.partIdentifier == next_pt.partIdentifier) and (current_pt.shoot == next_pt.shoot) and (current_pt.shoot == True):
            shoot += 1
            time += current_pt.time
        
        pt["index"] = id
        pt["shoot"] = shoot
        pt["identifier"] = current_pt.partIdentifier
        pt["total time"] = time
        
        if (current_pt.partIdentifier != next_pt.partIdentifier):
            waypoint_data.append(pt)
            id += 1
            shoot = 1
            time = current_pt.time

        if shoot <= 1:
            pass #print (id, shoot)
        pts[str(id)] = shoot  
        #print(pt)
        if(i>(len(waypoints)-1)):
            follow = False
            break

        i += 1
    
    return waypoint_data



## writing to json
def write_rosbag_to_json():
    ## parse xyz coordinates and quaternion values to regenerate position of shooting waypoints
    waypts = read_rosbag()
    data = parse_waypoint_data(waypts)

    with open(json_path, 'w'):
        compas.json_dump(data, json_path, pretty = True)

## writing to csv
def write_rosbag_to_csv():
    # take waypoint data and write into csv file
    waypts = read_rosbag()
    data = parse_waypoint_data(waypts)

    
    df = pd.DataFrame.from_records(data)
    print(df)
    with open(csv_path, 'w'):
        pd.DataFrame.to_csv(df, csv_path, ';', index=False)


# write_rosbag_to_json()
if __name__ == "__main__":
    # define path of the recorded rosbag (.bag and .dump)

    path = os.path.join(os.getcwd(), "prints", "rosbags")
    print(path)

    file_path = os.path.join(path, "2023-02-16_11-07-08", "2023-02-16_11-07-08.bag")
    csv_path = os.path.join(path, "2023-02-16_11-07-08", "2023-02-16_11-07-08.csv")

    json_path = os.path.join(path, "2023-02-16_11-07-08", "2023-02-16_11-07-08.json")

    write_rosbag_to_csv()
    write_rosbag_to_json()